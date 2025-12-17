import { Hono } from 'hono';
import { requireAuth, getAuthContext } from '../middleware/auth.js';

const translate = new Hono();

// Rate limiting store (in-memory)
const rateLimitStore = new Map<string, { count: number; resetTime: number }>();
const RATE_LIMIT_MAX = 10; // requests per minute
const RATE_LIMIT_WINDOW = 60 * 1000; // 1 minute in ms

/**
 * Check rate limit for user
 */
function checkRateLimit(userId: string): { allowed: boolean; retryAfter?: number } {
  const now = Date.now();
  const userLimit = rateLimitStore.get(userId);

  if (!userLimit || now > userLimit.resetTime) {
    // Reset or create new window
    rateLimitStore.set(userId, { count: 1, resetTime: now + RATE_LIMIT_WINDOW });
    return { allowed: true };
  }

  if (userLimit.count >= RATE_LIMIT_MAX) {
    const retryAfter = Math.ceil((userLimit.resetTime - now) / 1000);
    return { allowed: false, retryAfter };
  }

  // Increment count
  userLimit.count++;
  return { allowed: true };
}

/**
 * Translate text to Urdu using Google Translate API (free endpoint)
 */
async function translateWithGoogle(text: string): Promise<string> {
  try {
    // Skip empty or whitespace-only text
    if (!text || !text.trim()) {
      return text;
    }

    console.log('[Translate] Calling Google Translate API for text length:', text.length);
    const url = `https://translate.googleapis.com/translate_a/single?client=gtx&sl=en&tl=ur&dt=t&q=${encodeURIComponent(text)}`;

    const response = await fetch(url);

    if (!response.ok) {
      console.error('[Translate] Google API returned status:', response.status);
      return text;
    }

    const data = await response.json();

    // Extract translated text from response
    // Google returns nested arrays: [[["translated text", "original text", ...], ...], ...]
    let translatedText = '';
    if (data && Array.isArray(data[0])) {
      for (const item of data[0]) {
        if (Array.isArray(item) && item[0]) {
          translatedText += item[0];
        }
      }
    }

    console.log('[Translate] Google returned:', translatedText.substring(0, 200));
    return translatedText || text;
  } catch (error) {
    console.error('[Translate] Google Translate error:', error);
    return text; // Return original on error
  }
}

/**
 * Translate content to Urdu while preserving code blocks and structure
 * Uses line-by-line approach to avoid placeholder corruption
 */
async function translateToUrdu(content: string): Promise<string> {
  const lines = content.split('\n');
  const result: string[] = [];
  let inCodeBlock = false;
  let inFrontmatter = false;
  let frontmatterCount = 0;

  // Collect translatable text and translate in batches
  const translatableLines: { index: number; text: string }[] = [];

  for (let i = 0; i < lines.length; i++) {
    const line = lines[i];
    const trimmed = line.trim();

    // Track frontmatter
    if (trimmed === '---') {
      frontmatterCount++;
      if (frontmatterCount === 1) {
        inFrontmatter = true;
      } else if (frontmatterCount === 2) {
        inFrontmatter = false;
      }
      result.push(line);
      continue;
    }

    // Skip frontmatter content
    if (inFrontmatter) {
      result.push(line);
      continue;
    }

    // Track code blocks
    if (trimmed.startsWith('```')) {
      inCodeBlock = !inCodeBlock;
      result.push(line);
      continue;
    }

    // Skip code block content
    if (inCodeBlock) {
      result.push(line);
      continue;
    }

    // Skip empty lines
    if (!trimmed) {
      result.push(line);
      continue;
    }

    // This line needs translation
    translatableLines.push({ index: i, text: line });
    result.push(`__TRANSLATE_${i}__`); // Placeholder for now
  }

  // Translate all lines in batches (max 2000 chars per batch to avoid URL length limits)
  const BATCH_SIZE = 1500;
  let currentBatch: { index: number; text: string }[] = [];
  let currentLength = 0;
  const batches: { index: number; text: string }[][] = [];

  for (const item of translatableLines) {
    if (currentLength + item.text.length > BATCH_SIZE && currentBatch.length > 0) {
      batches.push(currentBatch);
      currentBatch = [];
      currentLength = 0;
    }
    currentBatch.push(item);
    currentLength += item.text.length;
  }
  if (currentBatch.length > 0) {
    batches.push(currentBatch);
  }

  // Translate each batch
  const translations = new Map<number, string>();

  for (const batch of batches) {
    // Join with newlines so Google preserves line structure
    const batchText = batch.map(item => item.text).join('\n');
    const translated = await translateWithGoogle(batchText);
    const translatedLines = translated.split('\n');

    // Map translations back to original indices
    for (let j = 0; j < batch.length; j++) {
      const originalIndex = batch[j].index;
      const translatedLine = translatedLines[j] || batch[j].text;
      translations.set(originalIndex, translatedLine);
    }

    // Small delay between batches to avoid rate limiting
    if (batches.indexOf(batch) < batches.length - 1) {
      await new Promise(resolve => setTimeout(resolve, 100));
    }
  }

  // Replace placeholders with translations
  for (let i = 0; i < result.length; i++) {
    const placeholder = `__TRANSLATE_${i}__`;
    if (result[i] === placeholder) {
      result[i] = translations.get(i) || lines[i];
    }
  }

  return result.join('\n');
}


// Apply auth middleware to all translate routes
translate.use('/translate', requireAuth);

// POST /api/translate - Translate chapter content to Urdu
translate.post('/translate', async (c) => {
  try {
    const authContext = getAuthContext(c);
    if (!authContext) {
      return c.json({
        success: false,
        error: 'AUTH_REQUIRED',
        message: 'Authentication required to translate content',
      }, 401);
    }

    // Check rate limit
    const rateLimit = checkRateLimit(authContext.user.id);
    if (!rateLimit.allowed) {
      return c.json({
        success: false,
        error: 'RATE_LIMITED',
        message: 'Too many translation requests. Please try again later.',
        retry_after: rateLimit.retryAfter,
      }, 429);
    }

    // Parse request body
    const body = await c.req.json();

    // Validate chapter_id
    if (!body.chapter_id || typeof body.chapter_id !== 'string' || body.chapter_id.trim() === '') {
      return c.json({
        success: false,
        error: 'INVALID_REQUEST',
        message: 'Chapter ID is required',
        field: 'chapter_id',
      }, 400);
    }

    // Validate content
    if (!body.content || typeof body.content !== 'string' || body.content.trim() === '') {
      return c.json({
        success: false,
        error: 'INVALID_REQUEST',
        message: 'Content is required',
        field: 'content',
      }, 400);
    }

    // Validate content length (max 100,000 characters)
    if (body.content.length > 100000) {
      return c.json({
        success: false,
        error: 'CONTENT_TOO_LONG',
        message: 'Content exceeds maximum length of 100,000 characters',
      }, 400);
    }

    // Log translation request
    console.log(`[Translate] Request from user ${authContext.user.id} for chapter: ${body.chapter_id}, timestamp: ${new Date().toISOString()}`);

    // Perform translation (async with Google Translate)
    const translatedContent = await translateToUrdu(body.content);
    const wordCount = body.content.split(/\s+/).filter(Boolean).length;

    return c.json({
      success: true,
      translated_content: translatedContent,
      chapter_id: body.chapter_id,
      word_count: wordCount,
      timestamp: new Date().toISOString(),
    });

  } catch (error) {
    console.error('[Translate] Error:', error);
    return c.json({
      success: false,
      error: 'TRANSLATION_FAILED',
      message: 'Translation service encountered an error. Please try again.',
      retry_after: 5,
    }, 500);
  }
});

export default translate;
