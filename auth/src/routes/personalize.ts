import { Hono } from 'hono';
import { eq } from 'drizzle-orm';
import { readFile } from 'fs/promises';
import { join, resolve } from 'path';
import { db } from '../db/client.js';
import { userProfiles } from '../db/schema.js';
import { requireAuth, getAuthContext } from '../middleware/auth.js';
import type { ProfileResponse } from '../types/api.js';

const personalize = new Hono();

// =============================================================================
// Rate Limiting (TASK-002)
// =============================================================================

const rateLimitStore = new Map<string, { count: number; resetTime: number }>();
const RATE_LIMIT_MAX = 5; // 5 requests per minute (more restrictive than translate)
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

// =============================================================================
// Profile Validation (TASK-004)
// =============================================================================

interface ProfileValidationResult {
  valid: boolean;
  missingFields: string[];
}

/**
 * Validate that user profile has all required fields for personalization
 */
function validateProfileCompleteness(profile: ProfileResponse | null): ProfileValidationResult {
  if (!profile) {
    return { valid: false, missingFields: ['profile'] };
  }

  const missingFields: string[] = [];

  // Software background
  if (!profile.software_background?.experience_level) {
    missingFields.push('experience_level');
  }
  if (!profile.software_background?.programming_languages?.length) {
    missingFields.push('programming_languages');
  }
  if (!profile.software_background?.frameworks_platforms?.length) {
    missingFields.push('frameworks_platforms');
  }

  // Hardware background
  if (!profile.hardware_background?.device_type) {
    missingFields.push('device_type');
  }
  if (!profile.hardware_background?.operating_system) {
    missingFields.push('operating_system');
  }
  if (!profile.hardware_background?.system_capability) {
    missingFields.push('system_capability');
  }

  return {
    valid: missingFields.length === 0,
    missingFields,
  };
}

// =============================================================================
// Chapter Content Fetching (TASK-005)
// =============================================================================

/**
 * Get the path to chapter content directory
 */
function getChapterContentPath(): string {
  // Use environment variable or default to relative path from auth service
  return process.env.CHAPTER_CONTENT_PATH || resolve(process.cwd(), '..', 'book-write', 'docs');
}

/**
 * Fetch chapter content from filesystem
 * Supports both `chapter-01.md` and `chapter-01/index.md` patterns
 */
async function fetchChapterContent(chapterId: string): Promise<string | null> {
  const basePath = getChapterContentPath();

  // Sanitize chapter ID to prevent directory traversal
  const sanitizedId = chapterId.replace(/\.\./g, '').replace(/^\/+/, '');

  // Try different file patterns
  const patterns = [
    join(basePath, `${sanitizedId}.md`),
    join(basePath, `${sanitizedId}.mdx`),
    join(basePath, sanitizedId, 'index.md'),
    join(basePath, sanitizedId, 'index.mdx'),
  ];

  for (const filePath of patterns) {
    try {
      const content = await readFile(filePath, 'utf-8');
      // Strip frontmatter
      const frontmatterMatch = content.match(/^---\n[\s\S]*?\n---\n/);
      if (frontmatterMatch) {
        return content.slice(frontmatterMatch[0].length);
      }
      return content;
    } catch {
      // File doesn't exist, try next pattern
      continue;
    }
  }

  return null;
}

// =============================================================================
// Google Gemini API Integration (TASK-006)
// =============================================================================

/**
 * System prompt for personalization
 */
const PERSONALIZATION_SYSTEM_PROMPT = `You are a content personalization assistant for a Physical AI & Humanoid Robotics textbook.

Your task is to adapt chapter content based on the user's background while following these rules:

MUST DO:
- Adjust explanation depth based on experience level
- Use wording appropriate for the experience level
- Add relevant context based on known programming languages
- Include hardware-aware notes based on system capability
- Preserve the exact document structure (headings, sections, lists)

MUST NOT:
- Change any code blocks (preserve exactly as-is, including all \`\`\` fenced blocks)
- Modify command-line examples
- Alter technical term definitions
- Introduce new topics not in the original
- Change factual meaning of any content
- Modify URLs or links
- Alter Mermaid diagram code (anything inside \`\`\`mermaid blocks)
- Add or remove headings

EXPERIENCE LEVEL ADAPTATIONS:
- beginner: Simpler language, more context, step-by-step guidance, explain acronyms
- intermediate: Balanced explanations, assume basic knowledge, practical focus
- advanced: Concise technical language, skip basics, focus on advanced concepts
- expert: Direct technical prose, assume deep domain knowledge, reference edge cases

PROGRAMMING LANGUAGE ADAPTATIONS:
- If user knows Python: Reference Python idioms and patterns where applicable
- If user knows C/C++: Add memory/performance considerations where relevant
- If user knows JavaScript: Draw parallels to async/event-driven patterns
- If user knows Java: Reference OOP concepts familiar to Java developers

HARDWARE ADAPTATIONS:
- low capability: Add warnings about resource-heavy operations, suggest lightweight alternatives
- medium capability: Balance between features and performance
- high capability: Mention GPU acceleration and parallel processing options
- embedded device type: Emphasize embedded-friendly approaches, mention Jetson/RPi specifics

OUTPUT FORMAT:
Return ONLY the personalized markdown content. Do not include explanations, meta-commentary, or wrap in code blocks.`;

/**
 * Build user prompt with profile context
 */
function buildUserPrompt(content: string, profile: ProfileResponse): string {
  const { software_background, hardware_background } = profile;

  return `Personalize the following chapter content for a user with this background:

**Experience Level**: ${software_background.experience_level}
**Programming Languages**: ${software_background.programming_languages.join(', ')}
**Frameworks/Platforms**: ${software_background.frameworks_platforms.join(', ')}
**Device Type**: ${hardware_background.device_type}
**Operating System**: ${hardware_background.operating_system}
**System Capability**: ${hardware_background.system_capability}

---

CHAPTER CONTENT:

${content}`;
}

/**
 * Personalize content using Google Gemini API
 */
async function personalizeWithGemini(
  content: string,
  profile: ProfileResponse
): Promise<{ personalizedContent: string; adjustmentsMade: string[] }> {
  const apiKey = process.env.GEMINI_API_KEY;

  if (!apiKey) {
    throw new Error('GEMINI_API_KEY environment variable is not set');
  }

  const userPrompt = buildUserPrompt(content, profile);

  // Use Gemini API directly
  const response = await fetch(
    `https://generativelanguage.googleapis.com/v1beta/models/gemini-2.0-flash-exp:generateContent?key=${apiKey}`,
    {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        contents: [
          {
            parts: [
              { text: PERSONALIZATION_SYSTEM_PROMPT + '\n\n' + userPrompt }
            ]
          }
        ],
        generationConfig: {
          temperature: 0.3, // Lower temperature for more consistent output
          maxOutputTokens: 32000, // Allow for long chapters
        },
        safetySettings: [
          { category: 'HARM_CATEGORY_HARASSMENT', threshold: 'BLOCK_NONE' },
          { category: 'HARM_CATEGORY_HATE_SPEECH', threshold: 'BLOCK_NONE' },
          { category: 'HARM_CATEGORY_SEXUALLY_EXPLICIT', threshold: 'BLOCK_NONE' },
          { category: 'HARM_CATEGORY_DANGEROUS_CONTENT', threshold: 'BLOCK_NONE' },
        ],
      }),
    }
  );

  if (!response.ok) {
    const errorText = await response.text();
    console.error('[Personalize] Gemini API error:', response.status, errorText);
    throw new Error(`Gemini API error: ${response.status}`);
  }

  const data = await response.json();

  // Extract text from Gemini response
  const personalizedContent = data.candidates?.[0]?.content?.parts?.[0]?.text || '';

  if (!personalizedContent) {
    throw new Error('Empty response from Gemini API');
  }

  // Build adjustments summary
  const adjustmentsMade: string[] = [];
  adjustmentsMade.push(`Adapted explanations for ${profile.software_background.experience_level} level`);

  if (profile.software_background.programming_languages.includes('Python')) {
    adjustmentsMade.push('Added Python-specific context');
  }
  if (profile.software_background.frameworks_platforms.includes('ROS/ROS 2')) {
    adjustmentsMade.push('Included ROS 2 familiarity assumptions');
  }
  if (profile.hardware_background.system_capability === 'low') {
    adjustmentsMade.push('Added resource-aware warnings');
  }
  if (profile.hardware_background.operating_system !== 'linux') {
    adjustmentsMade.push(`Added ${profile.hardware_background.operating_system}-specific notes`);
  }

  return { personalizedContent, adjustmentsMade };
}

// =============================================================================
// API Endpoint (TASK-007)
// =============================================================================

// Apply auth middleware to personalize routes
personalize.use('/personalize', requireAuth);

// POST /api/personalize - Personalize chapter content based on user profile
personalize.post('/personalize', async (c) => {
  try {
    const authContext = getAuthContext(c);
    if (!authContext) {
      return c.json({
        success: false,
        error: 'AUTH_REQUIRED',
        message: 'Authentication required to personalize content',
      }, 401);
    }

    // Check rate limit (TASK-002)
    const rateLimit = checkRateLimit(authContext.user.id);
    if (!rateLimit.allowed) {
      return c.json({
        success: false,
        error: 'RATE_LIMITED',
        message: 'Too many personalization requests. Please try again later.',
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

    // Fetch user profile from database (TASK-003)
    const [userProfile] = await db
      .select()
      .from(userProfiles)
      .where(eq(userProfiles.authUserId, authContext.user.id))
      .limit(1);

    if (!userProfile) {
      return c.json({
        success: false,
        error: 'PROFILE_NOT_FOUND',
        message: 'Please complete your profile to personalize content',
      }, 404);
    }

    // Transform database profile to ProfileResponse format
    const profileResponse: ProfileResponse = {
      auth_user_id: userProfile.authUserId,
      email: authContext.user.email,
      software_background: {
        programming_languages: userProfile.programmingLanguages,
        frameworks_platforms: userProfile.frameworksPlatforms,
        experience_level: userProfile.experienceLevel as ProfileResponse['software_background']['experience_level'],
      },
      hardware_background: {
        device_type: userProfile.deviceType as ProfileResponse['hardware_background']['device_type'],
        operating_system: userProfile.operatingSystem as ProfileResponse['hardware_background']['operating_system'],
        system_capability: userProfile.systemCapability as ProfileResponse['hardware_background']['system_capability'],
      },
      created_at: userProfile.createdAt.toISOString(),
      updated_at: userProfile.updatedAt.toISOString(),
    };

    // Validate profile completeness (TASK-004)
    const profileValidation = validateProfileCompleteness(profileResponse);
    if (!profileValidation.valid) {
      return c.json({
        success: false,
        error: 'PROFILE_INCOMPLETE',
        message: `Your profile is missing required fields: ${profileValidation.missingFields.join(', ')}`,
        missing_fields: profileValidation.missingFields,
      }, 400);
    }

    // Fetch chapter content (TASK-005)
    // For deployment compatibility, allow frontend to provide content directly
    let chapterContent = body.chapter_content;
    if (!chapterContent) {
      // Fallback to filesystem fetch if content not provided
      chapterContent = await fetchChapterContent(body.chapter_id);
      if (!chapterContent) {
        return c.json({
          success: false,
          error: 'CHAPTER_NOT_FOUND',
          message: 'Chapter not found and no content provided',
        }, 404);
      }
    }

    // Log personalization request
    console.log(`[Personalize] Request from user ${authContext.user.id} for chapter: ${body.chapter_id}, experience: ${profileResponse.software_background.experience_level}`);

    // Personalize with Gemini API (TASK-006)
    const { personalizedContent, adjustmentsMade } = await personalizeWithGemini(
      chapterContent,
      profileResponse
    );

    // Return success response
    return c.json({
      success: true,
      personalized_content: personalizedContent,
      chapter_id: body.chapter_id,
      personalization_summary: {
        experience_level: profileResponse.software_background.experience_level,
        programming_context: profileResponse.software_background.programming_languages,
        hardware_context: {
          system_capability: profileResponse.hardware_background.system_capability,
          operating_system: profileResponse.hardware_background.operating_system,
        },
        adjustments_made: adjustmentsMade,
      },
      timestamp: new Date().toISOString(),
    });

  } catch (error) {
    console.error('[Personalize] Error:', error);
    return c.json({
      success: false,
      error: 'PERSONALIZATION_FAILED',
      message: 'Personalization service encountered an error. Please try again.',
      retry_after: 5,
    }, 500);
  }
});

export default personalize;
