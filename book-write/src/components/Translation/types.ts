/**
 * Type definitions for Translation feature
 */

// =============================================================================
// Component Props
// =============================================================================

/**
 * Props for TranslateButton component
 */
export interface TranslateButtonProps {
  /** Unique chapter identifier for API call */
  chapterId: string;

  /** Original markdown content to translate */
  chapterContent: string;

  /** Optional: Callback when translation completes */
  onTranslationComplete?: (translatedContent: string) => void;

  /** Optional: Callback when toggle changes */
  onToggle?: (showTranslated: boolean) => void;

  /** Optional: Custom CSS class */
  className?: string;
}

/**
 * Props for UrduContent component
 */
export interface UrduContentProps {
  /** Translated Urdu markdown content */
  content: string;

  /** Optional: Custom CSS class */
  className?: string;
}

/**
 * Props for TranslationWrapper component
 */
export interface TranslationWrapperProps {
  /** Chapter ID from frontmatter or slug */
  chapterId: string;

  /** Raw markdown source for translation */
  rawMarkdown: string;

  /** React children (original rendered content) */
  children: React.ReactNode;
}

// =============================================================================
// State Types
// =============================================================================

/**
 * Translation operation status
 */
export type TranslationStatus = 'idle' | 'loading' | 'success' | 'error';

/**
 * State managed by TranslateButton component
 */
export interface TranslationState {
  /** Current translation operation status */
  status: TranslationStatus;

  /** Translated Urdu markdown content (cached after successful translation) */
  translatedContent: string | null;

  /** Whether to display translated content (true) or original (false) */
  showTranslated: boolean;

  /** Error message if translation failed */
  errorMessage: string | null;
}

// =============================================================================
// API Types
// =============================================================================

/**
 * POST /api/translate request body
 */
export interface TranslateRequest {
  /** Unique identifier for the chapter */
  chapter_id: string;

  /** Full markdown content of the chapter to translate */
  content: string;
}

/**
 * 200 OK response from /api/translate
 */
export interface TranslateSuccessResponse {
  success: true;

  /** Translated Urdu markdown content */
  translated_content: string;

  /** Echo back the chapter ID */
  chapter_id: string;

  /** Word count of original content */
  word_count: number;

  /** Timestamp of translation */
  timestamp: string;
}

/**
 * Error response from /api/translate
 */
export interface TranslateErrorResponse {
  success: false;

  /** Machine-readable error code */
  error: 'AUTH_REQUIRED' | 'INVALID_REQUEST' | 'CONTENT_TOO_LONG' | 'TRANSLATION_FAILED' | 'RATE_LIMITED';

  /** Human-readable error message */
  message: string;

  /** Field that caused the error (for validation errors) */
  field?: string;

  /** Seconds to wait before retry (for rate limiting) */
  retry_after?: number;
}

/**
 * Union type for all possible API responses
 */
export type TranslateResponse = TranslateSuccessResponse | TranslateErrorResponse;

// =============================================================================
// Constants
// =============================================================================

/**
 * Error types for translation feature
 */
export enum TranslationErrorType {
  AUTH_REQUIRED = 'AUTH_REQUIRED',
  INVALID_REQUEST = 'INVALID_REQUEST',
  INVALID_CHAPTER_ID = 'INVALID_CHAPTER_ID',
  CONTENT_TOO_LONG = 'CONTENT_TOO_LONG',
  TRANSLATION_FAILED = 'TRANSLATION_FAILED',
  RATE_LIMITED = 'RATE_LIMITED',
  NETWORK_ERROR = 'NETWORK_ERROR',
  TIMEOUT = 'TIMEOUT',
}

/**
 * User-facing error messages
 */
export const ERROR_MESSAGES: Record<TranslationErrorType, string> = {
  [TranslationErrorType.AUTH_REQUIRED]: 'Please sign in to translate content.',
  [TranslationErrorType.INVALID_REQUEST]: 'Translation request was invalid. Please try again.',
  [TranslationErrorType.INVALID_CHAPTER_ID]: 'Chapter not found.',
  [TranslationErrorType.CONTENT_TOO_LONG]: 'Chapter is too long to translate at once.',
  [TranslationErrorType.TRANSLATION_FAILED]: 'Translation failed. Please try again.',
  [TranslationErrorType.RATE_LIMITED]: 'Too many translation requests. Please wait and try again.',
  [TranslationErrorType.NETWORK_ERROR]: 'Connection error. Check your internet connection.',
  [TranslationErrorType.TIMEOUT]: 'Translation timed out. Please try again.',
};

/**
 * Button label text
 */
export const BUTTON_LABELS = {
  TRANSLATE: 'Translate to Urdu',
  TRANSLATE_URDU: 'اردو میں ترجمہ کریں',
  LOADING: 'Translating...',
  SHOW_ORIGINAL: 'Show Original',
  SHOW_ORIGINAL_URDU: 'اصل مواد دکھائیں',
  RETRY: 'Retry Translation',
} as const;

/**
 * API configuration
 */
export const TRANSLATION_CONFIG = {
  /** API endpoint for translation */
  ENDPOINT: '/api/translate',

  /** Request timeout in milliseconds (2 minutes for large chapters) */
  TIMEOUT_MS: 120000,

  /** Max content length (characters) */
  MAX_CONTENT_LENGTH: 100000,
} as const;
