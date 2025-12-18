/**
 * Type definitions for Personalization feature
 */

// =============================================================================
// Component Props
// =============================================================================

/**
 * Props for PersonalizeButton component
 */
export interface PersonalizeButtonProps {
  /** Unique chapter identifier for API call */
  chapterId: string;

  /** Original markdown content to personalize */
  chapterContent: string;

  /** Optional: Callback when personalization completes */
  onPersonalizationComplete?: (content: string) => void;

  /** Optional: Callback when toggle changes */
  onToggle?: (showPersonalized: boolean) => void;

  /** Optional: Custom CSS class */
  className?: string;
}

/**
 * Props for PersonalizedContent component
 */
export interface PersonalizedContentProps {
  /** Personalized markdown content */
  content: string;

  /** Optional personalization summary */
  personalizationSummary?: PersonalizationSummary;

  /** Optional: Custom CSS class */
  className?: string;
}

// =============================================================================
// State Types
// =============================================================================

/**
 * Personalization operation status
 */
export type PersonalizationStatus = 'idle' | 'loading' | 'success' | 'error';

/**
 * State managed by PersonalizeButton component
 */
export interface PersonalizationState {
  /** Current personalization operation status */
  status: PersonalizationStatus;

  /** Personalized markdown content (cached after successful personalization) */
  personalizedContent: string | null;

  /** Whether to display personalized content (true) or original (false) */
  showPersonalized: boolean;

  /** Error message if personalization failed */
  errorMessage: string | null;

  /** Summary of personalization adjustments */
  personalizationSummary: PersonalizationSummary | null;
}

// =============================================================================
// API Types
// =============================================================================

/**
 * POST /api/personalize request body
 */
export interface PersonalizeRequest {
  /** Unique identifier for the chapter */
  chapter_id: string;

  /** Chapter content to personalize (optional, fallback to server-side fetch) */
  chapter_content?: string;
}

/**
 * Personalization summary returned by API
 */
export interface PersonalizationSummary {
  /** User's experience level used for personalization */
  experience_level: string;

  /** Programming languages context */
  programming_context: string[];

  /** Hardware context */
  hardware_context: {
    system_capability: string;
    operating_system: string;
  };

  /** List of adjustments made during personalization */
  adjustments_made: string[];
}

/**
 * 200 OK response from /api/personalize
 */
export interface PersonalizeSuccessResponse {
  success: true;

  /** Personalized markdown content */
  personalized_content: string;

  /** Echo back the chapter ID */
  chapter_id: string;

  /** Summary of personalization */
  personalization_summary: PersonalizationSummary;

  /** Timestamp of personalization */
  timestamp: string;
}

/**
 * Error response from /api/personalize
 */
export interface PersonalizeErrorResponse {
  success: false;

  /** Machine-readable error code */
  error:
    | 'AUTH_REQUIRED'
    | 'INVALID_REQUEST'
    | 'PROFILE_NOT_FOUND'
    | 'PROFILE_INCOMPLETE'
    | 'CHAPTER_NOT_FOUND'
    | 'PERSONALIZATION_FAILED'
    | 'RATE_LIMITED';

  /** Human-readable error message */
  message: string;

  /** Field that caused the error (for validation errors) */
  field?: string;

  /** Missing fields (for profile incomplete error) */
  missing_fields?: string[];

  /** Seconds to wait before retry (for rate limiting) */
  retry_after?: number;
}

/**
 * Union type for all possible API responses
 */
export type PersonalizeResponse = PersonalizeSuccessResponse | PersonalizeErrorResponse;

// =============================================================================
// Constants
// =============================================================================

/**
 * Error types for personalization feature
 */
export type PersonalizationErrorType =
  | 'AUTH_REQUIRED'
  | 'INVALID_REQUEST'
  | 'PROFILE_NOT_FOUND'
  | 'PROFILE_INCOMPLETE'
  | 'CHAPTER_NOT_FOUND'
  | 'PERSONALIZATION_FAILED'
  | 'RATE_LIMITED'
  | 'NETWORK_ERROR'
  | 'TIMEOUT';

/**
 * User-facing error messages
 */
export const ERROR_MESSAGES: Record<PersonalizationErrorType, string> = {
  AUTH_REQUIRED: 'Please sign in to personalize content.',
  INVALID_REQUEST: 'Personalization request was invalid. Please try again.',
  PROFILE_NOT_FOUND: 'Please complete your profile to personalize content.',
  PROFILE_INCOMPLETE: 'Your profile is missing some information needed for personalization.',
  CHAPTER_NOT_FOUND: 'Chapter not found.',
  PERSONALIZATION_FAILED: 'Personalization failed. Please try again.',
  RATE_LIMITED: 'Too many personalization requests. Please wait and try again.',
  NETWORK_ERROR: 'Connection error. Check your internet connection.',
  TIMEOUT: 'Personalization timed out. Please try again.',
};

/**
 * Button label text
 */
export const BUTTON_LABELS = {
  PERSONALIZE: 'Personalize Content',
  LOADING: 'Personalizing...',
  SHOW_ORIGINAL: 'Show Original',
  PERSONALIZE_AGAIN: 'Personalize Again',
  RETRY: 'Retry Personalization',
} as const;

/**
 * API configuration
 */
export const PERSONALIZATION_CONFIG = {
  /** API endpoint for personalization */
  ENDPOINT: '/api/personalize',

  /** Request timeout in milliseconds (90 seconds for AI processing) */
  TIMEOUT_MS: 90000,
} as const;
