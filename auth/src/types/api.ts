// Enums matching OpenAPI spec
export type ExperienceLevel = 'beginner' | 'intermediate' | 'advanced' | 'expert';
export type DeviceType = 'desktop' | 'laptop' | 'tablet' | 'mobile' | 'embedded';
export type OperatingSystem = 'windows' | 'macos' | 'linux' | 'other';
export type SystemCapability = 'low' | 'medium' | 'high';

// Predefined options for multi-select fields
export const PROGRAMMING_LANGUAGES = [
  'Python',
  'JavaScript/TypeScript',
  'C/C++',
  'Java',
  'Go',
  'Rust',
  'Other',
] as const;

export const FRAMEWORKS_PLATFORMS = [
  'ROS/ROS 2',
  'TensorFlow',
  'PyTorch',
  'OpenCV',
  'Arduino/Embedded',
  'Web Development',
  'Mobile Development',
  'Other',
] as const;

export const EXPERIENCE_LEVELS: ExperienceLevel[] = ['beginner', 'intermediate', 'advanced', 'expert'];
export const DEVICE_TYPES: DeviceType[] = ['desktop', 'laptop', 'tablet', 'mobile', 'embedded'];
export const OPERATING_SYSTEMS: OperatingSystem[] = ['windows', 'macos', 'linux', 'other'];
export const SYSTEM_CAPABILITIES: SystemCapability[] = ['low', 'medium', 'high'];

// Software Background
export interface SoftwareBackground {
  programming_languages: string[];
  frameworks_platforms: string[];
  experience_level: ExperienceLevel;
}

// Hardware Background
export interface HardwareBackground {
  device_type: DeviceType;
  operating_system: OperatingSystem;
  system_capability: SystemCapability;
}

// Create Profile Request
export interface CreateProfileRequest {
  software_background: SoftwareBackground;
  hardware_background: HardwareBackground;
}

// Update Profile Request
export interface UpdateProfileRequest {
  software_background?: Partial<SoftwareBackground>;
  hardware_background?: Partial<HardwareBackground>;
}

// Profile Response
export interface ProfileResponse {
  auth_user_id: string;
  email: string;
  software_background: SoftwareBackground;
  hardware_background: HardwareBackground;
  created_at: string;
  updated_at: string;
}

// Create Profile Response
export interface CreateProfileResponse {
  profile_id: string;
  created_at: string;
}

// Update Profile Response
export interface UpdateProfileResponse {
  message: string;
  updated_at: string;
}

// Error Response
export interface ErrorResponse {
  error: string;
  field?: string;
  message: string;
}
