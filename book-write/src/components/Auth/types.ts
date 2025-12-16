// Enums matching backend
export type ExperienceLevel = 'beginner' | 'intermediate' | 'advanced' | 'expert';
export type DeviceType = 'desktop' | 'laptop' | 'tablet' | 'mobile' | 'embedded';
export type OperatingSystem = 'windows' | 'macos' | 'linux' | 'other';
export type SystemCapability = 'low' | 'medium' | 'high';

// Options for form selections
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

export const EXPERIENCE_LEVELS: { value: ExperienceLevel; label: string }[] = [
  { value: 'beginner', label: 'Beginner (0-1 years)' },
  { value: 'intermediate', label: 'Intermediate (1-3 years)' },
  { value: 'advanced', label: 'Advanced (3-5 years)' },
  { value: 'expert', label: 'Expert (5+ years)' },
];

export const DEVICE_TYPES: { value: DeviceType; label: string }[] = [
  { value: 'desktop', label: 'Desktop' },
  { value: 'laptop', label: 'Laptop' },
  { value: 'tablet', label: 'Tablet' },
  { value: 'mobile', label: 'Mobile' },
  { value: 'embedded', label: 'Embedded (Raspberry Pi, Jetson, etc.)' },
];

export const OPERATING_SYSTEMS: { value: OperatingSystem; label: string }[] = [
  { value: 'windows', label: 'Windows' },
  { value: 'macos', label: 'macOS' },
  { value: 'linux', label: 'Linux' },
  { value: 'other', label: 'Other' },
];

export const SYSTEM_CAPABILITIES: { value: SystemCapability; label: string }[] = [
  { value: 'low', label: 'Low (basic web browsing)' },
  { value: 'medium', label: 'Medium (can run IDEs and light simulations)' },
  { value: 'high', label: 'High (can run heavy simulations and GPU workloads)' },
];

// API Types
export interface SoftwareBackground {
  programming_languages: string[];
  frameworks_platforms: string[];
  experience_level: ExperienceLevel;
}

export interface HardwareBackground {
  device_type: DeviceType;
  operating_system: OperatingSystem;
  system_capability: SystemCapability;
}

export interface CreateProfileRequest {
  software_background: SoftwareBackground;
  hardware_background: HardwareBackground;
}

export interface ProfileResponse {
  auth_user_id: string;
  email: string;
  software_background: SoftwareBackground;
  hardware_background: HardwareBackground;
  created_at: string;
  updated_at: string;
}

export interface AuthUser {
  id: string;
  email: string;
  name?: string;
}

export interface AuthState {
  user: AuthUser | null;
  profile: ProfileResponse | null;
  isLoading: boolean;
  error: string | null;
}
