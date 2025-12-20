/**
 * Utility functions for Personalization feature
 */

import type { ProfileResponse } from '@site/src/components/Auth/types';

/**
 * Check if user profile has all required fields for personalization
 *
 * Required fields:
 * - experience_level
 * - programming_languages (non-empty array)
 * - frameworks_platforms (non-empty array)
 * - device_type
 * - operating_system
 * - system_capability
 *
 * @param profile - User profile from auth context
 * @returns true if profile has all required fields for personalization
 */
export function hasCompleteProfile(profile: ProfileResponse | null): boolean {
  if (!profile) return false;

  const { software_background, hardware_background } = profile;

  // Check software background
  if (!software_background) return false;
  if (!software_background.experience_level) return false;
  if (!software_background.programming_languages?.length) return false;
  if (!software_background.frameworks_platforms?.length) return false;

  // Check hardware background
  if (!hardware_background) return false;
  if (!hardware_background.device_type) return false;
  if (!hardware_background.operating_system) return false;
  if (!hardware_background.system_capability) return false;

  return true;
}

/**
 * Get list of missing profile fields
 *
 * @param profile - User profile from auth context
 * @returns Array of missing field names
 */
export function getMissingProfileFields(profile: ProfileResponse | null): string[] {
  if (!profile) return ['profile'];

  const missingFields: string[] = [];
  const { software_background, hardware_background } = profile;

  // Check software background
  if (!software_background) {
    missingFields.push('software_background');
  } else {
    if (!software_background.experience_level) {
      missingFields.push('experience_level');
    }
    if (!software_background.programming_languages?.length) {
      missingFields.push('programming_languages');
    }
    if (!software_background.frameworks_platforms?.length) {
      missingFields.push('frameworks_platforms');
    }
  }

  // Check hardware background
  if (!hardware_background) {
    missingFields.push('hardware_background');
  } else {
    if (!hardware_background.device_type) {
      missingFields.push('device_type');
    }
    if (!hardware_background.operating_system) {
      missingFields.push('operating_system');
    }
    if (!hardware_background.system_capability) {
      missingFields.push('system_capability');
    }
  }

  return missingFields;
}

/**
 * Format experience level for display
 *
 * @param level - Experience level from profile
 * @returns Formatted string for display
 */
export function formatExperienceLevel(level: string): string {
  const labels: Record<string, string> = {
    beginner: 'Beginner',
    intermediate: 'Intermediate',
    advanced: 'Advanced',
    expert: 'Expert',
  };
  return labels[level] || level;
}
