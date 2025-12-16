import {
  CreateProfileRequest,
  UpdateProfileRequest,
  EXPERIENCE_LEVELS,
  DEVICE_TYPES,
  OPERATING_SYSTEMS,
  SYSTEM_CAPABILITIES,
  PROGRAMMING_LANGUAGES,
  FRAMEWORKS_PLATFORMS,
  ExperienceLevel,
  DeviceType,
  OperatingSystem,
  SystemCapability,
} from '../types/api.js';

export interface ValidationError {
  field: string;
  message: string;
}

export interface ValidationResult {
  valid: boolean;
  errors: ValidationError[];
}

function isValidProgrammingLanguage(lang: string): boolean {
  return PROGRAMMING_LANGUAGES.includes(lang as typeof PROGRAMMING_LANGUAGES[number]) ||
    lang.startsWith('Other');
}

function isValidFrameworkPlatform(fw: string): boolean {
  return FRAMEWORKS_PLATFORMS.includes(fw as typeof FRAMEWORKS_PLATFORMS[number]) ||
    fw.startsWith('Other');
}

export function validateCreateProfile(data: unknown): ValidationResult {
  const errors: ValidationError[] = [];

  if (!data || typeof data !== 'object') {
    return {
      valid: false,
      errors: [{ field: 'body', message: 'Request body is required' }],
    };
  }

  const req = data as CreateProfileRequest;

  // Validate software_background
  if (!req.software_background) {
    errors.push({ field: 'software_background', message: 'Software background is required' });
  } else {
    const sw = req.software_background;

    // programming_languages
    if (!sw.programming_languages || !Array.isArray(sw.programming_languages)) {
      errors.push({ field: 'programming_languages', message: 'Programming languages is required and must be an array' });
    } else if (sw.programming_languages.length === 0) {
      errors.push({ field: 'programming_languages', message: 'At least one programming language is required' });
    } else {
      for (const lang of sw.programming_languages) {
        if (!isValidProgrammingLanguage(lang)) {
          errors.push({ field: 'programming_languages', message: `Invalid programming language: ${lang}` });
          break;
        }
      }
    }

    // frameworks_platforms
    if (!sw.frameworks_platforms || !Array.isArray(sw.frameworks_platforms)) {
      errors.push({ field: 'frameworks_platforms', message: 'Frameworks/platforms is required and must be an array' });
    } else if (sw.frameworks_platforms.length === 0) {
      errors.push({ field: 'frameworks_platforms', message: 'At least one framework/platform is required' });
    } else {
      for (const fw of sw.frameworks_platforms) {
        if (!isValidFrameworkPlatform(fw)) {
          errors.push({ field: 'frameworks_platforms', message: `Invalid framework/platform: ${fw}` });
          break;
        }
      }
    }

    // experience_level
    if (!sw.experience_level) {
      errors.push({ field: 'experience_level', message: 'Experience level is required' });
    } else if (!EXPERIENCE_LEVELS.includes(sw.experience_level as ExperienceLevel)) {
      errors.push({
        field: 'experience_level',
        message: `Invalid experience level. Must be one of: ${EXPERIENCE_LEVELS.join(', ')}`,
      });
    }
  }

  // Validate hardware_background
  if (!req.hardware_background) {
    errors.push({ field: 'hardware_background', message: 'Hardware background is required' });
  } else {
    const hw = req.hardware_background;

    // device_type
    if (!hw.device_type) {
      errors.push({ field: 'device_type', message: 'Device type is required' });
    } else if (!DEVICE_TYPES.includes(hw.device_type as DeviceType)) {
      errors.push({
        field: 'device_type',
        message: `Invalid device type. Must be one of: ${DEVICE_TYPES.join(', ')}`,
      });
    }

    // operating_system
    if (!hw.operating_system) {
      errors.push({ field: 'operating_system', message: 'Operating system is required' });
    } else if (!OPERATING_SYSTEMS.includes(hw.operating_system as OperatingSystem)) {
      errors.push({
        field: 'operating_system',
        message: `Invalid operating system. Must be one of: ${OPERATING_SYSTEMS.join(', ')}`,
      });
    }

    // system_capability
    if (!hw.system_capability) {
      errors.push({ field: 'system_capability', message: 'System capability is required' });
    } else if (!SYSTEM_CAPABILITIES.includes(hw.system_capability as SystemCapability)) {
      errors.push({
        field: 'system_capability',
        message: `Invalid system capability. Must be one of: ${SYSTEM_CAPABILITIES.join(', ')}`,
      });
    }
  }

  return {
    valid: errors.length === 0,
    errors,
  };
}

export function validateUpdateProfile(data: unknown): ValidationResult {
  const errors: ValidationError[] = [];

  if (!data || typeof data !== 'object') {
    return {
      valid: false,
      errors: [{ field: 'body', message: 'Request body is required' }],
    };
  }

  const req = data as UpdateProfileRequest;

  // At least one of software_background or hardware_background must be provided
  if (!req.software_background && !req.hardware_background) {
    errors.push({
      field: 'body',
      message: 'At least one of software_background or hardware_background must be provided',
    });
    return { valid: false, errors };
  }

  // Validate software_background if provided
  if (req.software_background) {
    const sw = req.software_background;

    if (sw.programming_languages !== undefined) {
      if (!Array.isArray(sw.programming_languages)) {
        errors.push({ field: 'programming_languages', message: 'Programming languages must be an array' });
      } else if (sw.programming_languages.length === 0) {
        errors.push({ field: 'programming_languages', message: 'At least one programming language is required' });
      } else {
        for (const lang of sw.programming_languages) {
          if (!isValidProgrammingLanguage(lang)) {
            errors.push({ field: 'programming_languages', message: `Invalid programming language: ${lang}` });
            break;
          }
        }
      }
    }

    if (sw.frameworks_platforms !== undefined) {
      if (!Array.isArray(sw.frameworks_platforms)) {
        errors.push({ field: 'frameworks_platforms', message: 'Frameworks/platforms must be an array' });
      } else if (sw.frameworks_platforms.length === 0) {
        errors.push({ field: 'frameworks_platforms', message: 'At least one framework/platform is required' });
      } else {
        for (const fw of sw.frameworks_platforms) {
          if (!isValidFrameworkPlatform(fw)) {
            errors.push({ field: 'frameworks_platforms', message: `Invalid framework/platform: ${fw}` });
            break;
          }
        }
      }
    }

    if (sw.experience_level !== undefined) {
      if (!EXPERIENCE_LEVELS.includes(sw.experience_level as ExperienceLevel)) {
        errors.push({
          field: 'experience_level',
          message: `Invalid experience level. Must be one of: ${EXPERIENCE_LEVELS.join(', ')}`,
        });
      }
    }
  }

  // Validate hardware_background if provided
  if (req.hardware_background) {
    const hw = req.hardware_background;

    if (hw.device_type !== undefined) {
      if (!DEVICE_TYPES.includes(hw.device_type as DeviceType)) {
        errors.push({
          field: 'device_type',
          message: `Invalid device type. Must be one of: ${DEVICE_TYPES.join(', ')}`,
        });
      }
    }

    if (hw.operating_system !== undefined) {
      if (!OPERATING_SYSTEMS.includes(hw.operating_system as OperatingSystem)) {
        errors.push({
          field: 'operating_system',
          message: `Invalid operating system. Must be one of: ${OPERATING_SYSTEMS.join(', ')}`,
        });
      }
    }

    if (hw.system_capability !== undefined) {
      if (!SYSTEM_CAPABILITIES.includes(hw.system_capability as SystemCapability)) {
        errors.push({
          field: 'system_capability',
          message: `Invalid system capability. Must be one of: ${SYSTEM_CAPABILITIES.join(', ')}`,
        });
      }
    }
  }

  return {
    valid: errors.length === 0,
    errors,
  };
}
