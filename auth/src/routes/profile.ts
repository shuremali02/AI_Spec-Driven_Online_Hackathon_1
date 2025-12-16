import { Hono } from 'hono';
import { eq } from 'drizzle-orm';
import { db } from '../db/client.js';
import { userProfiles } from '../db/schema.js';
import { requireAuth, getAuthContext } from '../middleware/auth.js';
import { validateCreateProfile, validateUpdateProfile } from '../utils/validation.js';
import {
  validationError,
  notFoundError,
  conflictError,
  serverError,
} from '../utils/errors.js';
import type {
  CreateProfileRequest,
  UpdateProfileRequest,
  ProfileResponse,
  CreateProfileResponse,
  UpdateProfileResponse,
} from '../types/api.js';

const profile = new Hono();

// All profile routes require authentication
profile.use('/profile', requireAuth);
profile.use('/profile/*', requireAuth);

// POST /api/profile - Create profile after signup
profile.post('/profile', async (c) => {
  try {
    const authContext = getAuthContext(c);
    if (!authContext) {
      return serverError(c, 'Auth context not found');
    }

    const body = await c.req.json();
    const validation = validateCreateProfile(body);

    if (!validation.valid) {
      const firstError = validation.errors[0];
      return validationError(c, firstError.field, firstError.message);
    }

    const data = body as CreateProfileRequest;

    // Check if profile already exists
    const existing = await db
      .select({ id: userProfiles.id })
      .from(userProfiles)
      .where(eq(userProfiles.authUserId, authContext.user.id))
      .limit(1);

    if (existing.length > 0) {
      return conflictError(c, 'Profile already exists for this user');
    }

    // Create the profile
    const [newProfile] = await db
      .insert(userProfiles)
      .values({
        authUserId: authContext.user.id,
        programmingLanguages: data.software_background.programming_languages,
        frameworksPlatforms: data.software_background.frameworks_platforms,
        experienceLevel: data.software_background.experience_level,
        deviceType: data.hardware_background.device_type,
        operatingSystem: data.hardware_background.operating_system,
        systemCapability: data.hardware_background.system_capability,
      })
      .returning({ id: userProfiles.id, createdAt: userProfiles.createdAt });

    const response: CreateProfileResponse = {
      profile_id: newProfile.id,
      created_at: newProfile.createdAt.toISOString(),
    };

    return c.json(response, 201);
  } catch (error) {
    console.error('Create profile error:', error);
    return serverError(c, 'Failed to create profile');
  }
});

// GET /api/profile - Get current user's profile
profile.get('/profile', async (c) => {
  try {
    const authContext = getAuthContext(c);
    if (!authContext) {
      return serverError(c, 'Auth context not found');
    }

    const [userProfile] = await db
      .select()
      .from(userProfiles)
      .where(eq(userProfiles.authUserId, authContext.user.id))
      .limit(1);

    if (!userProfile) {
      return notFoundError(c, 'Profile not found for this user');
    }

    const response: ProfileResponse = {
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

    return c.json(response);
  } catch (error) {
    console.error('Get profile error:', error);
    return serverError(c, 'Failed to fetch profile');
  }
});

// PUT /api/profile - Update current user's profile
profile.put('/profile', async (c) => {
  try {
    const authContext = getAuthContext(c);
    if (!authContext) {
      return serverError(c, 'Auth context not found');
    }

    const body = await c.req.json();
    const validation = validateUpdateProfile(body);

    if (!validation.valid) {
      const firstError = validation.errors[0];
      return validationError(c, firstError.field, firstError.message);
    }

    const data = body as UpdateProfileRequest;

    // Check if profile exists
    const [existingProfile] = await db
      .select()
      .from(userProfiles)
      .where(eq(userProfiles.authUserId, authContext.user.id))
      .limit(1);

    if (!existingProfile) {
      return notFoundError(c, 'Profile not found for this user');
    }

    // Build update object
    const updateData: Partial<{
      programmingLanguages: string[];
      frameworksPlatforms: string[];
      experienceLevel: string;
      deviceType: string;
      operatingSystem: string;
      systemCapability: string;
      updatedAt: Date;
    }> = {
      updatedAt: new Date(),
    };

    if (data.software_background) {
      if (data.software_background.programming_languages) {
        updateData.programmingLanguages = data.software_background.programming_languages;
      }
      if (data.software_background.frameworks_platforms) {
        updateData.frameworksPlatforms = data.software_background.frameworks_platforms;
      }
      if (data.software_background.experience_level) {
        updateData.experienceLevel = data.software_background.experience_level;
      }
    }

    if (data.hardware_background) {
      if (data.hardware_background.device_type) {
        updateData.deviceType = data.hardware_background.device_type;
      }
      if (data.hardware_background.operating_system) {
        updateData.operatingSystem = data.hardware_background.operating_system;
      }
      if (data.hardware_background.system_capability) {
        updateData.systemCapability = data.hardware_background.system_capability;
      }
    }

    const [updatedProfile] = await db
      .update(userProfiles)
      .set(updateData)
      .where(eq(userProfiles.authUserId, authContext.user.id))
      .returning({ updatedAt: userProfiles.updatedAt });

    const response: UpdateProfileResponse = {
      message: 'Profile updated successfully',
      updated_at: updatedProfile.updatedAt.toISOString(),
    };

    return c.json(response);
  } catch (error) {
    console.error('Update profile error:', error);
    return serverError(c, 'Failed to update profile');
  }
});

export default profile;
