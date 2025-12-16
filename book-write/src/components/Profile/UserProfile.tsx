import React, { useState } from 'react';
import { useAuth } from '../Auth/AuthProvider';
import { PROFILE_API_URL } from '../Auth/authClient';
import {
  PROGRAMMING_LANGUAGES,
  FRAMEWORKS_PLATFORMS,
  EXPERIENCE_LEVELS,
  DEVICE_TYPES,
  OPERATING_SYSTEMS,
  SYSTEM_CAPABILITIES,
  type ExperienceLevel,
  type DeviceType,
  type OperatingSystem,
  type SystemCapability,
} from '../Auth/types';
import styles from './UserProfile.module.css';

export function UserProfile() {
  const { user, profile, isLoading, error: authError, signOut, refreshProfile } = useAuth();

  const [isEditing, setIsEditing] = useState(false);
  const [isSaving, setIsSaving] = useState(false);
  const [saveError, setSaveError] = useState<string | null>(null);
  const [saveSuccess, setSaveSuccess] = useState(false);

  // Edit form state
  const [programmingLanguages, setProgrammingLanguages] = useState<string[]>([]);
  const [frameworksPlatforms, setFrameworksPlatforms] = useState<string[]>([]);
  const [experienceLevel, setExperienceLevel] = useState<ExperienceLevel | ''>('');
  const [deviceType, setDeviceType] = useState<DeviceType | ''>('');
  const [operatingSystem, setOperatingSystem] = useState<OperatingSystem | ''>('');
  const [systemCapability, setSystemCapability] = useState<SystemCapability | ''>('');

  const startEditing = () => {
    if (!profile) return;

    setProgrammingLanguages(profile.software_background.programming_languages);
    setFrameworksPlatforms(profile.software_background.frameworks_platforms);
    setExperienceLevel(profile.software_background.experience_level);
    setDeviceType(profile.hardware_background.device_type);
    setOperatingSystem(profile.hardware_background.operating_system);
    setSystemCapability(profile.hardware_background.system_capability);
    setIsEditing(true);
    setSaveError(null);
    setSaveSuccess(false);
  };

  const cancelEditing = () => {
    setIsEditing(false);
    setSaveError(null);
    setSaveSuccess(false);
  };

  const handleMultiSelect = (
    value: string,
    selected: string[],
    setSelected: React.Dispatch<React.SetStateAction<string[]>>
  ) => {
    if (selected.includes(value)) {
      setSelected(selected.filter(v => v !== value));
    } else {
      setSelected([...selected, value]);
    }
  };

  const handleSave = async () => {
    setSaveError(null);
    setSaveSuccess(false);

    // Validate
    if (programmingLanguages.length === 0) {
      setSaveError('Please select at least one programming language');
      return;
    }
    if (frameworksPlatforms.length === 0) {
      setSaveError('Please select at least one framework/platform');
      return;
    }
    if (!experienceLevel || !deviceType || !operatingSystem || !systemCapability) {
      setSaveError('Please fill in all required fields');
      return;
    }

    setIsSaving(true);
    try {
      const response = await fetch(PROFILE_API_URL, {
        method: 'PUT',
        credentials: 'include',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          software_background: {
            programming_languages: programmingLanguages,
            frameworks_platforms: frameworksPlatforms,
            experience_level: experienceLevel,
          },
          hardware_background: {
            device_type: deviceType,
            operating_system: operatingSystem,
            system_capability: systemCapability,
          },
        }),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.message || 'Failed to update profile');
      }

      await refreshProfile();
      setIsEditing(false);
      setSaveSuccess(true);

      // Clear success message after 3 seconds
      setTimeout(() => setSaveSuccess(false), 3000);
    } catch (err: unknown) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to save profile';
      setSaveError(errorMessage);
    } finally {
      setIsSaving(false);
    }
  };

  if (isLoading) {
    return (
      <div className={styles.container}>
        <div className={styles.loading}>Loading profile...</div>
      </div>
    );
  }

  if (!user) {
    return (
      <div className={styles.container}>
        <div className={styles.notLoggedIn}>
          <h2>Not Signed In</h2>
          <p>Please sign in to view your profile.</p>
        </div>
      </div>
    );
  }

  if (authError) {
    return (
      <div className={styles.container}>
        <div className={styles.error}>
          <h2>Error</h2>
          <p>{authError}</p>
        </div>
      </div>
    );
  }

  if (!profile) {
    return (
      <div className={styles.container}>
        <div className={styles.noProfile}>
          <h2>Profile Not Found</h2>
          <p>Your profile hasn't been created yet. Please complete the signup process.</p>
        </div>
      </div>
    );
  }

  const renderViewMode = () => (
    <div className={styles.profileContent}>
      <div className={styles.header}>
        <div className={styles.userInfo}>
          <div className={styles.avatar}>
            {(user.name || user.email)[0].toUpperCase()}
          </div>
          <div>
            <h2>{user.name || 'User'}</h2>
            <p className={styles.email}>{user.email}</p>
          </div>
        </div>
        <div className={styles.headerActions}>
          <button onClick={startEditing} className={styles.editButton}>
            Edit Profile
          </button>
          <button onClick={signOut} className={styles.signOutButton}>
            Sign Out
          </button>
        </div>
      </div>

      {saveSuccess && (
        <div className={styles.successMessage}>
          Profile updated successfully!
        </div>
      )}

      <div className={styles.sections}>
        <div className={styles.section}>
          <h3>Software Background</h3>
          <div className={styles.field}>
            <label>Programming Languages</label>
            <div className={styles.tags}>
              {profile.software_background.programming_languages.map(lang => (
                <span key={lang} className={styles.tag}>{lang}</span>
              ))}
            </div>
          </div>
          <div className={styles.field}>
            <label>Frameworks & Platforms</label>
            <div className={styles.tags}>
              {profile.software_background.frameworks_platforms.map(fw => (
                <span key={fw} className={styles.tag}>{fw}</span>
              ))}
            </div>
          </div>
          <div className={styles.field}>
            <label>Experience Level</label>
            <p>{EXPERIENCE_LEVELS.find(e => e.value === profile.software_background.experience_level)?.label || profile.software_background.experience_level}</p>
          </div>
        </div>

        <div className={styles.section}>
          <h3>Hardware Background</h3>
          <div className={styles.field}>
            <label>Device Type</label>
            <p>{DEVICE_TYPES.find(d => d.value === profile.hardware_background.device_type)?.label || profile.hardware_background.device_type}</p>
          </div>
          <div className={styles.field}>
            <label>Operating System</label>
            <p>{OPERATING_SYSTEMS.find(o => o.value === profile.hardware_background.operating_system)?.label || profile.hardware_background.operating_system}</p>
          </div>
          <div className={styles.field}>
            <label>System Capability</label>
            <p>{SYSTEM_CAPABILITIES.find(s => s.value === profile.hardware_background.system_capability)?.label || profile.hardware_background.system_capability}</p>
          </div>
        </div>
      </div>

      <div className={styles.metadata}>
        <p>Profile created: {new Date(profile.created_at).toLocaleDateString()}</p>
        <p>Last updated: {new Date(profile.updated_at).toLocaleDateString()}</p>
      </div>
    </div>
  );

  const renderEditMode = () => (
    <div className={styles.profileContent}>
      <div className={styles.header}>
        <h2>Edit Profile</h2>
        <div className={styles.headerActions}>
          <button onClick={cancelEditing} className={styles.cancelButton} disabled={isSaving}>
            Cancel
          </button>
          <button onClick={handleSave} className={styles.saveButton} disabled={isSaving}>
            {isSaving ? 'Saving...' : 'Save Changes'}
          </button>
        </div>
      </div>

      {saveError && (
        <div className={styles.errorMessage}>
          {saveError}
        </div>
      )}

      <div className={styles.editSections}>
        <div className={styles.editSection}>
          <h3>Software Background</h3>

          <div className={styles.formGroup}>
            <label>Programming Languages *</label>
            <div className={styles.checkboxGroup}>
              {PROGRAMMING_LANGUAGES.map((lang) => (
                <label key={lang} className={styles.checkboxLabel}>
                  <input
                    type="checkbox"
                    checked={programmingLanguages.includes(lang)}
                    onChange={() => handleMultiSelect(lang, programmingLanguages, setProgrammingLanguages)}
                  />
                  {lang}
                </label>
              ))}
            </div>
          </div>

          <div className={styles.formGroup}>
            <label>Frameworks & Platforms *</label>
            <div className={styles.checkboxGroup}>
              {FRAMEWORKS_PLATFORMS.map((fw) => (
                <label key={fw} className={styles.checkboxLabel}>
                  <input
                    type="checkbox"
                    checked={frameworksPlatforms.includes(fw)}
                    onChange={() => handleMultiSelect(fw, frameworksPlatforms, setFrameworksPlatforms)}
                  />
                  {fw}
                </label>
              ))}
            </div>
          </div>

          <div className={styles.formGroup}>
            <label>Experience Level *</label>
            <select
              value={experienceLevel}
              onChange={(e) => setExperienceLevel(e.target.value as ExperienceLevel)}
            >
              <option value="">Select experience level</option>
              {EXPERIENCE_LEVELS.map((level) => (
                <option key={level.value} value={level.value}>
                  {level.label}
                </option>
              ))}
            </select>
          </div>
        </div>

        <div className={styles.editSection}>
          <h3>Hardware Background</h3>

          <div className={styles.formGroup}>
            <label>Device Type *</label>
            <select
              value={deviceType}
              onChange={(e) => setDeviceType(e.target.value as DeviceType)}
            >
              <option value="">Select device type</option>
              {DEVICE_TYPES.map((device) => (
                <option key={device.value} value={device.value}>
                  {device.label}
                </option>
              ))}
            </select>
          </div>

          <div className={styles.formGroup}>
            <label>Operating System *</label>
            <select
              value={operatingSystem}
              onChange={(e) => setOperatingSystem(e.target.value as OperatingSystem)}
            >
              <option value="">Select operating system</option>
              {OPERATING_SYSTEMS.map((os) => (
                <option key={os.value} value={os.value}>
                  {os.label}
                </option>
              ))}
            </select>
          </div>

          <div className={styles.formGroup}>
            <label>System Capability *</label>
            <select
              value={systemCapability}
              onChange={(e) => setSystemCapability(e.target.value as SystemCapability)}
            >
              <option value="">Select system capability</option>
              {SYSTEM_CAPABILITIES.map((cap) => (
                <option key={cap.value} value={cap.value}>
                  {cap.label}
                </option>
              ))}
            </select>
          </div>
        </div>
      </div>
    </div>
  );

  return (
    <div className={styles.container}>
      {isEditing ? renderEditMode() : renderViewMode()}
    </div>
  );
}

export default UserProfile;
