import React, { useState } from 'react';
import { signUp } from './authClient';
import { PROFILE_API_URL } from './authClient';
import { useAuth } from './AuthProvider';
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
  type CreateProfileRequest,
} from './types';
import styles from './AuthForms.module.css';

interface SignupFormProps {
  onSuccess?: () => void;
  onSwitchToSignin?: () => void;
}

type SignupStep = 'credentials' | 'software' | 'hardware';

export function SignupForm({ onSuccess, onSwitchToSignin }: SignupFormProps) {
  const { refreshProfile, setProfile } = useAuth();

  // Step tracking
  const [step, setStep] = useState<SignupStep>('credentials');

  // Credentials
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [name, setName] = useState('');

  // Software background
  const [programmingLanguages, setProgrammingLanguages] = useState<string[]>([]);
  const [frameworksPlatforms, setFrameworksPlatforms] = useState<string[]>([]);
  const [experienceLevel, setExperienceLevel] = useState<ExperienceLevel | ''>('');

  // Hardware background
  const [deviceType, setDeviceType] = useState<DeviceType | ''>('');
  const [operatingSystem, setOperatingSystem] = useState<OperatingSystem | ''>('');
  const [systemCapability, setSystemCapability] = useState<SystemCapability | ''>('');

  // UI state
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [createdUserId, setCreatedUserId] = useState<string | null>(null);

  const validateCredentials = (): boolean => {
    if (!name || name.trim().length < 2) {
      setError('Name is required (minimum 2 characters)');
      return false;
    }
    if (!email || !password) {
      setError('Email and password are required');
      return false;
    }
    if (password.length < 8) {
      setError('Password must be at least 8 characters');
      return false;
    }
    if (password !== confirmPassword) {
      setError('Passwords do not match');
      return false;
    }
    return true;
  };

  const validateSoftware = (): boolean => {
    if (programmingLanguages.length === 0) {
      setError('Please select at least one programming language');
      return false;
    }
    if (frameworksPlatforms.length === 0) {
      setError('Please select at least one framework/platform');
      return false;
    }
    if (!experienceLevel) {
      setError('Please select your experience level');
      return false;
    }
    return true;
  };

  const validateHardware = (): boolean => {
    if (!deviceType) {
      setError('Please select your device type');
      return false;
    }
    if (!operatingSystem) {
      setError('Please select your operating system');
      return false;
    }
    if (!systemCapability) {
      setError('Please select your system capability');
      return false;
    }
    return true;
  };

  const handleCredentialsSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);

    if (!validateCredentials()) return;

    // Just validate and move to next step - no API call yet
    setStep('software');
  };

  const handleSoftwareSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);

    if (!validateSoftware()) return;

    setStep('hardware');
  };

  const handleHardwareSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);

    if (!validateHardware()) return;

    setIsLoading(true);
    try {
      // Step 1: Create user account with Better-Auth
      const signupResult = await signUp.email({
        email,
        password,
        name: name.trim(),
      });

      console.log('Signup result:', signupResult);

      if (signupResult.error) {
        console.error('Signup error:', signupResult.error);
        setError(signupResult.error.message || 'Account creation failed');
        return;
      }

      // Step 2: Create profile with all collected data
      const profileData: CreateProfileRequest = {
        software_background: {
          programming_languages: programmingLanguages,
          frameworks_platforms: frameworksPlatforms,
          experience_level: experienceLevel as ExperienceLevel,
        },
        hardware_background: {
          device_type: deviceType as DeviceType,
          operating_system: operatingSystem as OperatingSystem,
          system_capability: systemCapability as SystemCapability,
        },
      };

      const response = await fetch(PROFILE_API_URL, {
        method: 'POST',
        credentials: 'include',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(profileData),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        console.error('Profile creation failed:', errorData);
        throw new Error(errorData.message || 'Failed to create profile');
      }

      // Step 3: Refresh profile in context and redirect
      await refreshProfile();

      // Success - all data collected and saved!
      onSuccess?.();
    } catch (err: unknown) {
      const errorMessage = err instanceof Error ? err.message : 'Signup failed';
      setError(errorMessage);
    } finally {
      setIsLoading(false);
    }
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

  const renderProgressIndicator = () => (
    <div className={styles.progressIndicator}>
      <div className={`${styles.progressStep} ${step === 'credentials' ? styles.active : ''} ${['software', 'hardware'].includes(step) ? styles.completed : ''}`}>
        1. Account
      </div>
      <div className={`${styles.progressStep} ${step === 'software' ? styles.active : ''} ${step === 'hardware' ? styles.completed : ''}`}>
        2. Software
      </div>
      <div className={`${styles.progressStep} ${step === 'hardware' ? styles.active : ''}`}>
        3. Hardware
      </div>
    </div>
  );

  const renderCredentialsForm = () => (
    <form onSubmit={handleCredentialsSubmit} className={styles.form}>
      <h3>Create Your Account</h3>
      <p className={styles.subtitle}>Step 1 of 3: Enter your credentials</p>

      <div className={styles.formGroup}>
        <label htmlFor="name">Full Name *</label>
        <input
          id="name"
          type="text"
          value={name}
          onChange={(e) => setName(e.target.value)}
          placeholder="Your full name"
          required
          minLength={2}
          disabled={isLoading}
        />
      </div>

      <div className={styles.formGroup}>
        <label htmlFor="email">Email *</label>
        <input
          id="email"
          type="email"
          value={email}
          onChange={(e) => setEmail(e.target.value)}
          placeholder="your@email.com"
          required
          disabled={isLoading}
        />
      </div>

      <div className={styles.formGroup}>
        <label htmlFor="password">Password *</label>
        <input
          id="password"
          type="password"
          value={password}
          onChange={(e) => setPassword(e.target.value)}
          placeholder="At least 8 characters"
          required
          minLength={8}
          disabled={isLoading}
        />
      </div>

      <div className={styles.formGroup}>
        <label htmlFor="confirmPassword">Confirm Password *</label>
        <input
          id="confirmPassword"
          type="password"
          value={confirmPassword}
          onChange={(e) => setConfirmPassword(e.target.value)}
          placeholder="Repeat your password"
          required
          disabled={isLoading}
        />
      </div>

      <button type="submit" className={styles.submitButton} disabled={isLoading}>
        {isLoading ? 'Creating account...' : 'Next: Software Background'}
      </button>

      <p className={styles.switchPrompt}>
        Already have an account?{' '}
        <button type="button" onClick={onSwitchToSignin} className={styles.linkButton}>
          Sign in
        </button>
      </p>
    </form>
  );

  const renderSoftwareForm = () => (
    <form onSubmit={handleSoftwareSubmit} className={styles.form}>
      <h3>Software Background</h3>
      <p className={styles.subtitle}>Step 2 of 3: Tell us about your programming experience</p>

      <div className={styles.formGroup}>
        <label>Programming Languages * (select all that apply)</label>
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
        <label>Frameworks & Platforms * (select all that apply)</label>
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
        <label htmlFor="experienceLevel">Experience Level *</label>
        <select
          id="experienceLevel"
          value={experienceLevel}
          onChange={(e) => setExperienceLevel(e.target.value as ExperienceLevel)}
          required
        >
          <option value="">Select your experience level</option>
          {EXPERIENCE_LEVELS.map((level) => (
            <option key={level.value} value={level.value}>
              {level.label}
            </option>
          ))}
        </select>
      </div>

      <div className={styles.buttonGroup}>
        <button type="button" onClick={() => setStep('credentials')} className={styles.backButton}>
          Back
        </button>
        <button type="submit" className={styles.submitButton}>
          Next: Hardware Background
        </button>
      </div>
    </form>
  );

  const renderHardwareForm = () => (
    <form onSubmit={handleHardwareSubmit} className={styles.form}>
      <h3>Hardware Background</h3>
      <p className={styles.subtitle}>Step 3 of 3: Tell us about your development environment</p>

      <div className={styles.formGroup}>
        <label htmlFor="deviceType">Primary Device *</label>
        <select
          id="deviceType"
          value={deviceType}
          onChange={(e) => setDeviceType(e.target.value as DeviceType)}
          required
        >
          <option value="">Select your device type</option>
          {DEVICE_TYPES.map((device) => (
            <option key={device.value} value={device.value}>
              {device.label}
            </option>
          ))}
        </select>
      </div>

      <div className={styles.formGroup}>
        <label htmlFor="operatingSystem">Operating System *</label>
        <select
          id="operatingSystem"
          value={operatingSystem}
          onChange={(e) => setOperatingSystem(e.target.value as OperatingSystem)}
          required
        >
          <option value="">Select your operating system</option>
          {OPERATING_SYSTEMS.map((os) => (
            <option key={os.value} value={os.value}>
              {os.label}
            </option>
          ))}
        </select>
      </div>

      <div className={styles.formGroup}>
        <label htmlFor="systemCapability">System Capability *</label>
        <select
          id="systemCapability"
          value={systemCapability}
          onChange={(e) => setSystemCapability(e.target.value as SystemCapability)}
          required
        >
          <option value="">Select your system capability</option>
          {SYSTEM_CAPABILITIES.map((cap) => (
            <option key={cap.value} value={cap.value}>
              {cap.label}
            </option>
          ))}
        </select>
      </div>

      <div className={styles.buttonGroup}>
        <button type="button" onClick={() => setStep('software')} className={styles.backButton} disabled={isLoading}>
          Back
        </button>
        <button type="submit" className={styles.submitButton} disabled={isLoading}>
          {isLoading ? 'Creating profile...' : 'Complete Signup'}
        </button>
      </div>
    </form>
  );

  return (
    <div className={styles.authContainer}>
      {renderProgressIndicator()}

      {error && (
        <div className={styles.errorMessage}>
          {error}
        </div>
      )}

      {step === 'credentials' && renderCredentialsForm()}
      {step === 'software' && renderSoftwareForm()}
      {step === 'hardware' && renderHardwareForm()}
    </div>
  );
}

export default SignupForm;
