import { betterAuth } from 'better-auth';
import { drizzleAdapter } from 'better-auth/adapters/drizzle';
import { db } from './db/client.js';
import * as schema from './db/schema.js';
import 'dotenv/config';

export const auth = betterAuth({
  database: drizzleAdapter(db, {
    provider: 'pg',
    schema: schema,
  }),
  emailAndPassword: {
    enabled: true,
    minPasswordLength: 8,
  },
  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days in seconds
    updateAge: 60 * 60 * 24, // 1 day - sliding expiration
    cookieCache: {
      enabled: true,
      maxAge: 60 * 5, // 5 minutes
    },
  },
  advanced: {
    crossSubDomainCookies: {
      enabled: true,
    },
    defaultCookieAttributes: {
      sameSite: 'none',
      secure: true,
    },
  },
  trustedOrigins: [
    'http://localhost:3000',
    'http://127.0.0.1:3000',
    'https://shuremali02.github.io',
    process.env.FRONTEND_URL || '',
  ].filter(Boolean),
  baseURL: process.env.BETTER_AUTH_URL || 'https://shurem-better-auth.hf.space',
  secret: process.env.BETTER_AUTH_SECRET,
});

export type Session = typeof auth.$Infer.Session;
