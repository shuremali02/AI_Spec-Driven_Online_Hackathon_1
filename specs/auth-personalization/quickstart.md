# Quickstart: Authentication & User Personalization

**Feature**: auth-personalization
**Date**: 2025-12-16

---

## Prerequisites

- Node.js 18+ installed
- Access to Neon PostgreSQL database (same instance as chatbot)
- Docusaurus frontend running (`/book-write/`)

---

## 1. Setup Auth Service

### 1.1 Initialize Project

```bash
# From project root
mkdir -p auth/src/{routes,db}
cd auth

# Initialize Node.js project
npm init -y

# Install dependencies
npm install better-auth drizzle-orm @neondatabase/serverless dotenv hono
npm install -D typescript @types/node tsx drizzle-kit
```

### 1.2 Configure TypeScript

```bash
# Create tsconfig.json
cat > tsconfig.json << 'EOF'
{
  "compilerOptions": {
    "target": "ES2022",
    "module": "ESNext",
    "moduleResolution": "bundler",
    "strict": true,
    "esModuleInterop": true,
    "skipLibCheck": true,
    "outDir": "dist",
    "rootDir": "src"
  },
  "include": ["src/**/*"]
}
EOF
```

### 1.3 Create Environment File

```bash
# Create .env file (add to .gitignore!)
cat > .env << 'EOF'
DATABASE_URL=postgresql://user:password@your-neon-host/database?sslmode=require
BETTER_AUTH_SECRET=your-32-character-or-longer-secret-key-here
BETTER_AUTH_URL=http://localhost:3001
FRONTEND_URL=http://localhost:3000
NODE_ENV=development
EOF
```

---

## 2. Implement Core Files

### 2.1 Database Client (`src/db/client.ts`)

```typescript
import { neon } from '@neondatabase/serverless';
import { drizzle } from 'drizzle-orm/neon-http';
import * as schema from './schema';

const sql = neon(process.env.DATABASE_URL!);
export const db = drizzle(sql, { schema });
```

### 2.2 Database Schema (`src/db/schema.ts`)

```typescript
import { pgTable, uuid, varchar, text, timestamp } from 'drizzle-orm/pg-core';

export const userProfiles = pgTable('user_profiles', {
  id: uuid('id').primaryKey().defaultRandom(),
  authUserId: varchar('auth_user_id', { length: 255 }).unique().notNull(),
  programmingLanguages: text('programming_languages').array().notNull(),
  frameworksPlatforms: text('frameworks_platforms').array().notNull(),
  experienceLevel: varchar('experience_level', { length: 20 }).notNull(),
  deviceType: varchar('device_type', { length: 20 }).notNull(),
  operatingSystem: varchar('operating_system', { length: 20 }).notNull(),
  systemCapability: varchar('system_capability', { length: 10 }).notNull(),
  createdAt: timestamp('created_at', { withTimezone: true }).defaultNow().notNull(),
  updatedAt: timestamp('updated_at', { withTimezone: true }).defaultNow().notNull(),
});
```

### 2.3 Better-Auth Configuration (`src/better-auth.ts`)

```typescript
import { betterAuth } from 'better-auth';
import { drizzleAdapter } from 'better-auth/adapters/drizzle';
import { db } from './db/client';

export const auth = betterAuth({
  database: drizzleAdapter(db, {
    provider: 'pg',
  }),
  emailAndPassword: {
    enabled: true,
    minPasswordLength: 8,
  },
  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days
    updateAge: 60 * 60 * 24, // 1 day (sliding expiration)
    cookieCache: {
      enabled: true,
      maxAge: 60 * 5, // 5 minutes
    },
  },
  trustedOrigins: [process.env.FRONTEND_URL!],
});
```

### 2.4 Entry Point (`src/index.ts`)

```typescript
import { Hono } from 'hono';
import { cors } from 'hono/cors';
import { auth } from './better-auth';
import profileRoutes from './routes/profile';

const app = new Hono();

// CORS configuration
app.use('*', cors({
  origin: process.env.FRONTEND_URL!,
  credentials: true,
}));

// Better-Auth routes
app.on(['POST', 'GET'], '/api/auth/*', (c) => auth.handler(c.req.raw));

// Profile routes
app.route('/api', profileRoutes);

// Health check
app.get('/api/health', (c) => c.json({ status: 'ok', timestamp: new Date().toISOString() }));

export default {
  port: 3001,
  fetch: app.fetch,
};
```

---

## 3. Run Database Migration

```bash
# Generate migration
npx drizzle-kit generate:pg

# Run migration (or execute SQL directly in Neon console)
npx drizzle-kit push:pg
```

---

## 4. Start the Service

```bash
# Development mode
npx tsx src/index.ts

# Or add to package.json scripts
npm run dev
```

---

## 5. Test Endpoints

### 5.1 Health Check

```bash
curl http://localhost:3001/api/health
```

### 5.2 Signup (via Better-Auth)

```bash
curl -X POST http://localhost:3001/api/auth/sign-up/email \
  -H "Content-Type: application/json" \
  -d '{"email": "test@example.com", "password": "password123"}'
```

### 5.3 Create Profile

```bash
curl -X POST http://localhost:3001/api/profile \
  -H "Content-Type: application/json" \
  -H "Cookie: better-auth.session_token=<token>" \
  -d '{
    "software_background": {
      "programming_languages": ["Python"],
      "frameworks_platforms": ["ROS/ROS 2"],
      "experience_level": "intermediate"
    },
    "hardware_background": {
      "device_type": "laptop",
      "operating_system": "linux",
      "system_capability": "high"
    }
  }'
```

---

## 6. Frontend Integration

### 6.1 Install Better-Auth React Client

```bash
cd book-write
npm install @better-auth/react
```

### 6.2 Create Auth Provider

```typescript
// src/components/Auth/AuthProvider.tsx
import { createAuthClient } from '@better-auth/react';

export const authClient = createAuthClient({
  baseURL: 'http://localhost:3001',
});

export const { useSession, signIn, signUp, signOut } = authClient;
```

---

## Next Steps

1. Implement profile CRUD routes (`src/routes/profile.ts`)
2. Create frontend signup form with background questions
3. Create frontend signin form
4. Create profile display component
5. Implement rollback logic for signup failures

See `tasks.md` for detailed implementation tasks.

---

**END OF QUICKSTART**
