import type { Context, Next } from 'hono';
import { auth } from '../better-auth.js';
import { unauthorizedError } from '../utils/errors.js';

export interface AuthContext {
  user: {
    id: string;
    email: string;
    name?: string;
  };
  session: {
    id: string;
    userId: string;
    expiresAt: Date;
  };
}

// Middleware to validate session and attach user context
export async function requireAuth(c: Context, next: Next): Promise<Response | void> {
  try {
    const session = await auth.api.getSession({
      headers: c.req.raw.headers,
    });

    if (!session || !session.user) {
      return unauthorizedError(c);
    }

    // Attach user and session to context
    c.set('auth', {
      user: session.user,
      session: session.session,
    } as AuthContext);

    await next();
  } catch (error) {
    console.error('Auth middleware error:', error);
    return unauthorizedError(c);
  }
}

// Helper to get auth context from request
export function getAuthContext(c: Context): AuthContext | undefined {
  return c.get('auth') as AuthContext | undefined;
}
