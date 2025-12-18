import { Hono } from 'hono';
import { cors } from 'hono/cors';
import { auth } from './better-auth.js';
import healthRoutes from './routes/health.js';
import profileRoutes from './routes/profile.js';
import translateRoutes from './routes/translate.js';
import personalizeRoutes from './routes/personalize.js';
import 'dotenv/config';

const app = new Hono();

// CORS configuration - allow multiple origins
const allowedOrigins = [
  'http://localhost:3000',
  'http://127.0.0.1:3000',
  process.env.FRONTEND_URL,
  'https://shuremali02.github.io',
  'https://ai-spec-driven-online-hackathon-1.vercel.app',
  'https://physical-ai-book.vercel.app',
].filter(Boolean) as string[];

// Check if origin matches Vercel preview/production pattern
const isVercelOrigin = (origin: string): boolean => {
  return /^https:\/\/ai-spec-driven-online-hackathon-1(-[a-z0-9]+)?\.vercel\.app$/.test(origin) ||
         /^https:\/\/physical-ai-book(-[a-z0-9]+)?\.vercel\.app$/.test(origin);
};

app.use('*', cors({
  origin: (origin) => {
    // Allow requests with no origin (mobile apps, curl, etc.)
    if (!origin) return allowedOrigins[0];
    // Check if origin is in allowed list
    if (allowedOrigins.includes(origin)) return origin;
    // Check Vercel preview URLs
    if (isVercelOrigin(origin)) return origin;
    // Fallback
    return allowedOrigins[0];
  },
  credentials: true,
  allowMethods: ['GET', 'POST', 'PUT', 'DELETE', 'OPTIONS'],
  allowHeaders: ['Content-Type', 'Authorization', 'Cookie'],
  exposeHeaders: ['Set-Cookie'],
}));

// Better-Auth routes - handles /api/auth/*
app.on(['POST', 'GET'], '/api/auth/*', (c) => auth.handler(c.req.raw));

// Application routes
app.route('/api', healthRoutes);
app.route('/api', profileRoutes);
app.route('/api', translateRoutes);
app.route('/api', personalizeRoutes);

// Root endpoint
app.get('/', (c) => {
  return c.json({
    name: 'Auth Service',
    version: '1.0.0',
    endpoints: {
      health: '/api/health',
      auth: '/api/auth/*',
      profile: '/api/profile',
      translate: '/api/translate',
      personalize: '/api/personalize',
    },
  });
});

const port = parseInt(process.env.PORT || '3001', 10);

// Use Node.js adapter for Hono
import { serve } from '@hono/node-server';

serve({
  fetch: app.fetch,
  port,
}, (info) => {
  console.log(`Auth service running on http://localhost:${info.port}`);
});
