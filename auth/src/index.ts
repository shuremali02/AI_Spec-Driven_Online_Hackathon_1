import { Hono } from 'hono';
import { cors } from 'hono/cors';
import { auth } from './better-auth.js';
import healthRoutes from './routes/health.js';
import profileRoutes from './routes/profile.js';
import translateRoutes from './routes/translate.js';
import 'dotenv/config';

const app = new Hono();

// CORS configuration - allow multiple origins
const allowedOrigins = [
  'http://localhost:3000',
  'http://127.0.0.1:3000',
  process.env.FRONTEND_URL,
  'https://shuremali02.github.io',
].filter(Boolean) as string[];

app.use('*', cors({
  origin: (origin) => {
    // Allow requests with no origin (mobile apps, curl, etc.)
    if (!origin) return allowedOrigins[0];
    // Check if origin is in allowed list
    if (allowedOrigins.includes(origin)) return origin;
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
