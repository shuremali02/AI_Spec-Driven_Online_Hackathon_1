import { Hono } from 'hono';
import { cors } from 'hono/cors';
import { auth } from './better-auth.js';
import healthRoutes from './routes/health.js';
import profileRoutes from './routes/profile.js';
import 'dotenv/config';

const app = new Hono();

// CORS configuration
app.use('*', cors({
  origin: process.env.FRONTEND_URL || 'http://localhost:3000',
  credentials: true,
  allowMethods: ['GET', 'POST', 'PUT', 'DELETE', 'OPTIONS'],
  allowHeaders: ['Content-Type', 'Authorization'],
}));

// Better-Auth routes - handles /api/auth/*
app.on(['POST', 'GET'], '/api/auth/*', (c) => auth.handler(c.req.raw));

// Application routes
app.route('/api', healthRoutes);
app.route('/api', profileRoutes);

// Root endpoint
app.get('/', (c) => {
  return c.json({
    name: 'Auth Service',
    version: '1.0.0',
    endpoints: {
      health: '/api/health',
      auth: '/api/auth/*',
      profile: '/api/profile',
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
