---
title: Auth Service
emoji: 🔐
colorFrom: purple
colorTo: indigo
sdk: docker
pinned: false
app_port: 7860
---

# Auth Service

Authentication service for Physical AI & Humanoid Robotics Textbook using Better-Auth.

## Endpoints

- `GET /` - Service info
- `GET /api/health` - Health check
- `POST /api/auth/sign-up/email` - Sign up
- `POST /api/auth/sign-in/email` - Sign in
- `POST /api/auth/sign-out` - Sign out
- `GET /api/auth/session` - Get session
- `GET /api/profile` - Get user profile
- `PUT /api/profile` - Update user profile

## Environment Variables

Set these in HF Spaces secrets:
- `DATABASE_URL` - Neon Postgres connection string
- `BETTER_AUTH_SECRET` - Auth secret key
- `FRONTEND_URL` - Frontend URL for CORS
