import type { Context } from 'hono';
import type { ErrorResponse } from '../types/api.js';

export function errorResponse(
  c: Context,
  status: 400 | 401 | 403 | 404 | 409 | 500,
  error: string,
  message: string,
  field?: string
): Response {
  const body: ErrorResponse = {
    error,
    message,
    ...(field && { field }),
  };
  return c.json(body, status);
}

export function validationError(c: Context, field: string, message: string): Response {
  return errorResponse(c, 400, 'validation_error', message, field);
}

export function unauthorizedError(c: Context, message: string = 'Valid session required'): Response {
  return errorResponse(c, 401, 'unauthorized', message);
}

export function notFoundError(c: Context, message: string): Response {
  return errorResponse(c, 404, 'not_found', message);
}

export function conflictError(c: Context, message: string): Response {
  return errorResponse(c, 409, 'conflict', message);
}

export function serverError(c: Context, message: string = 'Internal server error'): Response {
  return errorResponse(c, 500, 'server_error', message);
}
