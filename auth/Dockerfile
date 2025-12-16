FROM node:20-slim AS builder

WORKDIR /app

# Copy package files
COPY package*.json ./

# Install ALL dependencies (including dev for TypeScript build)
RUN npm ci

# Copy source code
COPY . .

# Build TypeScript
RUN npm run build

# Production stage
FROM node:20-slim

WORKDIR /app

COPY package*.json ./

# Install only production dependencies
RUN npm ci --omit=dev

# Copy built files from builder
COPY --from=builder /app/dist ./dist

# Expose port (HF Spaces uses 7860 by default)
EXPOSE 7860

ENV PORT=7860

CMD ["node", "dist/index.js"]
