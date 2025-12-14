#!/bin/bash

# Deployment script for RAG Book Chatbot
set -e  # Exit on any error

echo "Starting deployment of RAG Book Chatbot..."

# Check if we're in the correct directory
if [ ! -f "backend/requirements.txt" ]; then
    echo "Error: backend/requirements.txt not found. Please run this script from the project root."
    exit 1
fi

# Install dependencies
echo "Installing Python dependencies..."
cd backend
pip install -r requirements.txt

# Check if environment variables are set
if [ -z "$QDRANT_URL" ] || [ -z "$QDRANT_API_KEY" ] || [ -z "$GEMINI_API_KEY" ]; then
    echo "Warning: Environment variables not set. Please ensure QDRANT_URL, QDRANT_API_KEY, and GEMINI_API_KEY are set."
    echo "You can create a .env file in the backend directory with these variables."
fi

# Run database migrations (if any)
echo "Setting up database tables..."
# The application will automatically create tables on startup

# Run tests
echo "Running tests..."
python -m pytest tests/ -v

# Start the application
echo "Starting the RAG Book Chatbot API..."
cd ..
uvicorn backend.main:app --host 0.0.0.0 --port ${PORT:-8000} --reload

echo "Deployment completed successfully!"