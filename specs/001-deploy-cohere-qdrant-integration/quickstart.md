# Quickstart Guide: RAG Chatbot Integration for ROS 2 Book

## Prerequisites

- Python 3.11+
- Node.js 18+
- Access to Cohere API key
- Access to Qdrant Cloud instance
- Access to Neon Postgres database
- OpenAI API key (for Agents SDK)

## Environment Setup

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. Set up the backend environment:
   ```bash
   cd backend
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   pip install -r requirements.txt
   ```

3. Set up environment variables:
   ```bash
   cp .env.example .env
   # Update .env with your API keys and connection strings
   ```

4. Set up the frontend environment:
   ```bash
   cd frontend  # or hackathon-ai-book if integrating directly
   npm install
   ```

## Configuration

1. Update `backend/.env` with your credentials:
   ```
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_API_KEY=your_qdrant_api_key
   QDRANT_HOST=your_qdrant_host
   QDRANT_PORT=6333
   DATABASE_URL=your_neon_postgres_connection_string
   OPENAI_API_KEY=your_openai_api_key
   ```

2. Configure Docusaurus to include the chatbot component:
   Update `docusaurus.config.js` to add the chat interface to desired pages.

## Running the Application

### Backend Development

1. Start the backend server:
   ```bash
   cd backend
   uvicorn src.main:app --reload --port 8000
   ```

2. The API will be available at `http://localhost:8000`

### Frontend Development

1. Start the Docusaurus development server:
   ```bash
   cd hackathon-ai-book
   npm run start
   ```

2. The frontend will be available at `http://localhost:3000`

### Running Tests

1. Backend tests:
   ```bash
   cd backend
   pytest
   ```

2. Frontend tests:
   ```bash
   cd frontend
   npm test
   ```

## Initializing the RAG System

1. Process the book content to generate embeddings:
   ```bash
   curl -X POST http://localhost:8000/v1/embeddings/process-book \
     -H "Content-Type: application/json" \
     -d '{
       "book_path": "/path/to/book/docs",
       "reprocess_all": false
     }'
   ```

2. Monitor the processing job status:
   ```bash
   curl http://localhost:8000/v1/embeddings/status/{job_id}
   ```

## Creating a New Chat Session

1. Start a new session:
   ```bash
   curl -X POST http://localhost:8000/v1/chat/start \
     -H "Content-Type: application/json" \
     -d '{
       "user_id": "user-123",
       "initial_query": "What is ROS 2?"
     }'
   ```

2. Send messages to the session:
   ```bash
   curl -X POST http://localhost:8000/v1/chat/{session_id}/message \
     -H "Content-Type: application/json" \
     -d '{
       "content": "How do I create a publisher in ROS 2?"
     }'
   ```

## Architecture Overview

The RAG Chatbot system consists of:

1. **Frontend** (Docusaurus): User interface with chat component
2. **Backend** (FastAPI): API server handling requests and RAG logic
3. **Vector Storage** (Qdrant): Stores document embeddings for semantic search
4. **Relational Storage** (Neon Postgres): Stores chat history and session data
5. **AI Services**: Cohere for embeddings, OpenAI Agents for responses

## Deployment

1. The Docusaurus frontend can be built and deployed to GitHub Pages:
   ```bash
   npm run build
   # Deployment to GitHub Pages via GitHub Actions
   ```

2. The FastAPI backend can be deployed as a container or serverless function depending on your infrastructure.

3. Ensure all environment variables are properly configured in your deployment environment.