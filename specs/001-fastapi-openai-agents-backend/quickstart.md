# Quickstart Guide: FastAPI Backend with OpenAI Agents SDK

## Prerequisites

- Python 3.11+
- Access to OpenAI API key
- Access to Qdrant vector database (with ROS 2 book content already embedded)
- Access to Neon Postgres database
- Git for version control

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
   # Update .env with your API keys and connection details
   ```

## Configuration

1. Update `backend/.env` with your service configurations:
   ```
   OPENAI_API_KEY=your_openai_api_key
   QDRANT_API_KEY=your_qdrant_api_key
   QDRANT_HOST=your_qdrant_instance.eu-west1.aws.cloud.qdrant.io
   QDRANT_PORT=6333
   QDRANT_COLLECTION_NAME=book_content_chunks
   DATABASE_URL=postgresql://username:password@neon_instance.neon.tech/dbname
   SECRET_KEY=your_secret_key
   ALGORITHM=HS256
   ACCESS_TOKEN_EXPIRE_MINUTES=30
   ```

2. Configure application settings in `backend/src/core/config.py`:
   ```python
   class Settings(BaseSettings):
       # API Settings
       api_v1_prefix: str = "/v1"
       debug: bool = False
       project_name: str = "ROS 2 Book RAG Chatbot"
       version: str = "1.0.0"
       allowed_origins: List[str] = ["http://localhost", "http://localhost:3000"]
       
       # Database Settings
       database_url: str
       
       # Qdrant Settings
       qdrant_host: str
       qdrant_port: int = 6333
       qdrant_api_key: str
       qdrant_collection_name: str = "book_content_chunks"
       
       # OpenAI Settings
       openai_api_key: str
       openai_model: str = "gpt-4-turbo-preview"
       embedding_model: str = "text-embedding-ada-002"
       
       # RAG Settings
       retrieval_top_k: int = 5
       similarity_threshold: float = 0.7
       max_tokens_response: int = 1000
       
       class Config:
           env_file = ".env"
   ```

## Running the Backend Server

### Development Mode

1. Start the backend server in development mode:
   ```bash
   cd backend
   uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
   ```

2. The API will be available at `http://localhost:8000`
3. The automatic API documentation will be available at `http://localhost:8000/docs`

### Production Mode

1. Build the Docker container:
   ```bash
   docker build -t ros2-book-rag-backend .
   ```

2. Run the container:
   ```bash
   docker run -d -p 8000:8000 --env-file ./backend/.env ros2-book-rag-backend
   ```

## API Usage

### Chat Endpoint

Submit a chat message to get an AI-generated response based on book content:

```bash
curl -X POST "http://localhost:8000/v1/chat" \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What is rclpy?",
    "conversation_id": "convo_12345",
    "user_id": "user_67890"
  }'
```

### Retrieve Endpoint

Retrieve relevant book content based on a query:

```bash
curl -X POST "http://localhost:8000/v1/retrieve" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "How do I create a ROS 2 publisher in Python?",
    "top_k": 3,
    "threshold": 0.7
  }'
```

### Get Conversation History

Retrieve the history of a specific conversation:

```bash
curl -X GET "http://localhost:8000/v1/conversations/convo_12345/messages" \
  -H "Authorization: Bearer your_token_here"
```

## Integration with Frontend

The backend provides these endpoints for integration with your Docusaurus frontend:

### Starting a New Conversation
```javascript
const startConversation = async () => {
  const response = await fetch('/v1/chat/start', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ user_id: 'current_user_id' })
  });
  const { conversation_id } = await response.json();
  return conversation_id;
};
```

### Sending a Message
```javascript
const sendMessage = async (conversationId, message) => {
  const response = await fetch('/v1/chat', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      message,
      conversation_id: conversationId,
      user_id: 'current_user_id'
    })
  });
  const data = await response.json();
  return data;
};
```

## Testing

### Running Unit Tests

1. Execute all unit tests:
   ```bash
   cd backend
   python -m pytest tests/unit/ -v
   ```

### Running Integration Tests

1. Execute all integration tests:
   ```bash
   cd backend
   python -m pytest tests/integration/ -v
   ```

### Running All Tests

1. Execute all tests with coverage:
   ```bash
   cd backend
   python -m pytest --cov=src --cov-report=html
   ```

## Architecture Overview

The FastAPI backend consists of:

1. **API Layer** (`src/api/`): FastAPI routers for handling HTTP requests
2. **Core Logic** (`src/core/`): Configuration, security, and LLM/RAG integrations
3. **Services Layer** (`src/services/`): Business logic for chat, retrieval, grounding, and history management
4. **Models Layer** (`src/models/`): Database models and Pydantic schemas
5. **Utilities** (`src/utils/`): Helper functions and utilities

## Troubleshooting

1. **OpenAI API Errors**: Verify that your OPENAI_API_KEY is valid and has sufficient credits
2. **Qdrant Connection Issues**: Check that your Qdrant credentials are correct and the instance is accessible
3. **Database Errors**: Ensure your Neon Postgres connection string is correct and you have proper permissions
4. **Slow Response Times**: Verify that your Qdrant instance has enough resources and the embeddings are properly indexed

## Performance Tuning

1. **Adjust Top-K Settings**: Modify the number of context chunks retrieved based on response quality vs. speed
2. **Similarity Thresholds**: Tune the threshold for content relevance to improve grounding quality
3. **Rate Limiting**: Configure appropriate rate limits based on your API usage patterns and costs
4. **Caching**: Consider implementing caching for frequently accessed content or conversation fragments