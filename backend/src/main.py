"""
Main FastAPI Application for the ROS 2 Book RAG Chatbot

This file sets up the FastAPI application, includes the API routes,
and initializes the required services like the RAG service that
integrates Cohere and Qdrant.
"""

from fastapi import FastAPI, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
import logging
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Import our services
from src.services.rag import RAGService
from src.services.qdrant_service import QdrantService
from src.services.embedding import CohereEmbeddingService

# Initialize logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Create the main FastAPI app
app = FastAPI(
    title="ROS 2 Book RAG API",
    description="API for the RAG chatbot system integrated with the ROS 2 book project",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize services
rag_service = RAGService()

class ChatRequest(BaseModel):
    content: str
    session_id: Optional[str] = None
    context_chunks: Optional[List[str]] = []

class ChatResponse(BaseModel):
    response: str
    relevant_chunks: List[str]
    confidence: float

class ProcessBookRequest(BaseModel):
    book_path: str
    reprocess_all: bool = False

class ProcessBookResponse(BaseModel):
    job_id: str
    status: str

class JobStatusResponse(BaseModel):
    job_id: str
    status: str
    progress: float
    details: Dict[str, Any]

class HealthResponse(BaseModel):
    status: str
    timestamp: str

class SessionResponse(BaseModel):
    id: str
    title: str
    created_at: str
    updated_at: str
    messages: List[Dict[str, Any]]

# API Routes

@app.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Health check endpoint to verify the API is running.
    """
    import datetime
    return {
        "status": "healthy",
        "timestamp": datetime.datetime.now().isoformat()
    }

@app.post("/v1/embeddings/process-book", response_model=ProcessBookResponse)
async def process_book_content(request: ProcessBookRequest):
    """
    Process book content from the specified directory, generate embeddings, and store in Qdrant.
    """
    try:
        # This would typically be a background job in a real implementation
        # For now, we'll run it synchronously
        success = await rag_service.process_book_content(request.book_path)
        
        if success:
            return ProcessBookResponse(
                job_id=f"job_{hash(request.book_path)}",
                status="completed" if success else "failed"
            )
        else:
            raise HTTPException(status_code=500, detail="Failed to process book content")
            
    except Exception as e:
        logger.error(f"Error processing book content: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error processing book content: {str(e)}")

@app.get("/v1/embeddings/status/{job_id}", response_model=JobStatusResponse)
async def get_job_status(job_id: str):
    """
    Get the status of a book embedding processing job.
    """
    # In a real implementation, this would track actual job progress
    # For now, we'll return a mock response
    return JobStatusResponse(
        job_id=job_id,
        status="completed",
        progress=100.0,
        details={"processed_chunks": 100, "total_chunks": 100}
    )

@app.post("/v1/chat/start", status_code=201)
async def start_chat_session():
    """
    Start a new chat session.
    """
    import uuid
    import datetime
    
    # In a real implementation, this would create a session in the database
    session_id = str(uuid.uuid4())
    title = "New ROS 2 Chat Session"
    
    # Mock response - in reality, you'd create a session in the database
    return {
        "session_id": session_id,
        "title": title,
        "created_at": datetime.datetime.now().isoformat()
    }

@app.post("/v1/chat/{session_id}/message", response_model=ChatResponse)
async def send_chat_message(session_id: str, request: ChatRequest):
    """
    Send a message in a chat session and get a response from the AI.
    """
    try:
        # Retrieve relevant content from the RAG system
        relevant_content = rag_service.retrieve_relevant_content(request.content)
        
        # Format the content for the LLM prompt
        context_texts = [item["content"] for item in relevant_content]
        context = "\n\n".join(context_texts)
        
        # In a real implementation, you would call an LLM (like OpenAI) here
        # with the retrieved context to generate a response.
        # For demonstration, we'll return a mock response.
        
        # Mock response - in a real implementation, you would:
        # 1. Call OpenAI's API with the context and user's query
        # 2. Generate a relevant response
        # 3. Return that response with relevant chunks and confidence
        mock_response = f"I found {len(relevant_content)} relevant sections in the ROS 2 book related to your query: '{request.content}'. Would you like me to explain more about ROS 2 publishers and subscribers?"
        
        relevant_chunk_ids = [item["id"] for item in relevant_content]
        
        return ChatResponse(
            response=mock_response,
            relevant_chunks=relevant_chunk_ids,
            confidence=0.85  # Mock confidence score
        )
        
    except Exception as e:
        logger.error(f"Error processing chat message: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error processing chat message: {str(e)}")

@app.get("/v1/chat/{session_id}", response_model=SessionResponse)
async def get_chat_session(session_id: str):
    """
    Get details of a specific chat session.
    """
    # In a real implementation, this would fetch session details from the database
    # For now, we'll return mock data
    import datetime
    
    return SessionResponse(
        id=session_id,
        title="Sample ROS 2 Chat Session",
        created_at=datetime.datetime.now().isoformat(),
        updated_at=datetime.datetime.now().isoformat(),
        messages=[]
    )

# Initialize services on startup
@app.on_event("startup")
async def startup_event():
    logger.info("Starting up the ROS 2 Book RAG API")
    
    # Initialize the Qdrant collection if needed
    if rag_service.qdrant_service.create_collection():
        logger.info("Qdrant collection is ready")
    else:
        logger.error("Failed to prepare Qdrant collection")

@app.on_event("shutdown")
async def shutdown_event():
    logger.info("Shutting down the ROS 2 Book RAG API")

# For running with uvicorn directly
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)