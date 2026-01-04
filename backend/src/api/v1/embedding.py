"""
Embedding API Endpoints

This module defines the API endpoints for embedding functionality in the RAG system.
"""

from fastapi import APIRouter, HTTPException, BackgroundTasks
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
import logging
import asyncio
from src.services.rag import RAGService

# Initialize logging
logger = logging.getLogger(__name__)

# Create API router
router = APIRouter()

# Initialize the RAG service
rag_service = RAGService()

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

@router.post("/process-book", response_model=ProcessBookResponse)
async def process_book_content(request: ProcessBookRequest, background_tasks: BackgroundTasks):
    """
    Process book content from the specified directory, generate embeddings, and store in Qdrant.
    """
    try:
        # This would typically be a background job in a real implementation
        # For now, we'll run it synchronously but in a real system you'd queue this
        success = await rag_service.process_book_content(request.book_path)
        
        job_id = f"job_{hash(request.book_path)}_{id(request)}"
        status = "completed" if success else "failed"
        
        if success:
            return ProcessBookResponse(
                job_id=job_id,
                status=status
            )
        else:
            raise HTTPException(status_code=500, detail="Failed to process book content")
            
    except Exception as e:
        logger.error(f"Error processing book content: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error processing book content: {str(e)}")

@router.get("/status/{job_id}", response_model=JobStatusResponse)
async def get_job_status(job_id: str):
    """
    Get the status of a book embedding processing job.
    """
    # In a real implementation, this would track actual job progress
    # For now, we'll return a mock response
    
    # Get collection info to provide actual progress
    collection_info = rag_service.get_collection_info()
    if collection_info:
        processed_chunks = collection_info.get('vectors_count', 0)
        total_chunks = processed_chunks  # In a real scenario, we'd know the total in advance
        progress = 100.0 if processed_chunks > 0 else 0.0
    else:
        processed_chunks = 0
        total_chunks = 0
        progress = 0.0

    return JobStatusResponse(
        job_id=job_id,
        status="completed" if progress == 100.0 else "processing",
        progress=progress,
        details={
            "processed_chunks": processed_chunks,
            "total_chunks": total_chunks
        }
    )