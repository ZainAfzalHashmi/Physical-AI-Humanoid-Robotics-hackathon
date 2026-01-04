"""
Health Check API Endpoints

This module defines the health check endpoint for the RAG system.
"""

from fastapi import APIRouter
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
import datetime
import logging
from src.services.rag import RAGService

# Initialize logging
logger = logging.getLogger(__name__)

# Create API router
router = APIRouter()

# Initialize the RAG service
rag_service = RAGService()

class HealthResponse(BaseModel):
    status: str
    timestamp: str
    services: Dict[str, bool]

@router.get("/", response_model=HealthResponse)
async def health_check():
    """
    Health check endpoint to verify the API and its dependencies are running.
    """
    try:
        # Check if Qdrant is accessible
        collection_info = rag_service.get_collection_info()
        qdrant_healthy = collection_info is not None
        
        # Check if Cohere service is accessible by generating a simple test embedding
        try:
            test_embedding = rag_service.cohere_service.generate_single_embedding("test")
            cohere_healthy = test_embedding is not None
        except Exception:
            cohere_healthy = False
        
        # Overall status
        overall_status = "healthy" if (qdrant_healthy and cohere_healthy) else "degraded"
        
        services_status = {
            "qdrant": qdrant_healthy,
            "cohere": cohere_healthy
        }
        
        return HealthResponse(
            status=overall_status,
            timestamp=datetime.datetime.now().isoformat(),
            services=services_status
        )
    except Exception as e:
        logger.error(f"Health check failed: {str(e)}")
        return HealthResponse(
            status="unhealthy",
            timestamp=datetime.datetime.now().isoformat(),
            services={
                "qdrant": False,
                "cohere": False
            }
        )