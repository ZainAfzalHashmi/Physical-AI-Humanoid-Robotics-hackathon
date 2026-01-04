"""
Qdrant Service Module

This module provides functions for interacting with the Qdrant vector database.
It handles operations like creating collections, uploading embeddings, 
searching for similar content, and managing the vector database.
"""

import asyncio
from typing import List, Dict, Optional, Any
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct
import logging
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Initialize logger
logger = logging.getLogger(__name__)

class QdrantService:
    def __init__(self):
        """
        Initialize the Qdrant service with connection parameters from environment variables.
        """
        # Check for QDRANT_URL first (as it appears in your .env file), fallback to QDRANT_HOST
        self.host = os.getenv("QDRANT_URL", os.getenv("QDRANT_HOST", "localhost"))
        # Extract just the host part from the URL if it's a full URL
        if self.host.startswith("https://"):
            self.host = self.host[8:]  # Remove "https://"
        elif self.host.startswith("http://"):
            self.host = self.host[7:]  # Remove "http://"

        self.port = int(os.getenv("QDRANT_PORT", 6333))
        self.api_key = os.getenv("QDRANT_API_KEY")
        self.collection_name = os.getenv("QDRANT_COLLECTION_NAME", "book_content_chunks")
        
        # Set up the client
        print(f"Qdrant config - Host: {self.host}, Port: {self.port}, API Key exists: {bool(self.api_key)}")
        if self.api_key and self.host != "localhost":
            # For Qdrant Cloud - ensure URL includes https://
            if not self.host.startswith("https://") and not self.host.startswith("http://"):
                self.host = f"https://{self.host}"
            print(f"Connecting to Qdrant Cloud at: {self.host}")
            self.client = QdrantClient(
                url=self.host,
                api_key=self.api_key
            )
        elif self.api_key and self.host == "localhost":
            # When host is localhost but API key exists, likely user forgot to set the proper cloud URL
            print("ERROR: Host is set to localhost but API key exists. Please update your QDRANT_HOST with your cloud instance URL in the .env file")
            raise Exception("Invalid Qdrant configuration: Host is localhost but API key exists")
        else:
            # For local development
            print(f"Connecting to local Qdrant at: {self.host}:{self.port}")
            self.client = QdrantClient(host=self.host, port=self.port)
        
        logger.info(f"QdrantService initialized with host: {self.host}, collection: {self.collection_name}")

    def create_collection(self, vector_size: int = 1024) -> bool:
        """
        Create a collection in Qdrant to store book content embeddings.
        
        Args:
            vector_size: The size of the embedding vectors
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Define the collection configuration
            collection_config = models.CreateCollection(
                vectors={
                    "content": models.VectorParams(
                        size=vector_size,
                        distance=models.Distance.COSINE
                    )
                },
                # Define payload schema
                optimizers_config=models.OptimizersConfigDiff(
                    memmap_threshold=20000,
                )
            )
            
            # Check if collection already exists
            collections = self.client.get_collections()
            collection_names = [collection.name for collection in collections.collections]

            if self.collection_name in collection_names:
                logger.info(f"Collection '{self.collection_name}' already exists")
                return True

            # Create the collection
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=collection_config.vectors
            )
            
            logger.info(f"Collection '{self.collection_name}' created successfully")
            return True
            
        except Exception as e:
            logger.error(f"Error creating collection: {str(e)}")
            return False

    def add_embeddings(self, points: List[Dict]) -> bool:
        """
        Add multiple embeddings to the Qdrant collection.
        
        Args:
            points: List of dictionaries containing id, vector, and payload
            
        Returns:
            True if successful, False otherwise
        """
        try:
            qdrant_points = []
            for point in points:
                qdrant_point = PointStruct(
                    id=point["id"],
                    vector={"content": point["vector"]},  # Updated for new API
                    payload=point["payload"]
                )
                qdrant_points.append(qdrant_point)
            
            self.client.upsert(
                collection_name=self.collection_name,
                points=qdrant_points
            )
            
            logger.info(f"Added {len(points)} embeddings to collection '{self.collection_name}'")
            return True
            
        except Exception as e:
            logger.error(f"Error adding embeddings: {str(e)}")
            return False

    def search_similar(self, query_vector: List[float], limit: int = 5) -> List[Dict]:
        """
        Search for similar content based on the query vector.

        Args:
            query_vector: The embedding vector to search for
            limit: Maximum number of results to return

        Returns:
            List of similar content with scores
        """
        try:
            # Use the correct vector name for search
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector={"content": query_vector},  # Updated for new API
                limit=limit
            )

            results = []
            for result in search_results:
                results.append({
                    "id": result.id,
                    "payload": result.payload,
                    "score": result.score
                })

            logger.info(f"Found {len(results)} similar items")
            return results

        except Exception as e:
            logger.error(f"Error searching for similar content: {str(e)}")
            return []

    def get_content_by_ids(self, ids: List[str]) -> List[Dict]:
        """
        Retrieve specific content by their IDs.

        Args:
            ids: List of content IDs to retrieve

        Returns:
            List of content items
        """
        try:
            points = self.client.retrieve(
                collection_name=self.collection_name,
                ids=ids,
                with_payload=True,
                with_vectors=False
            )

            results = []
            for point in points:
                results.append({
                    "id": point.id,
                    "payload": point.payload
                })

            logger.info(f"Retrieved {len(results)} content items by ID")
            return results

        except Exception as e:
            logger.error(f"Error retrieving content by IDs: {str(e)}")
            return []

    def delete_collection(self) -> bool:
        """
        Delete the entire collection and all its contents.
        
        Returns:
            True if successful, False otherwise
        """
        try:
            self.client.delete_collection(collection_name=self.collection_name)
            logger.info(f"Collection '{self.collection_name}' deleted successfully")
            return True
        except Exception as e:
            logger.error(f"Error deleting collection: {str(e)}")
            return False

    def get_collection_info(self) -> Optional[Dict]:
        """
        Get information about the collection including point count.

        Returns:
            Collection information or None if error
        """
        try:
            collection_info = self.client.get_collection(collection_name=self.collection_name)
            info = {
                "status": collection_info.status,
                "vectors_count": getattr(collection_info, 'vectors_count', getattr(collection_info, 'points_count', 0)),
                "segments_count": getattr(collection_info, 'segments_count', 0),
                "config": getattr(collection_info, 'config', {})
            }
            logger.info(f"Collection info retrieved: {info}")
            return info
        except Exception as e:
            logger.error(f"Error getting collection info: {str(e)}")
            return None


# Example usage
if __name__ == "__main__":
    # Initialize the service
    qdrant_service = QdrantService()
    
    # Example of creating a collection
    success = qdrant_service.create_collection(vector_size=1024)
    if success:
        print("Collection created successfully")
    else:
        print("Failed to create collection")