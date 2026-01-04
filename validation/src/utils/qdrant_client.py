"""
Qdrant client wrapper for the RAG validation system.
Provides a simplified interface to interact with Qdrant for validation purposes.
"""
import os
import time
import logging
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct
import cohere
import numpy as np


class QdrantClientWrapper:
    """
    Wrapper around QdrantClient to provide methods specifically for validation purposes.
    """
    
    def __init__(self, host: str, port: int, api_key: str, collection_name: str):
        self.logger = logging.getLogger(__name__)
        self.collection_name = collection_name
        
        # Initialize Qdrant client
        if api_key:
            # For Qdrant Cloud
            if not host.startswith("https://") and not host.startswith("http://"):
                host = f"https://{host}"
            self.client = QdrantClient(
                url=host,
                api_key=api_key
            )
        else:
            # For local development
            self.client = QdrantClient(host=host, port=port)
        
        # Initialize Cohere client for embedding generation if API key is available
        cohere_api_key = os.getenv("COHERE_API_KEY")
        if cohere_api_key:
            self.cohere_client = cohere.Client(cohere_api_key)
            self.cohere_model = os.getenv("COHERE_MODEL", "embed-english-v3.0")
        else:
            self.cohere_client = None
            self.logger.warning("COHERE_API_KEY not found in environment. Embedding generation may not work.")
        
        self.logger.info(f"QdrantClientWrapper initialized for collection: {self.collection_name}")
    
    def generate_embedding(self, text: str) -> Optional[List[float]]:
        """
        Generate an embedding for the given text using Cohere.
        
        Args:
            text: Text to generate embedding for
            
        Returns:
            List of floats representing the embedding, or None if failed.
        """
        if not self.cohere_client:
            self.logger.error("Cohere client not initialized. Cannot generate embeddings.")
            return None
        
        try:
            # Generate embedding using Cohere
            response = self.cohere_client.embed(
                texts=[text],
                model=self.cohere_model,
                input_type="search_query"  # Using search_query type for queries
            )
            
            # Extract the embedding from the response
            if response.embeddings and len(response.embeddings) > 0:
                return response.embeddings[0]
            else:
                self.logger.error("No embeddings returned from Cohere API.")
                return None
                
        except Exception as e:
            self.logger.error(f"Error generating embedding for text '{text[:50]}...': {str(e)}")
            return None
    
    def search_similar(self, query_vector: List[float], limit: int = 5) -> List[Dict]:
        """
        Search for similar content based on the query vector.
        
        Args:
            query_vector: The embedding vector to search for
            limit: Maximum number of results to return
            
        Returns:
            List of similar content with scores.
        """
        try:
            # Use the collection's vector name (assuming 'content' based on our setup)
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector={"content": query_vector},  # Updated for new API
                limit=limit
            )
            
            results = []
            for result in search_results:
                results.append({
                    "id": result.id,
                    "content": result.payload.get("content", "") if result.payload else "",
                    "metadata": result.payload or {},
                    "score": result.score
                })
                
            self.logger.debug(f"Found {len(results)} similar items for query")
            return results
            
        except Exception as e:
            self.logger.error(f"Error searching for similar content: {str(e)}")
            return []
    
    def get_sample_chunks_by_module(self, module_name: str) -> List[Dict]:
        """
        Get sample chunks from a specific module.
        
        Args:
            module_name: Name of the module to get samples from
            
        Returns:
            List of content chunks from the specified module.
        """
        try:
            # Search for points where the module field matches
            scroll_result = self.client.scroll(
                collection_name=self.collection_name,
                scroll_filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="module",
                            match=models.MatchValue(value=module_name)
                        )
                    ]
                ),
                limit=10  # Get up to 10 sample chunks
            )
            
            results = []
            for point in scroll_result[0]:  # scroll returns (points, next_page_offset)
                results.append({
                    "id": point.id,
                    "content": point.payload.get("content", "") if point.payload else "",
                    "metadata": point.payload or {}
                })
            
            self.logger.debug(f"Retrieved {len(results)} sample chunks for module '{module_name}'")
            return results
            
        except Exception as e:
            self.logger.error(f"Error getting sample chunks for module '{module_name}': {str(e)}")
            return []
    
    def get_collection_info(self) -> Optional[Dict]:
        """
        Get information about the collection including point count.
        
        Returns:
            Collection information or None if error.
        """
        try:
            collection_info = self.client.get_collection(collection_name=self.collection_name)
            info = {
                "status": getattr(collection_info, 'status', 'unknown'),
                "vectors_count": getattr(collection_info, 'vectors_count', getattr(collection_info, 'points_count', 0)),
                "segments_count": getattr(collection_info, 'segments_count', 0),
                "config": getattr(collection_info, 'config', {})
            }
            self.logger.debug(f"Collection info retrieved: {info}")
            return info
        except Exception as e:
            self.logger.error(f"Error getting collection info: {str(e)}")
            return None
    
    def validate_connection(self) -> bool:
        """
        Validate that the connection to Qdrant is working.
        
        Returns:
            True if connection is valid, False otherwise.
        """
        try:
            # Try to get collection info to verify connection
            info = self.get_collection_info()
            return info is not None
        except Exception as e:
            self.logger.error(f"Connection validation failed: {str(e)}")
            return False


# Example usage
if __name__ == "__main__":
    # Example of how to use this client wrapper
    import os
    
    # Get configuration from environment variables
    host = os.getenv("QDRANT_HOST", "localhost")
    port = int(os.getenv("QDRANT_PORT", 6333))
    api_key = os.getenv("QDRANT_API_KEY")
    collection_name = os.getenv("QDRANT_COLLECTION_NAME", "book_content_chunks")
    
    # Initialize the client wrapper
    qdrant_wrapper = QdrantClientWrapper(
        host=host,
        port=port,
        api_key=api_key,
        collection_name=collection_name
    )
    
    # Test the connection
    if qdrant_wrapper.validate_connection():
        print("Qdrant connection is valid")
        
        # Example: Generate embedding for a test query
        query = "What is rclpy?"
        embedding = qdrant_wrapper.generate_embedding(query)
        
        if embedding:
            print(f"Generated embedding with {len(embedding)} dimensions")
            
            # Example: Search for similar content
            results = qdrant_wrapper.search_similar(embedding, limit=3)
            print(f"Found {len(results)} similar results")
            
            for i, result in enumerate(results):
                print(f"Result {i+1}: Score={result['score']:.3f}, Content='{result['content'][:100]}...'")
        else:
            print("Failed to generate embedding")
    else:
        print("Qdrant connection validation failed")