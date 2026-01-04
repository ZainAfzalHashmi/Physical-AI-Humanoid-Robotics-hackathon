"""
Cohere Service Module

This module provides functions for interacting with the Cohere API to generate embeddings
from text content. It handles operations like text preprocessing, embedding generation,
and batch processing of content chunks.
"""

import asyncio
import cohere
from typing import List, Dict, Optional, Union
import logging
import os
from dotenv import load_dotenv
import numpy as np
from tenacity import retry, stop_after_attempt, wait_exponential

# Load environment variables
load_dotenv()

# Initialize logger
logger = logging.getLogger(__name__)

class CohereEmbeddingService:
    def __init__(self):
        """
        Initialize the Cohere embedding service with API key from environment variables.
        """
        self.api_key = os.getenv("COHERE_API_KEY")
        if not self.api_key:
            raise ValueError("COHERE_API_KEY environment variable is not set")
        
        self.client = cohere.Client(self.api_key)
        self.model = os.getenv("COHERE_MODEL", "embed-english-v3.0")
        
        logger.info(f"CohereEmbeddingService initialized with model: {self.model}")

    @retry(stop=stop_after_attempt(3), wait=wait_exponential(multiplier=1, min=4, max=10))
    def generate_embeddings(self, texts: List[str], input_type: str = "search_document") -> Optional[List[List[float]]]:
        """
        Generate embeddings for a list of texts using Cohere API.
        
        Args:
            texts: List of text strings to generate embeddings for
            input_type: Type of input (search_document, search_query, etc.)
            
        Returns:
            List of embeddings (each embedding is a list of floats) or None if error
        """
        try:
            # Cohere API has a limit on the number of texts per request
            # For safety, we'll keep batches under 96 (the actual limit is 96)
            batch_size = min(96, len(texts))
            all_embeddings = []
            
            # Process in batches if needed
            for i in range(0, len(texts), batch_size):
                batch = texts[i:i + batch_size]
                
                response = self.client.embed(
                    texts=batch,
                    model=self.model,
                    input_type=input_type
                )
                
                batch_embeddings = [embedding for embedding in response.embeddings]
                all_embeddings.extend(batch_embeddings)
                
                logger.info(f"Processed batch {i//batch_size + 1}/{(len(texts)-1)//batch_size + 1}, "
                           f"received {len(batch_embeddings)} embeddings")
            
            logger.info(f"Successfully generated {len(all_embeddings)} embeddings for {len(texts)} texts")
            return all_embeddings
            
        except Exception as e:
            logger.error(f"Error generating embeddings: {str(e)}")
            raise e

    def generate_single_embedding(self, text: str, input_type: str = "search_document") -> Optional[List[float]]:
        """
        Generate a single embedding for a text string.
        
        Args:
            text: Text string to generate embedding for
            input_type: Type of input (search_document, search_query, etc.)
            
        Returns:
            Embedding as a list of floats or None if error
        """
        try:
            embeddings = self.generate_embeddings([text], input_type)
            if embeddings and len(embeddings) > 0:
                return embeddings[0]
            return None
        except Exception as e:
            logger.error(f"Error generating single embedding: {str(e)}")
            return None

    def preprocess_text(self, text: str, max_length: int = 4096) -> List[str]:
        """
        Preprocess text by cleaning and splitting into appropriate chunks.
        
        Args:
            text: Input text to preprocess
            max_length: Maximum length of text chunks
            
        Returns:
            List of text chunks
        """
        # Clean the text
        cleaned_text = text.strip()
        
        # If the text is already short enough, return as is
        if len(cleaned_text) <= max_length:
            return [cleaned_text]
        
        # Otherwise, split the text into chunks
        chunks = []
        sentences = cleaned_text.split('. ')
        
        current_chunk = ""
        for sentence in sentences:
            # Add a period back if it was removed
            sentence_with_period = sentence + '.'
            
            if len(current_chunk) + len(sentence_with_period) <= max_length:
                current_chunk += ' ' + sentence_with_period
            else:
                if current_chunk:
                    chunks.append(current_chunk.strip())
                current_chunk = sentence_with_period
        
        # Add the last chunk if it exists
        if current_chunk.strip():
            chunks.append(current_chunk.strip())
        
        return chunks

    def process_content_chunks(self, content_chunks: List[Dict]) -> List[Dict]:
        """
        Process a list of content chunks by generating embeddings for each.
        
        Args:
            content_chunks: List of content chunks, each with 'id' and 'content' keys
            
        Returns:
            List of dictionaries with 'id', 'vector', and 'payload' for Qdrant
        """
        try:
            # Extract just the content texts to generate embeddings for
            texts = [chunk['content'] for chunk in content_chunks]
            
            # Generate embeddings
            embeddings = self.generate_embeddings(texts)
            
            if not embeddings or len(embeddings) != len(content_chunks):
                logger.error("Mismatch between number of texts and generated embeddings")
                return []
            
            # Format the results for Qdrant
            qdrant_points = []
            for i, embedding in enumerate(embeddings):
                qdrant_point = {
                    "id": content_chunks[i]['id'],
                    "vector": embedding,
                    "payload": content_chunks[i].get('metadata', {})
                }
                qdrant_points.append(qdrant_point)
            
            logger.info(f"Processed {len(content_chunks)} content chunks into Qdrant points")
            return qdrant_points
            
        except Exception as e:
            logger.error(f"Error processing content chunks: {str(e)}")
            return []

    def calculate_similarity(self, vector1: List[float], vector2: List[float]) -> float:
        """
        Calculate cosine similarity between two vectors.
        
        Args:
            vector1: First embedding vector
            vector2: Second embedding vector
            
        Returns:
            Cosine similarity score between -1 and 1
        """
        try:
            v1 = np.array(vector1)
            v2 = np.array(vector2)
            
            # Calculate cosine similarity
            dot_product = np.dot(v1, v2)
            norm_v1 = np.linalg.norm(v1)
            norm_v2 = np.linalg.norm(v2)
            
            if norm_v1 == 0 or norm_v2 == 0:
                return 0.0
            
            similarity = dot_product / (norm_v1 * norm_v2)
            return float(similarity)
        except Exception as e:
            logger.error(f"Error calculating similarity: {str(e)}")
            return 0.0


# Example usage
if __name__ == "__main__":
    # Initialize the service
    cohere_service = CohereEmbeddingService()
    
    # Example text
    example_texts = [
        "ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software.",
        "In ROS 2, nodes are C++ or Python programs that communicate with each other.",
        "Publishers and subscribers in ROS 2 use topics to communicate messages."
    ]
    
    # Generate embeddings
    try:
        embeddings = cohere_service.generate_embeddings(example_texts)
        if embeddings:
            print(f"Generated {len(embeddings)} embeddings")
            print(f"First embedding has {len(embeddings[0])} dimensions")
        else:
            print("Failed to generate embeddings")
    except Exception as e:
        print(f"Error: {str(e)}")