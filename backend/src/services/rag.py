"""
RAG (Retrieval-Augmented Generation) Service Module

This module provides the main RAG functionality by combining the Cohere embedding service
and Qdrant vector database service. It handles the entire pipeline from content ingestion
to similarity search and retrieval for the chatbot.
"""

import asyncio
from typing import List, Dict, Optional, Any
from .qdrant_service import QdrantService
from .embedding import CohereEmbeddingService
import logging
import os
from dotenv import load_dotenv
from pathlib import Path

# Load environment variables
load_dotenv()

# Initialize logger
logger = logging.getLogger(__name__)

class RAGService:
    def __init__(self):
        """
        Initialize the RAG service with Cohere and Qdrant services.
        """
        self.qdrant_service = QdrantService()
        self.cohere_service = CohereEmbeddingService()
        
        logger.info("RAG Service initialized with Cohere and Qdrant services")

    async def process_book_content(self, book_content_dir: str) -> bool:
        """
        Process all book content from the specified directory, generate embeddings,
        and store them in Qdrant.
        
        Args:
            book_content_dir: Path to the directory containing book content
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Create the Qdrant collection if it doesn't exist
            if not self.qdrant_service.create_collection():
                logger.error("Failed to create Qdrant collection")
                return False
            
            # Find all markdown files in the book content directory
            content_dir_path = Path(book_content_dir)
            markdown_files = list(content_dir_path.rglob("*.md"))
            
            logger.info(f"Found {len(markdown_files)} markdown files to process")
            
            # Process each markdown file
            processed_count = 0
            for file_path in markdown_files:
                try:
                    with open(file_path, 'r', encoding='utf-8') as f:
                        content = f.read()
                    
                    # Preprocess the content into chunks
                    content_chunks = self.preprocess_book_content(content, str(file_path))

                    # Create content chunk objects for embedding
                    content_objects = []
                    for i, chunk in enumerate(content_chunks):
                        # Generate a valid numeric ID - use a combination of file hash and chunk index
                        file_hash = abs(hash(file_path.name)) % 1000000
                        chunk_id = file_hash * 1000 + i  # This creates a unique integer ID
                        content_object = {
                            "id": chunk_id,
                            "content": chunk,
                            "metadata": {
                                "source_file": str(file_path),
                                "chunk_index": i,
                                "original_file": file_path.name
                            }
                        }
                        content_objects.append(content_object)
                    
                    # Process content chunks through the Cohere service
                    qdrant_points = self.cohere_service.process_content_chunks(content_objects)
                    
                    if qdrant_points:
                        # Add embeddings to Qdrant
                        if self.qdrant_service.add_embeddings(qdrant_points):
                            processed_count += len(content_objects)
                            logger.info(f"Processed {len(content_objects)} chunks from {file_path.name}")
                        else:
                            logger.error(f"Failed to add embeddings to Qdrant for {file_path.name}")
                    else:
                        logger.warning(f"No embeddings generated for {file_path.name}")
                        
                except Exception as e:
                    logger.error(f"Error processing file {file_path}: {str(e)}")
                    continue
            
            logger.info(f"Successfully processed {processed_count} content chunks from {len(markdown_files)} files")
            return True
            
        except Exception as e:
            logger.error(f"Error processing book content: {str(e)}")
            return False

    def preprocess_book_content(self, content: str, source_file: str) -> List[str]:
        """
        Preprocess book content by breaking it into chunks suitable for embedding.
        
        Args:
            content: Raw book content as a string
            source_file: Path to the source file
            
        Returns:
            List of content chunks
        """
        # Define chunk size parameters
        max_chunk_size = 1500  # characters
        overlap_size = 200     # characters
        
        # Split content into paragraphs
        paragraphs = content.split('\n\n')
        
        chunks = []
        current_chunk = ""
        
        for paragraph in paragraphs:
            # If adding this paragraph would exceed the max size, save the current chunk
            if len(current_chunk) + len(paragraph) > max_chunk_size and current_chunk:
                chunks.append(current_chunk.strip())
                
                # Add overlap by keeping part of the previous chunk
                if len(paragraph) < max_chunk_size:
                    # Start new chunk with overlap
                    overlap_start = len(current_chunk) - overlap_size
                    if overlap_start > 0:
                        current_chunk = current_chunk[overlap_start:] + paragraph
                    else:
                        current_chunk = current_chunk + paragraph
                else:
                    current_chunk = paragraph
            else:
                # Add paragraph to current chunk
                current_chunk += '\n\n' + paragraph
        
        # Add the last chunk if it exists
        if current_chunk.strip():
            chunks.append(current_chunk.strip())
        
        # Filter out very short chunks
        chunks = [chunk for chunk in chunks if len(chunk) > 100]
        
        logger.info(f"Preprocessed {source_file} into {len(chunks)} chunks")
        return chunks

    def retrieve_relevant_content(self, query: str, top_k: int = 5) -> List[Dict]:
        """
        Retrieve content relevant to the query by generating embedding and searching Qdrant.
        
        Args:
            query: User query to find relevant content for
            top_k: Number of top results to return
            
        Returns:
            List of relevant content chunks with metadata
        """
        try:
            # Generate embedding for the query
            query_embedding = self.cohere_service.generate_single_embedding(
                query, 
                input_type="search_query"
            )
            
            if not query_embedding:
                logger.error("Failed to generate embedding for query")
                return []
            
            # Search for similar content in Qdrant
            search_results = self.qdrant_service.search_similar(
                query_vector=query_embedding,
                limit=top_k
            )
            
            relevant_content = []
            for result in search_results:
                relevant_content.append({
                    "id": result["id"],
                    "content": result["payload"]["content"],
                    "source_file": result["payload"]["source_file"],
                    "score": result["score"],
                    "original_file": result["payload"].get("original_file", "unknown")
                })
            
            logger.info(f"Retrieved {len(relevant_content)} relevant content chunks for query")
            return relevant_content
            
        except Exception as e:
            logger.error(f"Error retrieving relevant content: {str(e)}")
            return []

    def get_content_by_ids(self, ids: List[str]) -> List[Dict]:
        """
        Retrieve specific content by their IDs.
        
        Args:
            ids: List of content IDs to retrieve
            
        Returns:
            List of content chunks with metadata
        """
        try:
            # Fetch content from Qdrant by IDs
            results = self.qdrant_service.get_content_by_ids(ids)
            
            content_list = []
            for result in results:
                content_list.append({
                    "id": result["id"],
                    "content": result["payload"]["content"],
                    "source_file": result["payload"]["source_file"],
                    "original_file": result["payload"].get("original_file", "unknown")
                })
            
            logger.info(f"Retrieved {len(content_list)} content items by IDs")
            return content_list
            
        except Exception as e:
            logger.error(f"Error retrieving content by IDs: {str(e)}")
            return []

    def get_collection_info(self) -> Optional[Dict]:
        """
        Get information about the Qdrant collection.
        
        Returns:
            Collection information or None if error
        """
        return self.qdrant_service.get_collection_info()


# Example usage
if __name__ == "__main__":
    import asyncio
    
    # Initialize the service
    rag_service = RAGService()
    
    # Example: Process book content (uncomment to use with actual book directory)
    # success = asyncio.run(rag_service.process_book_content("/path/to/book/content"))
    # if success:
    #     print("Book content processed successfully")
    # else:
    #     print("Failed to process book content")
    
    # Example: Retrieve relevant content for a sample query
    sample_query = "How to create a publisher in ROS 2?"
    relevant_content = rag_service.retrieve_relevant_content(sample_query)
    
    print(f"Found {len(relevant_content)} relevant content chunks for query: '{sample_query}'")
    for i, content in enumerate(relevant_content[:2]):  # Show first 2 results
        print(f"\n{i+1}. Source: {content['source_file']}")
        print(f"   Score: {content['score']:.3f}")
        print(f"   Content preview: {content['content'][:100]}...")