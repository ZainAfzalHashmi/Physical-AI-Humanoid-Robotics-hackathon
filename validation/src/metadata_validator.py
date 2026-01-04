"""
Validator for metadata integrity in RAG retrieval.
Tests that metadata correctly identifies content (e.g., "Module 1: ROS 2" chapters).
"""
import time
import logging
from typing import List, Dict, Any
from src.utils.qdrant_client import QdrantClientWrapper
from src.config.test_config import TestConfig


class MetadataValidator:
    """
    Validates the metadata integrity of retrieved content.
    Ensures that metadata correctly identifies content such as "Module 1: ROS 2" chapters.
    """
    
    def __init__(self, qdrant_client: QdrantClientWrapper, config: TestConfig):
        self.qdrant_client = qdrant_client
        self.config = config
        self.logger = logging.getLogger(__name__)
    
    def validate_query_metadata(self, query: str, retrieved_chunks: List[Dict]) -> Dict[str, Any]:
        """
        Validate metadata for results of a specific query.
        
        Args:
            query: The original query
            retrieved_chunks: List of chunks retrieved for the query
            
        Returns:
            Dictionary containing metadata validation results.
        """
        start_time = time.time()
        
        try:
            if not retrieved_chunks:
                return {
                    "success": False,
                    "error_message": "No content retrieved for metadata validation",
                    "query": query,
                    "metadata_accuracy": 0.0,
                    "valid_metadata_count": 0,
                    "total_chunks": 0,
                    "latency_ms": (time.time() - start_time) * 1000,
                    "details": {}
                }
            
            # Validate metadata for each retrieved chunk
            metadata_validation_results = []
            for chunk in retrieved_chunks:
                validation_result = self._validate_chunk_metadata(chunk)
                metadata_validation_results.append(validation_result)
            
            # Calculate overall metadata accuracy
            valid_count = sum(1 for v in metadata_validation_results if v["valid"])
            total_count = len(metadata_validation_results)
            accuracy = valid_count / total_count if total_count > 0 else 0.0
            
            # Check if metadata meets specific requirements for the query
            query_specific_pass = self._check_query_specific_metadata(query, retrieved_chunks)
            
            latency_ms = (time.time() - start_time) * 1000
            
            return {
                "success": accuracy > 0.8 and query_specific_pass,  # Consider successful if >80% accuracy
                "query": query,
                "metadata_accuracy": accuracy,
                "valid_metadata_count": valid_count,
                "total_chunks": total_count,
                "latency_ms": latency_ms,
                "detailed_results": metadata_validation_results,
                "query_specific_check": query_specific_pass,
                "error_message": None,
                "details": {
                    "expected_module": "Module 1: ROS 2" if "ROS 2" in query else None,
                    "valid_modules_found": [
                        chunk.get("metadata", {}).get("module") 
                        for chunk in retrieved_chunks 
                        if chunk.get("metadata", {}).get("module")
                    ]
                }
            }
            
        except Exception as e:
            latency_ms = (time.time() - start_time) * 1000
            self.logger.error(f"Error validating metadata for query '{query}': {str(e)}")
            return {
                "success": False,
                "error_message": str(e),
                "query": query,
                "metadata_accuracy": 0.0,
                "valid_metadata_count": 0,
                "total_chunks": len(retrieved_chunks) if retrieved_chunks else 0,
                "latency_ms": latency_ms,
                "detailed_results": [],
                "query_specific_check": False,
                "details": {"exception": str(e)}
            }
    
    def validate_module_metadata(self, module_name: str, expected_chapters: List[str] = None) -> Dict[str, Any]:
        """
        Validate metadata specifically for a module (e.g., "Module 1: ROS 2").
        
        Args:
            module_name: The name of the module to validate
            expected_chapters: List of expected chapters in the module (optional)
            
        Returns:
            Dictionary containing module-specific metadata validation results.
        """
        start_time = time.time()
        
        try:
            # Get sample chunks from this module
            sample_chunks = self.qdrant_client.get_sample_chunks_by_module(module_name)
            
            if not sample_chunks:
                return {
                    "success": False,
                    "module": module_name,
                    "error_message": f"No content found for module: {module_name}",
                    "metadata_accuracy": 0.0,
                    "valid_metadata_count": 0,
                    "total_chunks_sampled": 0,
                    "latency_ms": (time.time() - start_time) * 1000,
                    "details": {}
                }
            
            # Validate metadata for each sample chunk
            metadata_validation_results = []
            for chunk in sample_chunks:
                validation_result = self._validate_chunk_metadata(chunk)
                metadata_validation_results.append(validation_result)
            
            # Calculate accuracy
            valid_count = sum(1 for v in metadata_validation_results if v["valid"])
            total_count = len(metadata_validation_results)
            accuracy = valid_count / total_count if total_count > 0 else 0.0
            
            # Check if expected chapters are properly identified
            found_chapters = set()
            for chunk in sample_chunks:
                chapter = chunk.get("metadata", {}).get("chapter")
                if chapter:
                    found_chapters.add(chapter)
            
            chapters_valid = True
            if expected_chapters:
                expected_set = set(expected_chapters)
                found_set = set(found_chapters)
                chapters_valid = expected_set.issubset(found_set)
            
            latency_ms = (time.time() - start_time) * 1000
            
            return {
                "success": accuracy > 0.9 and chapters_valid,  # High standard for module validation
                "module": module_name,
                "metadata_accuracy": accuracy,
                "valid_metadata_count": valid_count,
                "total_chunks_sampled": total_count,
                "expected_chapters": expected_chapters,
                "found_chapters": list(found_chapters),
                "chapters_validation_passed": chapters_valid,
                "latency_ms": latency_ms,
                "details": {
                    "expected_vs_found_chapters": {
                        "expected": expected_chapters,
                        "found": list(found_chapters),
                        "missing": list(expected_set - found_set) if expected_chapters else []
                    }
                }
            }
            
        except Exception as e:
            latency_ms = (time.time() - start_time) * 1000
            self.logger.error(f"Error validating metadata for module '{module_name}': {str(e)}")
            return {
                "success": False,
                "module": module_name,
                "error_message": str(e),
                "metadata_accuracy": 0.0,
                "valid_metadata_count": 0,
                "total_chunks_sampled": 0,
                "expected_chapters": expected_chapters,
                "found_chapters": [],
                "chapters_validation_passed": False,
                "latency_ms": latency_ms,
                "details": {"exception": str(e)}
            }
    
    def _validate_chunk_metadata(self, chunk: Dict) -> Dict[str, Any]:
        """
        Validate metadata for a single chunk.
        
        Args:
            chunk: The chunk to validate
            
        Returns:
            Dictionary containing validation result for this chunk.
        """
        metadata = chunk.get("metadata", {})
        
        # Check required fields
        required_fields = ["source_file", "chapter", "module"]
        missing_fields = [field for field in required_fields if not metadata.get(field)]
        
        # Check if module field contains expected pattern
        module = metadata.get("module", "")
        module_valid = "ROS 2" in module or "module" in module.lower()
        
        # Check if chapter field is appropriate
        chapter = metadata.get("chapter", "")
        chapter_valid = bool(chapter.strip())
        
        # Check if source file is valid
        source_file = metadata.get("source_file", "")
        source_valid = source_file.strip().endswith(('.md', '.txt', '.rst')) if source_file else False
        
        return {
            "id": chunk.get("id"),
            "valid": len(missing_fields) == 0 and module_valid and chapter_valid and source_valid,
            "missing_fields": missing_fields,
            "module_valid": module_valid,
            "chapter_valid": chapter_valid,
            "source_valid": source_valid,
            "metadata_values": {
                "module": module,
                "chapter": chapter,
                "source_file": source_file
            }
        }
    
    def _check_query_specific_metadata(self, query: str, retrieved_chunks: List[Dict]) -> bool:
        """
        Check if the metadata is appropriate for the specific query.
        
        Args:
            query: The original query
            retrieved_chunks: List of retrieved chunks
            
        Returns:
            True if metadata is appropriate for the query, False otherwise.
        """
        # If query is about ROS 2 modules, check if retrieved content is from Module 1: ROS 2
        if "ROS 2" in query or "module 1" in query.lower():
            module_matching_chunks = sum(
                1 for chunk in retrieved_chunks 
                if "ROS 2" in chunk.get("metadata", {}).get("module", "")
            )
            # At least 50% of retrieved chunks should be from ROS 2 modules
            return module_matching_chunks / len(retrieved_chunks) >= 0.5 if retrieved_chunks else True
        
        # For other queries, just check that metadata exists and is valid
        valid_metadata_chunks = sum(
            1 for chunk in retrieved_chunks
            if self._validate_chunk_metadata(chunk)["valid"]
        )
        
        return valid_metadata_chunks / len(retrieved_chunks) >= 0.8 if retrieved_chunks else True


# Example usage
if __name__ == "__main__":
    # For standalone testing
    print("This module should be used as part of the RagTester")