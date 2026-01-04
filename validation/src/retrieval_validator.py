"""
Validator for RAG retrieval functionality.
Tests the ability to fetch relevant text chunks from Qdrant based on natural language queries.
"""
import time
import logging
from typing import List, Dict, Any, Optional
from src.utils.qdrant_client import QdrantClientWrapper
from src.config.test_config import TestConfig
from src.utils.metrics_collector import MetricsCollector


class RetrievalValidator:
    """
    Validates the retrieval functionality of the RAG pipeline.
    Ensures that relevant text chunks are retrieved from Qdrant based on natural language queries.
    """
    
    def __init__(self, qdrant_client: QdrantClientWrapper, config: TestConfig):
        self.qdrant_client = qdrant_client
        self.config = config
        self.logger = logging.getLogger(__name__)
        self.metrics_collector = MetricsCollector()
    
    def validate_query(self, query: str) -> Dict[str, Any]:
        """
        Validate retrieval for a specific query.
        
        Args:
            query: The natural language query to test
            
        Returns:
            Dictionary containing validation results.
        """
        start_time = time.time()
        
        try:
            # Generate embedding for the query
            query_embedding = self.qdrant_client.generate_embedding(query)
            
            if query_embedding is None:
                return {
                    "success": False,
                    "error_message": "Failed to generate embedding for query",
                    "query": query,
                    "retrieved_chunks_count": 0,
                    "expected_chunks_count": self.config.top_k_results,
                    "latency_ms": (time.time() - start_time) * 1000,
                    "score": 0.0,
                    "details": {}
                }
            
            # Retrieve similar content from Qdrant
            retrieved_chunks = self.qdrant_client.search_similar(
                query_embedding, 
                limit=self.config.top_k_results
            )
            
            latency_ms = (time.time() - start_time) * 1000
            
            # Evaluate retrieval quality
            evaluation = self._evaluate_retrieval(query, retrieved_chunks)
            
            return {
                "success": evaluation["success"],
                "query": query,
                "retrieved_chunks_count": len(retrieved_chunks),
                "expected_chunks_count": self.config.top_k_results,
                "latency_ms": latency_ms,
                "score": evaluation["score"],
                "retrieved_chunks": retrieved_chunks,
                "content_relevance_score": evaluation["relevance_score"],
                "details": evaluation["details"],
                "error_message": None
            }
            
        except Exception as e:
            latency_ms = (time.time() - start_time) * 1000
            self.logger.error(f"Error validating retrieval for query '{query}': {str(e)}")
            return {
                "success": False,
                "error_message": str(e),
                "query": query,
                "retrieved_chunks_count": 0,
                "expected_chunks_count": self.config.top_k_results,
                "latency_ms": latency_ms,
                "score": 0.0,
                "retrieved_chunks": [],
                "content_relevance_score": 0.0,
                "details": {"exception": str(e)}
            }
    
    def validate_specific_ros2_terms(self, terms: List[str] = None) -> Dict[str, Any]:
        """
        Validate retrieval specifically for ROS 2 technical terms.
        
        Args:
            terms: List of ROS 2 terms to validate (default: self.config.expected_entities)
            
        Returns:
            Dictionary containing validation results for ROS 2 terms.
        """
        if terms is None:
            terms = self.config.expected_entities.get("ros2_terms", ["rclpy", "nodes"])
        
        results = {
            "tested_terms": terms,
            "individual_results": [],
            "summary": {
                "total_terms": len(terms),
                "successful_retrievals": 0,
                "average_score": 0.0
            }
        }
        
        total_score = 0.0
        
        for term in terms:
            result = self.validate_query(term)
            results["individual_results"].append(result)
            
            if result["success"]:
                results["summary"]["successful_retrievals"] += 1
                total_score += result["score"]
        
        if results["summary"]["successful_retrievals"] > 0:
            results["summary"]["average_score"] = (
                total_score / results["summary"]["successful_retrievals"]
            )
        
        return results
    
    def _evaluate_retrieval(self, query: str, retrieved_chunks: List[Dict]) -> Dict[str, Any]:
        """
        Evaluate the quality of retrieved content for a query.
        
        Args:
            query: The original query
            retrieved_chunks: List of retrieved content chunks
            
        Returns:
            Dictionary containing evaluation results.
        """
        if not retrieved_chunks:
            return {
                "success": False,
                "score": 0.0,
                "relevance_score": 0.0,
                "details": {"message": "No content retrieved for query"}
            }
        
        # Check if at least some content was retrieved
        success = len(retrieved_chunks) > 0
        
        # Calculate content relevance score based on query terms
        relevance_score = self._calculate_relevance_score(query, retrieved_chunks)
        
        # Calculate overall score based on relevance and number of results
        score = relevance_score  # For now, just use relevance as the score
        
        # Check if retrieved content matches expected entities for this query
        expected_entities = self.config.expected_entities.get(query.lower(), [])
        if expected_entities:
            entity_match_score = self._check_entity_matches(expected_entities, retrieved_chunks)
            # Adjust score based on entity matches
            score = (score + entity_match_score) / 2
        
        return {
            "success": success and relevance_score > 0.3,  # Consider successful if relevance > 0.3
            "score": score,
            "relevance_score": relevance_score,
            "details": {
                "retrieved_count": len(retrieved_chunks),
                "expected_entities": expected_entities,
                "entity_match_score": entity_match_score if expected_entities else None
            }
        }
    
    def _calculate_relevance_score(self, query: str, retrieved_chunks: List[Dict]) -> float:
        """
        Calculate a relevance score based on how well the retrieved content matches the query.
        
        Args:
            query: The original query
            retrieved_chunks: List of retrieved content chunks
            
        Returns:
            Relevance score between 0 and 1.
        """
        if not retrieved_chunks:
            return 0.0
        
        query_lower = query.lower()
        total_relevance = 0.0
        
        for chunk in retrieved_chunks:
            content = chunk.get("content", "").lower()
            # Simple relevance calculation based on query term presence
            query_terms = query_lower.split()
            matching_terms = sum(1 for term in query_terms if term in content)
            relevance = matching_terms / len(query_terms) if query_terms else 0.0
            total_relevance += relevance
        
        avg_relevance = total_relevance / len(retrieved_chunks)
        
        # Boost score based on semantic similarity from Qdrant scores if available
        if "score" in retrieved_chunks[0]:
            qdrant_scores = [c.get("score", 0.0) for c in retrieved_chunks]
            avg_qdrant_score = sum(qdrant_scores) / len(qdrant_scores)
            # Combine content-based and Qdrant-based scores
            combined_score = (avg_relevance + avg_qdrant_score) / 2
            return min(1.0, combined_score)  # Ensure score doesn't exceed 1.0
        
        return avg_relevance
    
    def _check_entity_matches(self, expected_entities: List[str], retrieved_chunks: List[Dict]) -> float:
        """
        Check how many expected entities are found in retrieved content.
        
        Args:
            expected_entities: List of expected entities
            retrieved_chunks: List of retrieved content chunks
            
        Returns:
            Entity match score between 0 and 1.
        """
        if not expected_entities:
            return 1.0  # No entities to match, so consider it a perfect match
        
        all_content = " ".join([chunk.get("content", "") for chunk in retrieved_chunks]).lower()
        
        matched_entities = 0
        for entity in expected_entities:
            if entity.lower() in all_content:
                matched_entities += 1
        
        return matched_entities / len(expected_entities)


# Example usage
if __name__ == "__main__":
    # For standalone testing
    print("This module should be used as part of the RagTester")