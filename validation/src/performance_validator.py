"""
Validator for RAG pipeline performance.
Tests that pipeline latency is within acceptable limits for a chatbot.
"""
import time
import statistics
import logging
from typing import List, Dict, Any
from src.utils.qdrant_client import QdrantClientWrapper
from src.config.test_config import TestConfig


class PerformanceValidator:
    """
    Validates the performance of the RAG pipeline.
    Ensures that pipeline latency is within acceptable limits for a chatbot.
    """
    
    def __init__(self, qdrant_client: QdrantClientWrapper, config: TestConfig):
        self.qdrant_client = qdrant_client
        self.config = config
        self.logger = logging.getLogger(__name__)
    
    def validate_query_performance(self, query: str, iterations: int = 10) -> Dict[str, Any]:
        """
        Validate performance for a specific query over multiple iterations.
        
        Args:
            query: The query to test
            iterations: Number of iterations to run (default: 10)
            
        Returns:
            Dictionary containing performance validation results.
        """
        start_time = time.time()
        
        latencies = []
        errors = []
        
        try:
            # Run the query multiple times to get performance metrics
            for i in range(iterations):
                iteration_start = time.perf_counter()
                
                try:
                    # Generate embedding and retrieve similar content
                    query_embedding = self.qdrant_client.generate_embedding(query)
                    
                    if query_embedding is not None:
                        # Perform the retrieval
                        retrieved_chunks = self.qdrant_client.search_similar(
                            query_embedding, 
                            limit=self.config.top_k_results
                        )
                    
                    # Record the latency for this iteration
                    iteration_latency = (time.perf_counter() - iteration_start) * 1000  # Convert to milliseconds
                    latencies.append(iteration_latency)
                    
                except Exception as e:
                    error_time = (time.perf_counter() - iteration_start) * 1000
                    latencies.append(error_time)  # Include error time in metrics
                    errors.append(str(e))
                    self.logger.error(f"Error in iteration {i+1} for query '{query}': {str(e)}")
            
            if not latencies:
                return {
                    "success": False,
                    "error_message": "All iterations failed",
                    "query": query,
                    "iterations": iterations,
                    "latency_ms": {"avg": 0, "min": 0, "max": 0, "p95": 0},
                    "success_rate": 0.0,
                    "error_count": iterations,
                    "errors": errors
                }
            
            # Calculate performance metrics
            avg_latency = statistics.mean(latencies)
            min_latency = min(latencies)
            max_latency = max(latencies)
            p95_latency = self._calculate_percentile(latencies, 95) if latencies else 0
            
            # Calculate success rate (queries completed without error)
            success_rate = (iterations - len(errors)) / iterations if iterations > 0 else 0
            
            # Overall success based on latency and success rate thresholds
            latency_acceptable = avg_latency <= self.config.max_latency_threshold_ms
            success_rate_acceptable = success_rate >= 0.95  # 95% success rate
            
            overall_success = latency_acceptable and success_rate_acceptable
            
            latency_metrics = {
                "avg": avg_latency,
                "min": min_latency,
                "max": max_latency,
                "p95": p95_latency,
                "threshold": self.config.max_latency_threshold_ms
            }
            
            return {
                "success": overall_success,
                "query": query,
                "iterations": iterations,
                "latency_ms": latency_metrics,
                "success_rate": success_rate,
                "error_count": len(errors),
                "errors": errors if errors else None,
                "latency_breakdown": latencies,
                "details": {
                    "latency_acceptable": latency_acceptable,
                    "success_rate_acceptable": success_rate_acceptable,
                    "expected_max_latency": self.config.max_latency_threshold_ms
                }
            }
            
        except Exception as e:
            self.logger.error(f"Error validating performance for query '{query}': {str(e)}")
            return {
                "success": False,
                "error_message": str(e),
                "query": query,
                "iterations": iterations,
                "latency_ms": {"avg": 0, "min": 0, "max": 0, "p95": 0},
                "success_rate": 0.0,
                "error_count": iterations,
                "errors": [str(e)]
            }
    
    def validate_all_queries_performance(self, queries: List[str], iterations: int = 10) -> Dict[str, Any]:
        """
        Validate performance for all queries.
        
        Args:
            queries: List of queries to test
            iterations: Number of iterations per query
            
        Returns:
            Dictionary containing overall performance validation results.
        """
        start_time = time.time()
        
        all_results = []
        query_success_count = 0
        
        for query in queries:
            result = self.validate_query_performance(query, iterations)
            all_results.append(result)
            
            if result["success"]:
                query_success_count += 1
        
        # Calculate aggregate metrics
        all_latencies = []
        all_success_rates = []
        
        for result in all_results:
            if result["latency_ms"] and isinstance(result["latency_ms"], dict):
                all_latencies.append(result["latency_ms"]["avg"])
            all_success_rates.append(result["success_rate"])
        
        aggregate_avg_latency = statistics.mean(all_latencies) if all_latencies else 0
        aggregate_success_rate = statistics.mean(all_success_rates) if all_success_rates else 0
        
        # Overall success: 95% of queries must meet performance criteria
        overall_success_rate = query_success_count / len(queries) if queries else 0
        overall_success = overall_success_rate >= 0.95
        
        return {
            "success": overall_success,
            "total_queries": len(queries),
            "successful_queries": query_success_count,
            "overall_success_rate": overall_success_rate,
            "aggregate_metrics": {
                "avg_latency": aggregate_avg_latency,
                "avg_success_rate": aggregate_success_rate
            },
            "individual_results": all_results,
            "latency_threshold": self.config.max_latency_threshold_ms,
            "required_success_rate": 0.95,
            "validation_time": (time.time() - start_time) * 1000
        }
    
    def validate_concurrent_load(self, query: str, num_clients: int = 5, iterations_per_client: int = 5) -> Dict[str, Any]:
        """
        Validate performance under concurrent load.
        
        Args:
            query: The query to test
            num_clients: Number of concurrent clients
            iterations_per_client: Number of iterations per client
            
        Returns:
            Dictionary containing concurrent load validation results.
        """
        import concurrent.futures
        import threading
        
        start_time = time.time()
        
        def run_iterations(client_id: int):
            """Run iterations for a single client."""
            client_latencies = []
            client_errors = []
            
            for i in range(iterations_per_client):
                iteration_start = time.perf_counter()
                
                try:
                    # Generate embedding and retrieve similar content
                    query_embedding = self.qdrant_client.generate_embedding(query)
                    
                    if query_embedding is not None:
                        # Perform the retrieval
                        retrieved_chunks = self.qdrant_client.search_similar(
                            query_embedding, 
                            limit=self.config.top_k_results
                        )
                    
                    # Record the latency for this iteration
                    iteration_latency = (time.perf_counter() - iteration_start) * 1000  # Convert to milliseconds
                    client_latencies.append(iteration_latency)
                    
                except Exception as e:
                    error_time = (time.perf_counter() - iteration_start) * 1000
                    client_latencies.append(error_time)  # Include error time in metrics
                    client_errors.append(str(e))
                    self.logger.error(f"Error in client {client_id}, iteration {i+1}: {str(e)}")
            
            return {
                "client_id": client_id,
                "latencies": client_latencies,
                "errors": client_errors
            }
        
        # Execute all clients concurrently
        all_client_results = []
        
        with concurrent.futures.ThreadPoolExecutor(max_workers=num_clients) as executor:
            futures = [executor.submit(run_iterations, i) for i in range(num_clients)]
            all_client_results = [future.result() for future in futures]
        
        # Aggregate results
        all_latencies = []
        all_errors = []
        
        for client_result in all_client_results:
            all_latencies.extend(client_result["latencies"])
            all_errors.extend(client_result["errors"])
        
        if not all_latencies:
            return {
                "success": False,
                "error_message": "All concurrent requests failed",
                "query": query,
                "num_clients": num_clients,
                "iterations_per_client": iterations_per_client,
                "latency_ms": {"avg": 0, "min": 0, "max": 0, "p95": 0},
                "success_rate": 0.0,
                "error_count": len(all_errors),
                "errors": all_errors
            }
        
        # Calculate aggregate metrics
        avg_latency = statistics.mean(all_latencies)
        min_latency = min(all_latencies)
        max_latency = max(all_latencies)
        p95_latency = self._calculate_percentile(all_latencies, 95) if all_latencies else 0
        
        # Calculate success rate
        success_rate = (len(all_latencies) - len(all_errors)) / len(all_latencies) if all_latencies else 0
        
        # Overall success based on latency and success rate thresholds
        latency_acceptable = avg_latency <= self.config.max_latency_threshold_ms
        success_rate_acceptable = success_rate >= 0.90  # Slightly lower threshold for concurrent load
        
        overall_success = latency_acceptable and success_rate_acceptable
        
        latency_metrics = {
            "avg": avg_latency,
            "min": min_latency,
            "max": max_latency,
            "p95": p95_latency,
            "threshold": self.config.max_latency_threshold_ms
        }
        
        return {
            "success": overall_success,
            "query": query,
            "num_clients": num_clients,
            "iterations_per_client": iterations_per_client,
            "total_requests": num_clients * iterations_per_client,
            "latency_ms": latency_metrics,
            "success_rate": success_rate,
            "error_count": len(all_errors),
            "errors": all_errors if all_errors else None,
            "client_results": all_client_results,
            "details": {
                "latency_acceptable": latency_acceptable,
                "success_rate_acceptable": success_rate_acceptable,
                "expected_max_latency": self.config.max_latency_threshold_ms
            }
        }
    
    def _calculate_percentile(self, data: List[float], percentile: float) -> float:
        """
        Calculate the specified percentile of the data.
        
        Args:
            data: List of numeric values
            percentile: Percentile to calculate (e.g., 95 for p95)
            
        Returns:
            The calculated percentile value.
        """
        if not data:
            return 0.0
        
        sorted_data = sorted(data)
        index = (percentile / 100) * (len(sorted_data) - 1)
        
        if index.is_integer():
            return sorted_data[int(index)]
        else:
            lower_index = int(index)
            upper_index = lower_index + 1
            weight = index - lower_index
            return sorted_data[lower_index] * (1 - weight) + sorted_data[upper_index] * weight


# Example usage
if __name__ == "__main__":
    # For standalone testing
    print("This module should be used as part of the RagTester")