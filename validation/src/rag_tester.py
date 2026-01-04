"""
Main RAG validation orchestrator that coordinates all validation activities.
"""
import json
import time
import logging
from typing import List, Dict, Any
from src.retrieval_validator import RetrievalValidator
from src.metadata_validator import MetadataValidator
from src.performance_validator import PerformanceValidator
from src.config.test_config import TestConfig
from src.utils.qdrant_client import QdrantClientWrapper
from src.utils.metrics_collector import MetricsCollector


class RagTester:
    """
    Main orchestrator for RAG pipeline validation.
    Coordinates retrieval, metadata, and performance validation.
    """
    
    def __init__(self, config_path: str = None):
        # Initialize logger
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
        # Load configuration
        self.config = TestConfig(config_path)
        
        # Initialize Qdrant client
        self.qdrant_client = QdrantClientWrapper(
            host=self.config.qdrant_host,
            port=self.config.qdrant_port,
            api_key=self.config.qdrant_api_key,
            collection_name=self.config.qdrant_collection_name
        )
        
        # Initialize validators
        self.retrieval_validator = RetrievalValidator(self.qdrant_client, self.config)
        self.metadata_validator = MetadataValidator(self.qdrant_client, self.config)
        self.performance_validator = PerformanceValidator(self.qdrant_client, self.config)
        
        # Initialize metrics collector
        self.metrics_collector = MetricsCollector()
        
        self.logger.info("RAG Tester initialized successfully")
    
    def run_comprehensive_validation(self, queries: List[str] = None) -> Dict[str, Any]:
        """
        Run comprehensive validation of the RAG pipeline.
        
        Args:
            queries: List of queries to test. If None, uses default queries from config.
            
        Returns:
            Dictionary containing validation results.
        """
        start_time = time.time()
        
        if queries is None:
            queries = self.config.test_queries
        
        self.logger.info(f"Starting comprehensive validation for {len(queries)} queries")
        
        results = {
            "summary": {
                "total_queries": len(queries),
                "start_time": start_time,
                "test_config": self.config.to_dict()
            },
            "validation_results": [],
            "metrics": {}
        }
        
        for query in queries:
            self.logger.info(f"Validating query: '{query}'")
            
            # Run retrieval validation
            retrieval_result = self.retrieval_validator.validate_query(query)
            
            # Run metadata validation
            metadata_result = self.metadata_validator.validate_query_metadata(
                query, 
                retrieval_result.get("retrieved_chunks", [])
            )
            
            # Run performance validation
            performance_result = self.performance_validator.validate_query_performance(query)
            
            # Combine results
            combined_result = {
                "query": query,
                "retrieval": retrieval_result,
                "metadata": metadata_result,
                "performance": performance_result,
                "overall_success": (
                    retrieval_result.get("success", False) and
                    metadata_result.get("success", False) and
                    performance_result.get("success", False)
                )
            }
            
            results["validation_results"].append(combined_result)
            
            # Collect metrics
            self.metrics_collector.collect_validation_result(combined_result)
        
        # Finalize metrics
        results["metrics"] = self.metrics_collector.get_metrics()
        results["summary"]["end_time"] = time.time()
        results["summary"]["duration"] = results["summary"]["end_time"] - start_time
        
        self.logger.info(f"Comprehensive validation completed in {results['summary']['duration']:.2f}s")
        
        return results
    
    def run_retrieval_only_validation(self, queries: List[str]) -> Dict[str, Any]:
        """
        Run only retrieval validation.
        """
        results = []
        for query in queries:
            result = self.retrieval_validator.validate_query(query)
            results.append({
                "query": query,
                "result": result
            })
        return {"retrieval_results": results}
    
    def run_metadata_only_validation(self, queries: List[str]) -> Dict[str, Any]:
        """
        Run only metadata validation.
        """
        results = []
        for query in queries:
            # First retrieve content for metadata validation
            retrieval_result = self.retrieval_validator.validate_query(query)
            chunks = retrieval_result.get("retrieved_chunks", [])
            
            result = self.metadata_validator.validate_query_metadata(query, chunks)
            results.append({
                "query": query,
                "result": result
            })
        return {"metadata_results": results}
    
    def run_performance_only_validation(self, queries: List[str], iterations: int = 10) -> Dict[str, Any]:
        """
        Run only performance validation.
        """
        results = []
        for query in queries:
            result = self.performance_validator.validate_query_performance(query, iterations)
            results.append({
                "query": query,
                "result": result
            })
        return {"performance_results": results}


def main():
    """
    Main function to run the RAG validation from command line.
    """
    import argparse
    
    parser = argparse.ArgumentParser(description="RAG Pipeline Validation Tool")
    parser.add_argument("--config-path", type=str, help="Path to configuration file")
    parser.add_argument("--queries", nargs="+", help="Specific queries to validate")
    parser.add_argument("--mode", choices=["comprehensive", "retrieval", "metadata", "performance"], 
                       default="comprehensive", help="Validation mode to run")
    
    args = parser.parse_args()
    
    # Initialize the tester
    tester = RagTester(config_path=args.config_path)
    
    # Run validation based on mode
    if args.mode == "comprehensive":
        results = tester.run_comprehensive_validation(args.queries)
    elif args.mode == "retrieval":
        results = tester.run_retrieval_only_validation(args.queries or tester.config.test_queries)
    elif args.mode == "metadata":
        results = tester.run_metadata_only_validation(args.queries or tester.config.test_queries)
    elif args.mode == "performance":
        results = tester.run_performance_only_validation(args.queries or tester.config.test_queries)
    
    # Print results
    print(json.dumps(results, indent=2, default=str))


if __name__ == "__main__":
    main()