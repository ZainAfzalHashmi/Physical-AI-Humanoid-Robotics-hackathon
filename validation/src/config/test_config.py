"""
Configuration management for the RAG validation system.
"""
import os
import json
from typing import Dict, Any, List


class TestConfig:
    """
    Configuration class for the RAG validation system.
    Handles loading and accessing configuration parameters.
    """
    
    def __init__(self, config_path: str = None):
        # Default configuration
        self.qdrant_host = os.getenv("QDRANT_HOST", "localhost")
        self.qdrant_port = int(os.getenv("QDRANT_PORT", 6333))
        self.qdrant_api_key = os.getenv("QDRANT_API_KEY")
        self.qdrant_collection_name = os.getenv("QDRANT_COLLECTION_NAME", "book_content_chunks")
        self.max_latency_threshold_ms = float(os.getenv("MAX_LATENCY_THRESHOLD_MS", 500.0))
        self.min_similarity_score = float(os.getenv("MIN_SIMILARITY_SCORE", 0.7))
        self.top_k_results = int(os.getenv("TOP_K_RESULTS", 5))
        self.test_queries = [
            "rclpy",
            "ROS 2 nodes", 
            "publisher subscriber pattern",
            "ROS 2 services",
            "actionlib in ROS 2"
        ]
        self.expected_entities = {
            "rclpy": ["rclpy", "nodes", "callbacks"],
            "ROS 2 nodes": ["nodes", "rclpy", "publisher", "subscriber"],
            "publisher subscriber pattern": ["publisher", "subscriber", "topics", "messages"],
            "ROS 2 services": ["services", "client", "server"],
            "actionlib in ROS 2": ["actions", "action client", "action server"]
        }
        
        # Load configuration from file if provided
        if config_path and os.path.exists(config_path):
            self.load_from_file(config_path)
    
    def load_from_file(self, config_path: str):
        """
        Load configuration from a JSON file.
        
        Args:
            config_path: Path to the configuration file.
        """
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                config_data = json.load(f)
            
            # Update configuration values from file
            self.qdrant_host = config_data.get("qdrant_host", self.qdrant_host)
            self.qdrant_port = config_data.get("qdrant_port", self.qdrant_port)
            self.qdrant_api_key = config_data.get("qdrant_api_key", self.qdrant_api_key)
            self.qdrant_collection_name = config_data.get("qdrant_collection_name", self.qdrant_collection_name)
            self.max_latency_threshold_ms = config_data.get("max_latency_threshold_ms", self.max_latency_threshold_ms)
            self.min_similarity_score = config_data.get("min_similarity_score", self.min_similarity_score)
            self.top_k_results = config_data.get("top_k_results", self.top_k_results)
            self.test_queries = config_data.get("test_queries", self.test_queries)
            self.expected_entities = config_data.get("expected_entities", self.expected_entities)
            
            print(f"Configuration loaded from {config_path}")
        except Exception as e:
            print(f"Error loading configuration from {config_path}: {str(e)}")
            print("Using default configuration values")
    
    def save_to_file(self, config_path: str):
        """
        Save current configuration to a JSON file.
        
        Args:
            config_path: Path to save the configuration file.
        """
        config_data = self.to_dict()
        
        try:
            # Create directory if it doesn't exist
            os.makedirs(os.path.dirname(config_path), exist_ok=True)
            
            with open(config_path, 'w', encoding='utf-8') as f:
                json.dump(config_data, f, indent=2)
            
            print(f"Configuration saved to {config_path}")
        except Exception as e:
            print(f"Error saving configuration to {config_path}: {str(e)}")
    
    def to_dict(self) -> Dict[str, Any]:
        """
        Convert configuration to a dictionary.
        
        Returns:
            Dictionary representation of the configuration.
        """
        return {
            "qdrant_host": self.qdrant_host,
            "qdrant_port": self.qdrant_port,
            "qdrant_api_key": self.qdrant_api_key,
            "qdrant_collection_name": self.qdrant_collection_name,
            "max_latency_threshold_ms": self.max_latency_threshold_ms,
            "min_similarity_score": self.min_similarity_score,
            "top_k_results": self.top_k_results,
            "test_queries": self.test_queries,
            "expected_entities": self.expected_entities
        }


# Example usage
if __name__ == "__main__":
    # Create and display configuration
    config = TestConfig()
    
    print("Current Configuration:")
    print(json.dumps(config.to_dict(), indent=2))
    
    # Example: Save to file
    config.save_to_file("config/default_config.json")