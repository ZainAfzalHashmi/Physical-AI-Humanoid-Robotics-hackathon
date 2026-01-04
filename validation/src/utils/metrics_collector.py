"""
Metrics collector for RAG validation system.
Collects and aggregates performance and validation metrics.
"""
import time
import json
import csv
from datetime import datetime
from typing import Dict, Any, List
import logging
import os


class MetricsCollector:
    """
    Collects and aggregates metrics from the RAG validation process.
    Provides methods to report on validation results and performance.
    """
    
    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.validation_results = []
        self.performance_metrics = []
        self.start_time = time.time()
    
    def collect_validation_result(self, result: Dict[str, Any]):
        """
        Collect a validation result for aggregation.
        
        Args:
            result: The validation result to collect.
        """
        result_with_timestamp = {
            **result,
            "timestamp": datetime.now().isoformat()
        }
        self.validation_results.append(result_with_timestamp)
        self.logger.debug(f"Collected validation result for query: '{result.get('query', 'unknown')}'")
    
    def collect_performance_metric(self, metric: Dict[str, Any]):
        """
        Collect a performance metric for aggregation.
        
        Args:
            metric: The performance metric to collect.
        """
        metric_with_timestamp = {
            **metric,
            "timestamp": datetime.now().isoformat()
        }
        self.performance_metrics.append(metric_with_timestamp)
        self.logger.debug(f"Collected performance metric: {metric.get('type', 'unknown')}")
    
    def get_metrics(self) -> Dict[str, Any]:
        """
        Get aggregated metrics.
        
        Returns:
            Dictionary containing aggregated metrics.
        """
        total_time = time.time() - self.start_time
        
        # Calculate validation metrics
        total_validations = len(self.validation_results)
        successful_validations = sum(1 for r in self.validation_results if r.get('overall_success', False))
        success_rate = successful_validations / total_validations if total_validations > 0 else 0
        
        # Calculate average latency across all validations
        total_latency = sum(
            r.get('performance', {}).get('latency_ms', {}).get('avg', 0) 
            for r in self.validation_results 
            if r.get('performance', {}).get('latency_ms', {}).get('avg') is not None
        )
        avg_latency = total_latency / total_validations if total_validations > 0 else 0
        
        # Calculate performance metrics
        total_performance_tests = len(self.performance_metrics)
        avg_performance_latency = 0
        if self.performance_metrics:
            perf_latencies = [
                m.get('latency', 0) 
                for m in self.performance_metrics 
                if m.get('latency') is not None
            ]
            avg_performance_latency = sum(perf_latencies) / len(perf_latencies) if perf_latencies else 0
        
        metrics = {
            "summary": {
                "total_time_seconds": total_time,
                "start_time": datetime.fromtimestamp(self.start_time).isoformat(),
                "end_time": datetime.now().isoformat()
            },
            "validation_metrics": {
                "total_validations": total_validations,
                "successful_validations": successful_validations,
                "success_rate": success_rate,
                "average_latency_ms": avg_latency
            },
            "performance_metrics": {
                "total_performance_tests": total_performance_tests,
                "average_performance_latency_ms": avg_performance_latency
            },
            "details": {
                "validation_results_breakdown": self._get_validation_breakdown(),
                "performance_distribution": self._get_performance_distribution()
            }
        }
        
        return metrics
    
    def _get_validation_breakdown(self) -> Dict[str, Any]:
        """
        Get detailed breakdown of validation results.
        
        Returns:
            Dictionary with validation breakdown details.
        """
        if not self.validation_results:
            return {}
        
        # Count by success/failure
        successful = [r for r in self.validation_results if r.get('overall_success', False)]
        failed = [r for r in self.validation_results if not r.get('overall_success', False)]
        
        # Calculate average scores by type
        retrieval_scores = [
            r.get('retrieval', {}).get('score', 0) 
            for r in self.validation_results 
            if r.get('retrieval', {}).get('score') is not None
        ]
        avg_retrieval_score = sum(retrieval_scores) / len(retrieval_scores) if retrieval_scores else 0
        
        metadata_scores = [
            r.get('metadata', {}).get('metadata_accuracy', 0) 
            for r in self.validation_results 
            if r.get('metadata', {}).get('metadata_accuracy') is not None
        ]
        avg_metadata_score = sum(metadata_scores) / len(metadata_scores) if metadata_scores else 0
        
        performance_scores = [
            r.get('performance', {}).get('success_rate', 0) 
            for r in self.validation_results 
            if r.get('performance', {}).get('success_rate') is not None
        ]
        avg_performance_score = sum(performance_scores) / len(performance_scores) if performance_scores else 0
        
        return {
            "successful_validations": len(successful),
            "failed_validations": len(failed),
            "average_retrieval_score": avg_retrieval_score,
            "average_metadata_score": avg_metadata_score,
            "average_performance_score": avg_performance_score
        }
    
    def _get_performance_distribution(self) -> Dict[str, Any]:
        """
        Get distribution of performance metrics.
        
        Returns:
            Dictionary with performance distribution details.
        """
        if not self.performance_metrics:
            return {}
        
        latencies = [m.get('latency', 0) for m in self.performance_metrics if m.get('latency') is not None]
        if not latencies:
            return {}
        
        # Basic statistics
        min_latency = min(latencies)
        max_latency = max(latencies)
        avg_latency = sum(latencies) / len(latencies)
        
        # Count how many are within acceptable limits
        acceptable_threshold = 500  # ms
        within_threshold = sum(1 for l in latencies if l <= acceptable_threshold)
        within_threshold_rate = within_threshold / len(latencies) if latencies else 0
        
        return {
            "min_latency_ms": min_latency,
            "max_latency_ms": max_latency,
            "avg_latency_ms": avg_latency,
            "within_threshold_rate": within_threshold_rate,
            "acceptable_threshold_ms": acceptable_threshold
        }
    
    def save_validation_results(self, filepath: str = None) -> str:
        """
        Save validation results to a JSON file.
        
        Args:
            filepath: Path to save the file. If None, generates a default path.
            
        Returns:
            Path where the file was saved.
        """
        if filepath is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"validation_results_{timestamp}.json"
            filepath = os.path.join("validation", "reports", filename)
            os.makedirs(os.path.dirname(filepath), exist_ok=True)
        
        data = {
            "timestamp": datetime.now().isoformat(),
            "total_results": len(self.validation_results),
            "results": self.validation_results,
            "aggregated_metrics": self.get_metrics()
        }
        
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False, default=str)
        
        self.logger.info(f"Validation results saved to {filepath}")
        return filepath
    
    def save_performance_metrics(self, filepath: str = None) -> str:
        """
        Save performance metrics to a CSV file.
        
        Args:
            filepath: Path to save the file. If None, generates a default path.
            
        Returns:
            Path where the file was saved.
        """
        if filepath is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"performance_metrics_{timestamp}.csv"
            filepath = os.path.join("validation", "reports", filename)
            os.makedirs(os.path.dirname(filepath), exist_ok=True)
        
        if not self.performance_metrics:
            self.logger.warning("No performance metrics to save")
            return filepath
        
        # Write to CSV
        with open(filepath, 'w', newline='', encoding='utf-8') as csvfile:
            fieldnames = self._get_csv_fieldnames()
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            
            writer.writeheader()
            for metric in self.performance_metrics:
                # Flatten nested dictionaries for CSV
                flattened = self._flatten_dict_for_csv(metric)
                writer.writerow(flattened)
        
        self.logger.info(f"Performance metrics saved to {filepath}")
        return filepath
    
    def _get_csv_fieldnames(self) -> List[str]:
        """
        Get field names for CSV output based on first metric.
        """
        if not self.performance_metrics:
            return ["timestamp", "type", "value", "details"]
        
        fieldnames = set()
        for metric in self.performance_metrics[:5]:  # Check first 5 for efficiency
            for key, value in self._flatten_dict_for_csv(metric).items():
                fieldnames.add(key)
        
        return sorted(list(fieldnames))
    
    def _flatten_dict_for_csv(self, d: Dict[str, Any], parent_key: str = '', sep: str = '.') -> Dict[str, Any]:
        """
        Flatten a nested dictionary for CSV output.
        """
        items = []
        for k, v in d.items():
            new_key = f"{parent_key}{sep}{k}" if parent_key else k
            if isinstance(v, dict):
                items.extend(self._flatten_dict_for_csv(v, new_key, sep=sep).items())
            else:
                items.append((new_key, v))
        return dict(items)
    
    def generate_summary_report(self) -> str:
        """
        Generate a human-readable summary report.
        
        Returns:
            Formatted summary report as a string.
        """
        metrics = self.get_metrics()
        
        report = f"""
RAG Pipeline Validation Summary Report
======================================

Validation Period: {metrics['summary']['start_time']} to {metrics['summary']['end_time']}
Total Duration: {metrics['summary']['total_time_seconds']:.2f} seconds

VALIDATION METRICS:
- Total validations: {metrics['validation_metrics']['total_validations']}
- Successful validations: {metrics['validation_metrics']['successful_validations']}
- Success rate: {metrics['validation_metrics']['success_rate']:.2%}
- Average latency: {metrics['validation_metrics']['average_latency_ms']:.2f} ms

PERFORMANCE METRICS:
- Total performance tests: {metrics['performance_metrics']['total_performance_tests']}
- Average performance latency: {metrics['performance_metrics']['average_performance_latency_ms']:.2f} ms

DETAILED BREAKDOWN:
- Successful validations: {metrics['details']['validation_results_breakdown']['successful_validations']}
- Failed validations: {metrics['details']['validation_results_breakdown']['failed_validations']}
- Average retrieval score: {metrics['details']['validation_results_breakdown']['average_retrieval_score']:.2f}
- Average metadata score: {metrics['details']['validation_results_breakdown']['average_metadata_score']:.2f}
- Average performance score: {metrics['details']['validation_results_breakdown']['average_performance_score']:.2f}

PERFORMANCE DISTRIBUTION:
- Min latency: {metrics['details']['performance_distribution'].get('min_latency_ms', 'N/A')} ms
- Max latency: {metrics['details']['performance_distribution'].get('max_latency_ms', 'N/A')} ms
- Avg latency: {metrics['details']['performance_distribution'].get('avg_latency_ms', 'N/A')} ms
- Within threshold rate: {metrics['details']['performance_distribution'].get('within_threshold_rate', 'N/A'):.2%}
        """
        
        return report.strip()


# Example usage
if __name__ == "__main__":
    # Example of how to use the metrics collector
    collector = MetricsCollector()
    
    # Simulate adding some validation results
    test_result = {
        "query": "What is rclpy?",
        "overall_success": True,
        "retrieval": {"success": True, "score": 0.92},
        "metadata": {"success": True, "metadata_accuracy": 0.95},
        "performance": {"success": True, "success_rate": 0.98}
    }
    
    collector.collect_validation_result(test_result)
    
    # Get and print metrics
    metrics = collector.get_metrics()
    print("Collected Metrics:")
    print(json.dumps(metrics, indent=2))
    
    # Generate and print summary
    summary = collector.generate_summary_report()
    print("\nSummary Report:")
    print(summary)