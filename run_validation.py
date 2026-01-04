#!/usr/bin/env python3
"""
Entry point for the RAG Pipeline Validation system.
This script allows running the validation from the command line.
"""

import argparse
import sys
import os

# Add the validation/src directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'validation', 'src'))

from rag_tester import RagTester


def main():
    parser = argparse.ArgumentParser(description="RAG Pipeline Validation Tool")
    
    parser.add_argument(
        "--config-path", 
        type=str, 
        help="Path to the configuration file"
    )
    
    parser.add_argument(
        "--queries", 
        nargs="+", 
        help="Specific queries to validate (if not provided, uses default queries from config)"
    )
    
    parser.add_argument(
        "--mode", 
        choices=["comprehensive", "retrieval", "metadata", "performance"], 
        default="comprehensive",
        help="Validation mode to run: comprehensive (all tests), retrieval (only retrieval), metadata (only metadata), or performance (only performance)"
    )
    
    parser.add_argument(
        "--iterations", 
        type=int, 
        default=10,
        help="Number of iterations for performance testing (default: 10)"
    )
    
    parser.add_argument(
        "--output-dir", 
        type=str, 
        default="validation/reports",
        help="Directory to save validation reports (default: validation/reports)"
    )
    
    args = parser.parse_args()
    
    try:
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
            results = tester.run_performance_only_validation(
                args.queries or tester.config.test_queries, 
                args.iterations
            )
        
        # Print results summary
        print("Validation Results Summary:")
        print("="*50)
        
        if "summary" in results:
            summary = results["summary"]
            print(f"Total queries tested: {summary.get('total_queries', 'N/A')}")
            print(f"Start time: {summary.get('start_time', 'N/A')}")
            print(f"Duration: {summary.get('duration', 'N/A'):.2f} seconds")
        
        if "validation_results" in results:
            successful = sum(1 for r in results["validation_results"] if r.get("overall_success", False))
            total = len(results["validation_results"])
            success_rate = successful / total if total > 0 else 0
            print(f"Overall success rate: {success_rate:.2%} ({successful}/{total})")
        
        # Save detailed results
        os.makedirs(args.output_dir, exist_ok=True)
        
        import json
        from datetime import datetime
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_file = os.path.join(args.output_dir, f"validation_results_{args.mode}_{timestamp}.json")
        
        with open(output_file, 'w', encoding='utf-8') as f:
            json.dump(results, f, indent=2, default=str)
        
        print(f"\nDetailed results saved to: {output_file}")
        
        # Generate and save summary report
        if hasattr(tester, 'metrics_collector'):
            report = tester.metrics_collector.generate_summary_report()
            report_file = os.path.join(args.output_dir, f"summary_report_{args.mode}_{timestamp}.txt")
            
            with open(report_file, 'w', encoding='utf-8') as f:
                f.write(report)
            
            print(f"Summary report saved to: {report_file}")
        
        # Exit with appropriate code
        if "validation_results" in results:
            successful = sum(1 for r in results["validation_results"] if r.get("overall_success", False))
            total = len(results["validation_results"])
            success_rate = successful / total if total > 0 else 0
            
            # Exit with error code if success rate is below 90%
            if success_rate < 0.9:
                print(f"\nWARNING: Success rate ({success_rate:.2%}) is below 90%")
                sys.exit(1)
        
        return 0
        
    except KeyboardInterrupt:
        print("\nValidation interrupted by user")
        sys.exit(130)  # Standard exit code for Ctrl+C
    except Exception as e:
        print(f"Error during validation: {str(e)}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    exit(main())