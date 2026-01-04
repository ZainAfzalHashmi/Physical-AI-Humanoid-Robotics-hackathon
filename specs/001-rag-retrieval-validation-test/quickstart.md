# Quickstart Guide: RAG Pipeline Retrieval & Validation Test

## Prerequisites

- Python 3.11+
- Access to Qdrant instance (with ROS 2 book content already embedded)
- Access to the Cohere API key (for potential additional validation)
- Git for version control

## Environment Setup

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. Set up the validation environment:
   ```bash
   cd validation
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   pip install -r requirements.txt
   ```

3. Set up environment variables:
   ```bash
   cp .env.example .env
   # Update .env with your Qdrant connection details and API keys
   ```

## Configuration

1. Update `validation/.env` with your Qdrant configuration:
   ```
   QDRANT_HOST=your-qdrant-host
   QDRANT_PORT=6333
   QDRANT_API_KEY=your-qdrant-api-key
   QDRANT_COLLECTION_NAME=book_content_chunks
   COHERE_API_KEY=your-cohere-api-key
   ```

2. Customize test queries in `validation/tests/test_data/queries.json`:
   ```json
   {
     "queries": [
       {
         "term": "rclpy",
         "category": "technical term",
         "expected_entities": ["rclpy", "nodes", "callbacks"]
       },
       {
         "term": "ROS 2 nodes",
         "category": "concept",
         "expected_entities": ["nodes", "rclpy", "publisher", "subscriber"]
       }
     ]
   }
   ```

## Running Validation Tests

### Run Complete Validation Suite

1. Execute all validation tests:
   ```bash
   cd validation
   python -m pytest tests/ -v
   ```

### Run Specific Validation Tests

1. Run only retrieval validation:
   ```bash
   python -m pytest tests/test_retrieval_validator.py -v
   ```

2. Run only metadata validation:
   ```bash
   python -m pytest tests/test_metadata_validator.py -v
   ```

3. Run only performance validation:
   ```bash
   python -m pytest tests/test_performance_validator.py -v
   ```

### Run Validation with Custom Configuration

1. Run validation with a specific configuration file:
   ```bash
   python src/rag_tester.py --config-path config/custom_config.json
   ```

## Executing Individual Validation Modules

### Direct Execution of Validation Components

1. Test retrieval functionality directly:
   ```bash
   python src/retrieval_validator.py --query "rclpy"
   ```

2. Test performance validation directly:
   ```bash
   python src/performance_validator.py --iterations 100
   ```

3. Validate metadata integrity:
   ```bash
   python src/metadata_validator.py --module "Module 1: ROS 2"
   ```

## Understanding Validation Results

The validation outputs results in multiple formats:

1. **Console Output**: Shows basic pass/fail status for each test
2. **JSON Report**: Detailed results saved to `validation/reports/validation_results.json`
3. **Performance Metrics**: Latency measurements saved to `validation/reports/performance_metrics.csv`

Example of successful validation result:
```
Validation Result:
- Query: "rclpy"
- Success: True
- Score: 0.94
- Retrieved Chunks: 5
- Expected Chunks: 3
- Latency: 245ms
- Expected Max Latency: 500ms
- Metadata Accuracy: 1.0
- Content Relevance: 0.92
```

## Integration with CI/CD

To integrate validation tests into your CI/CD pipeline:

1. Add validation step to your pipeline configuration:
   ```yaml
   - name: Run RAG Validation Tests
     run: |
       cd validation
       python -m pytest tests/ -v --junit-xml=report.xml
   ```

2. Set up validation to run before deploying any changes to the RAG pipeline.

## Troubleshooting

1. **Connection Issues**: Ensure your Qdrant instance is accessible and credentials are correct
2. **No Results Found**: Verify that the Qdrant collection contains the expected content embeddings
3. **Performance Issues**: Check that the Qdrant instance has adequate resources for the expected load
4. **Validation Failures**: Review the validation logs in `validation/logs/` for detailed error information

## Architecture Overview

The validation system consists of:

1. **Main Orchestrator** (`rag_tester.py`): Coordinates all validation activities
2. **Retrieval Validator** (`retrieval_validator.py`): Tests content retrieval functionality
3. **Metadata Validator** (`metadata_validator.py`): Verifies metadata integrity
4. **Performance Validator** (`performance_validator.py`): Ensures latency requirements are met
5. **Utilities** (`utils/`): Shared components like Qdrant client and metrics collection