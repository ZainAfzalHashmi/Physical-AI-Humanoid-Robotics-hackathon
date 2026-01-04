# Data Model: RAG Pipeline Retrieval & Validation Test

## Entities

### RetrievedContentChunk
**Description**: Represents a segment of text from the ROS 2 book that matches a query, including the raw text content and associated metadata

**Fields**:
- `id` (string): Unique identifier for the content chunk in Qdrant
- `content` (string): The raw text content of the chunk
- `source_file` (string): Path to the original source file in the book structure
- `chapter` (string): The chapter identifier to which this chunk belongs
- `module` (string): The module identifier (e.g., "Module 1: ROS 2")
- `embedding` (list of numbers): Vector representation of the content for semantic search
- `created_at` (datetime): Timestamp when chunk was created
- `updated_at` (datetime): Timestamp of last update

**Relationships**:
- Belongs to a single Module
- Belongs to a single Chapter within that module

### QueryTerm
**Description**: A natural language search phrase or question that is used to find relevant content in the RAG system

**Fields**:
- `id` (string): Unique identifier for the query term
- `term` (string): The actual natural language query text
- `category` (string): The category of the query (e.g., "technical term", "concept", "example")
- `expected_entities` (list of strings): List of entities expected to be in the response (e.g., ["rclpy", "nodes"])
- `created_at` (datetime): Timestamp when query was created
- `updated_at` (datetime): Timestamp of last update

**Relationships**:
- Produces zero or more RetrievedContentChunks through retrieval
- Belongs to a TestScenario

### ValidationResult
**Description**: An output that indicates the success or failure of a specific test case in the validation process

**Fields**:
- `id` (string): Unique identifier for the validation result
- `test_type` (string): The type of validation (e.g., "retrieval", "metadata", "performance")
- `query_term` (string): The query that was tested
- `success` (boolean): Whether the validation was successful
- `score` (number): A numerical score representing the quality of the result (0-1)
- `retrieved_chunks_count` (integer): Number of chunks retrieved
- `expected_chunks_count` (integer): Expected number of chunks to retrieve
- `latency_ms` (number): Time taken to execute the retrieval in milliseconds
- `expected_latency_ms` (number): Expected maximum latency (500ms)
- `metadata_accuracy` (number): Accuracy score for metadata validation (0-1)
- `content_relevance_score` (number): Relevance score of retrieved content (0-1)
- `details` (object): Additional details about the validation
- `error_message` (string): Error message if validation failed
- `timestamp` (datetime): When the validation was performed

**Relationships**:
- Generated from a QueryTerm
- Associated with zero or more RetrievedContentChunks
- Part of a TestRun

### LatencyMeasurement
**Description**: A timing metric that captures the duration from query submission to result retrieval completion

**Fields**:
- `id` (string): Unique identifier for the measurement
- `query_term` (string): The query that was measured
- `start_time` (datetime): When the query was submitted
- `end_time` (datetime): When the result was received
- `duration_ms` (number): The total duration in milliseconds
- `timestamp` (datetime): When the measurement was taken

**Relationships**:
- Associated with a QueryTerm
- Part of a ValidationResult

### TestConfiguration
**Description**: Configuration parameters for the validation process

**Fields**:
- `id` (string): Unique identifier for the configuration
- `qdrant_host` (string): Host address for Qdrant connection
- `qdrant_port` (integer): Port number for Qdrant connection
- `qdrant_collection_name` (string): Name of the collection to query
- `max_latency_threshold_ms` (number): Maximum acceptable latency (500ms)
- `min_similarity_score` (number): Minimum acceptable similarity score (0.7)
- `top_k_results` (integer): Number of top results to retrieve (5)
- `test_queries` (list of strings): List of queries to test
- `expected_entities` (object): Mapping of queries to expected entities
- `updated_at` (datetime): Timestamp of last configuration update

## State Transitions

### ValidationResult States
- `created` → `in_progress`: When validation starts
- `in_progress` → `completed`: When validation completes successfully
- `in_progress` → `failed`: When validation encounters an error
- `completed` → `reviewed`: When result is reviewed by QA engineer

## Validation Rules

### RetrievedContentChunk
- Content must be between 50 and 10000 characters
- Source file must be a valid path in the book structure
- Module and chapter fields must not be empty
- Embedding must be a valid vector with consistent dimensions (1024 for Cohere embeddings)

### QueryTerm
- Term must be between 1 and 500 characters
- Category must be one of: "technical term", "concept", "example", "question"
- Expected entities should not exceed 10 items

### ValidationResult
- Latency must be a positive number
- Success flag must match the comparison between latency and threshold
- Score must be between 0 and 1
- If success is false, error_message should not be empty

### TestConfiguration
- Qdrant host must be a valid URL or IP address
- Port must be between 1 and 65535
- Max latency threshold must be positive
- Min similarity score must be between 0 and 1