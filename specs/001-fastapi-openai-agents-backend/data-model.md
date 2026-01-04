# Data Model: FastAPI Backend with OpenAI Agents SDK

## Entities

### Conversation Thread
**Description**: A collection of messages between a user and the chatbot, including metadata for tracking and persistence

**Fields**:
- `id` (UUID): Unique identifier for the conversation thread
- `user_id` (string): Identifier for the user associated with this conversation (nullable for anonymous users)
- `title` (string): Automatically generated title for the conversation
- `created_at` (datetime): Timestamp when the conversation was initiated
- `updated_at` (datetime): Timestamp of the last activity in the conversation
- `is_active` (boolean): Whether the conversation is currently active
- `metadata` (JSON): Additional information about the conversation context

**Relationships**:
- One-to-many with ChatMessage (conversation contains multiple messages)

### Chat Message
**Description**: A single message in a conversation, containing either a user query or AI response

**Fields**:
- `id` (UUID): Unique identifier for the message
- `conversation_id` (UUID): Reference to the parent conversation thread
- `sender_type` (enum): Either 'user' or 'assistant'
- `content` (text): The actual content of the message
- `timestamp` (datetime): When the message was created
- `message_type` (enum): 'query', 'response', or 'system' for different types of messages
- `context_chunks_used` (array[UUID]): References to the context chunks used to generate this response
- `response_confidence` (number): Confidence score for AI responses (0-1 scale)

**Relationships**:
- Many-to-one with ConversationThread (message belongs to one conversation)
- Many-to-many with ContextChunk (via context_chunks_used) showing which content was used in responses

### Context Chunk
**Description**: A segment of book content retrieved from Qdrant for grounding AI responses

**Fields**:
- `id` (UUID): Unique identifier for the context chunk
- `content` (text): The actual text content of the chunk
- `source_file` (string): Original file where this content originated from
- `section_title` (string): Section or chapter where this content belongs
- `embedding_vector` (array[float]): Vector representation used for similarity matching
- `relevance_score` (number): Score indicating relevance to a particular query (0-1 scale)
- `chunk_metadata` (JSON): Additional metadata like page number, position in document, etc.

**Relationships**:
- Many-to-many with ChatMessage (multiple messages may reference the same context chunks)
- Belongs to a single BookVolume (content originates from a specific book volume)

### Grounding Verification
**Description**: A validation record ensuring that AI responses are based only on book content

**Fields**:
- `id` (UUID): Unique identifier for the verification record
- `chat_message_id` (UUID): Reference to the message being verified
- `verification_result` (enum): 'passed', 'failed', or 'manual_review'
- `used_context_chunks` (array[UUID]): The context chunks that were used to validate the response
- `validation_score` (number): Numerical score indicating how well the response matches the provided context
- `validation_details` (text): Explanation of the validation process and results
- `verified_at` (datetime): Timestamp when verification was performed

**Relationships**:
- One-to-one with ChatMessage (each message can have one grounding verification)
- Many-to-many with ContextChunk (verification may reference multiple chunks)

### Book Volume
**Description**: A collection of content that represents a book or similar document

**Fields**:
- `id` (UUID): Unique identifier for the book volume
- `title` (string): Title of the book
- `author` (string): Author(s) of the book
- `version` (string): Version of the book content
- `language` (string): Language code (e.g., 'en', 'es')
- `published_date` (date): Date when this version was published
- `metadata` (JSON): Additional book-specific metadata

**Relationships**:
- One-to-many with ContextChunk (one book contains many content chunks)
- One-to-many with Chapter (one book contains many chapters)

## State Transitions

### Conversation Thread States
- `created` → `active`: When first message is added to a conversation
- `active` → `inactive`: After period of inactivity (configurable timeout) or user action
- `inactive` → `active`: When user engages again
- `inactive` → `archived`: After extended inactivity or user action
- `archived` → `deleted`: When user deletes conversation or automatic cleanup occurs

### Chat Message States
- `created` → `processed`: When message is received and initial processing is complete
- `processed` → `responded`: When AI response is generated
- `responded` → `validated`: When grounding verification completes
- `validated` → `delivered`: When response is delivered to user

### Grounding Verification States
- `created` → `verifying`: When validation process begins
- `verifying` → `passed`: When response is confirmed to be based on book content
- `verifying` → `failed`: When response contains information not in book content
- `failed` → `manual_review`: When automatic validation is inconclusive

## Validation Rules

### Conversation Thread
- User_id must be a valid identifier if provided
- Title must be between 5 and 100 characters
- Created_at must be before updated_at if both are set
- Is_active can only be true if conversation has at least one message

### Chat Message
- Content must be between 1 and 10,000 characters
- Sender_type must be either 'user' or 'assistant'
- Conversation_id must reference an existing conversation thread
- Context_chunks_used must reference valid context chunks if provided
- Response_confidence must be between 0 and 1 if provided

### Context Chunk
- Content must be between 50 and 5,000 characters
- Source_file must be a valid path
- Embedding_vector must have consistent dimensions (1024 for Cohere embeddings)
- Relevance_score must be between 0 and 1
- Section_title must not be empty

### Grounding Verification
- Chat_message_id must reference an existing chat message
- Verification_result must be one of the allowed enum values
- Validation_score must be between 0 and 1
- Validation_details should be present if verification_result is 'failed'