# Data Model: RAG Chatbot Integration for ROS 2 Book Project

## Entities

### Book Content Chunk
**Description**: Represents a segment of book content extracted from Markdown files

**Fields**:
- `id` (string): Unique identifier for the chunk
- `content` (string): The raw text of the content chunk
- `source_file` (string): Path to the original Markdown file
- `section_title` (string): Title of the section this chunk belongs to
- `page_reference` (string): Page/chapter reference in the book
- `embedding` (array of numbers): Vector representation of the content
- `created_at` (datetime): Timestamp when chunk was created
- `updated_at` (datetime): Timestamp of last update

**Relationships**:
- One-to-many with ChatMessage (through semantic similarity queries)
- Belongs to a single Book

### Chat Session
**Description**: Represents a conversation session between a user and the AI

**Fields**:
- `id` (string): Unique identifier for the session
- `user_id` (string): Identifier for the user (optional, for anonymous usage)
- `title` (string): Generated title for the conversation
- `created_at` (datetime): Timestamp when session started
- `updated_at` (datetime): Timestamp of last interaction
- `is_active` (boolean): Whether the session is currently active

**Relationships**:
- One-to-many with ChatMessage

### Chat Message
**Description**: Represents a single message in a conversation

**Fields**:
- `id` (string): Unique identifier for the message
- `session_id` (string): Reference to the parent ChatSession
- `role` (string): Either "user" or "assistant"
- `content` (string): The text content of the message
- `timestamp` (datetime): When the message was created
- `relevant_chunks` (array of strings): IDs of BookContentChunk that were referenced

**Relationships**:
- Belongs to a single ChatSession
- Many-to-many with BookContentChunk (through relevant_chunks)

## State Transitions

### Chat Session States
- `created` → `active`: When first message is added
- `active` → `archived`: After period of inactivity or user action
- `archived` → `deleted`: When explicitly removed by user

## Validation Rules

### Book Content Chunk
- Content must be between 100 and 2000 characters
- Source file must exist in the book repository
- Embedding must be a valid vector with consistent dimensions

### Chat Session
- Title must be between 5 and 100 characters
- Cannot have more than 1000 messages
- Must be active for at least one message to exist

### Chat Message
- Content must be between 1 and 10000 characters
- Role must be either "user" or "assistant"
- Timestamp must be within the session's active period