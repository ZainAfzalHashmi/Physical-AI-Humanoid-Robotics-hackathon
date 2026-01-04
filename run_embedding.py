"""
Script to run the embedding process for the ROS 2 book content
"""

import sys
import os
import asyncio

# Add the backend/src directory to the Python path
sys.path.insert(0, os.path.join(os.getcwd(), 'backend', 'src'))

from dotenv import load_dotenv
from services.rag import RAGService

# Load environment variables from .env file
load_dotenv()

def run_embedding_process():
    """
    Run the embedding process for the book content
    """
    print("Starting the embedding process for ROS 2 book content...")

    # Initialize the RAG service
    rag_service = RAGService()

    # Path to the book content in the repository
    book_content_path = os.path.join(os.getcwd(), "hackathon-ai-book", "docs")

    # If the hackathon-ai-book/docs doesn't exist, try the root directory
    if not os.path.exists(book_content_path):
        book_content_path = os.path.join(os.getcwd())

    print(f"Looking for book content in: {book_content_path}")

    # Check if the path exists
    if not os.path.exists(book_content_path):
        print(f"Book content path does not exist: {book_content_path}")
        # Let's check what markdown files exist in the current directory
        current_dir = os.getcwd()
        print(f"Looking for markdown files in: {current_dir}")

        # Find all markdown files in the current directory and subdirectories
        for root, dirs, files in os.walk(current_dir):
            for file in files:
                if file.endswith('.md'):
                    print(f"Found markdown file: {os.path.join(root, file)}")

        # Use the root directory as fallback
        book_content_path = current_dir

    # Run the embedding process
    try:
        print("Creating Qdrant collection...")
        if not rag_service.qdrant_service.create_collection():
            print("Failed to create Qdrant collection")
            return False

        print(f"Processing book content from: {book_content_path}")
        success = asyncio.run(rag_service.process_book_content(book_content_path))

        if success:
            print("SUCCESS: Embedding process completed successfully!")

            # Get collection info to verify embeddings were created
            collection_info = rag_service.get_collection_info()
            if collection_info:
                print(f"[INFO] Collection info:")
                print(f"   Status: {collection_info.get('status')}")
                print(f"   Vectors count: {collection_info.get('vectors_count')}")
                print(f"   Segments count: {collection_info.get('segments_count')}")
            else:
                print("[WARNING] Could not retrieve collection info")

            return True
        else:
            print("[ERROR] Embedding process failed!")
            return False

    except Exception as e:
        print(f"ERROR: Error during embedding process: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    print("Starting RAG Embedding Process")
    print("="*50)

    success = run_embedding_process()

    print("="*50)
    if success:
        print("SUCCESS: Embedding process completed successfully!")
        print("Your book content is now embedded in Qdrant and ready for semantic search!")
    else:
        print("ERROR: Embedding process failed!")
        print("Please check your .env file and internet connection, then try again.")