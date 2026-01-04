"""
Test script to check Qdrant vector storage
"""

import sys
import os

# Add the backend/src directory to the Python path
sys.path.insert(0, os.path.join(os.getcwd(), 'backend', 'src'))

from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct
import os

# Load environment variables from .env file
load_dotenv()

# Initialize the client
host = os.getenv("QDRANT_URL", os.getenv("QDRANT_HOST", "localhost"))
api_key = os.getenv("QDRANT_API_KEY")
collection_name = os.getenv("QDRANT_COLLECTION_NAME", "book_content_chunks")

if api_key and host != "localhost":
    # For Qdrant Cloud - ensure URL includes https://
    if not host.startswith("https://") and not host.startswith("http://"):
        host = f"https://{host}"
    client = QdrantClient(
        url=host,
        api_key=api_key
    )
else:
    # For local development
    client = QdrantClient(host=host, port=6333)

print(f"Testing Qdrant connection to: {host}")
print(f"Collection: {collection_name}")

try:
    # Create a test point with a simple embedding
    test_embedding = [0.1] * 1024  # Simple test embedding
    test_point = PointStruct(
        id=999999,  # Use a high ID to avoid conflicts
        vector={"content": test_embedding},  # Named vector
        payload={"test": True, "description": "This is a test point"}
    )
    
    # Upsert the test point
    client.upsert(
        collection_name=collection_name,
        points=[test_point]
    )
    
    print("Test point stored successfully")
    
    # Retrieve the test point to check if vector is stored correctly
    retrieved_points = client.retrieve(
        collection_name=collection_name,
        ids=[999999],
        with_payload=True,
        with_vectors=True  # Important: Request vectors to be returned
    )
    
    if retrieved_points:
        point = retrieved_points[0]
        print(f"Retrieved point ID: {point.id}")
        print(f"Point payload: {point.payload}")
        print(f"Point vector type: {type(point.vector)}")
        print(f"Point vector: {point.vector}")
        if isinstance(point.vector, dict) and 'content' in point.vector:
            print(f"Content vector first 5 values: {point.vector['content'][:5]}")
            print(f"Content vector length: {len(point.vector['content'])}")
        elif isinstance(point.vector, list):
            print(f"Vector first 5 values: {point.vector[:5]}")
            print(f"Vector length: {len(point.vector)}")
    else:
        print("No points retrieved")
    
    # Clean up - delete the test point
    client.delete(
        collection_name=collection_name,
        points_selector=models.PointIdsList(
            points=[999999]
        )
    )
    print("Test point cleaned up")
    
except Exception as e:
    print(f"Error in test: {str(e)}")
    import traceback
    traceback.print_exc()