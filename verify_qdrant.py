"""
Script to verify the data stored in Qdrant
"""

import sys
import os

# Add the backend/src directory to the Python path
sys.path.insert(0, os.path.join(os.getcwd(), 'backend', 'src'))

from dotenv import load_dotenv
from services.qdrant_service import QdrantService

# Load environment variables from .env file
load_dotenv()

def verify_qdrant_data():
    """
    Verify that the data was stored correctly in Qdrant
    """
    print("Verifying data stored in Qdrant...")
    
    # Initialize the Qdrant service
    qdrant_service = QdrantService()
    
    try:
        # Get collection info
        collection_info = qdrant_service.get_collection_info()
        if collection_info:
            print(f"Collection Status: {collection_info.get('status')}")
            print(f"Vectors Count: {collection_info.get('vectors_count')}")
            print(f"Segments Count: {collection_info.get('segments_count')}")
        else:
            print("Could not retrieve collection info")
            return
        
        # Get all points to verify the structure
        all_points = qdrant_service.client.scroll(
            collection_name=qdrant_service.collection_name,
            limit=5  # Just get first 5 to verify structure
        )
        
        if all_points[0]:  # Points exist
            print(f"\nSample of stored points (first 5):")
            for point in all_points[0][:5]:
                print(f"  ID: {point.id}")
                print(f"  Payload keys: {list(point.payload.keys()) if point.payload else 'None'}")
                print(f"  Vector type: {type(point.vector)}")
                if hasattr(point.vector, 'get'):
                    vector_keys = list(point.vector.keys()) if point.vector else []
                    print(f"  Vector keys: {vector_keys}")
                    if vector_keys:
                        sample_vector = point.vector[vector_keys[0]]
                        print(f"  Vector size (first {vector_keys[0]}): {len(sample_vector) if sample_vector else 0}")
                else:
                    print(f"  Vector size (direct): {len(point.vector) if point.vector else 0}")
                print(f"  Payload sample: {str(point.payload)[:200]}...")
                print("  ---")
        else:
            print("No points found in the collection")
            
        # Check a few more details
        print(f"\nVerifying collection structure...")
        collection_info = qdrant_service.client.get_collection(qdrant_service.collection_name)
        if hasattr(collection_info.config, 'params'):
            print(f"Collection vectors config: {collection_info.config.params.vectors}")
        else:
            print(f"Collection vectors config: {collection_info.config}")
        
    except Exception as e:
        print(f"Error verifying Qdrant data: {str(e)}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    print("Verifying Qdrant Data Structure")
    print("="*50)
    
    verify_qdrant_data()
    
    print("="*50)
    print("Verification complete!")