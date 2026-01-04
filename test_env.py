"""
Test script to verify your environment variables are set correctly
"""

import os
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

print("Environment Variables Check")
print("="*50)

# Check Qdrant variables
qdrant_url = os.getenv("QDRANT_URL", os.getenv("QDRANT_HOST", "localhost"))
qdrant_port = os.getenv("QDRANT_PORT", "6333")
qdrant_api_key = os.getenv("QDRANT_API_KEY")
qdrant_collection_name = os.getenv("QDRANT_COLLECTION_NAME", "book_content_chunks")

print(f"QDRANT_URL: {qdrant_url}")
print(f"QDRANT_PORT: {qdrant_port}")
print(f"QDRANT_API_KEY exists: {bool(qdrant_api_key)}")
print(f"QDRANT_COLLECTION_NAME: {qdrant_collection_name}")

if qdrant_api_key and qdrant_url == "localhost":
    print("\n[ISSUE] You have a QDRANT_API_KEY but QDRANT_URL is still 'localhost'")
    print("   Please update your .env file with your Qdrant Cloud instance URL")
    print("   Example: QDRANT_URL=your-instance-subdomain.europe-west3-0.gcp.cloud.qdrant.io")
else:
    print("\n[OK] Qdrant configuration appears correct")

# Check Cohere variables
cohere_api_key = os.getenv("COHERE_API_KEY")
cohere_model = os.getenv("COHERE_MODEL", "embed-english-v3.0")

print(f"\nCOHERE_API_KEY exists: {bool(cohere_api_key)}")
print(f"COHERE_MODEL: {cohere_model}")

if not cohere_api_key:
    print("[ERROR] COHERE_API_KEY is not set in your .env file")
else:
    print("[OK] Cohere API key is set")

print(f"\n{'='*50}")
if (qdrant_api_key and qdrant_url != "localhost") and cohere_api_key:
    print("[SUCCESS] All required environment variables are set correctly!")
    print("You can now run the embedding process.")
else:
    print("[ERROR] Some environment variables need to be corrected.")
    print("Please update your .env file and try again.")