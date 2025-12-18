import os
import glob
from pathlib import Path
from typing import List

from dotenv import load_dotenv
from langchain_text_splitters import RecursiveCharacterTextSplitter
from langchain_openai import OpenAIEmbeddings
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct

# Load environment variables
load_dotenv()

# Initialize OpenAI embeddings
embeddings = OpenAIEmbeddings(model="text-embedding-3-small", openai_api_key=os.getenv("OPENAI_API_KEY"))

def process_markdown_files(docs_dir: str) -> List[dict]:
    """
    Loop through all .md files in the docs directory and extract content
    """
    md_files = glob.glob(os.path.join(docs_dir, "**", "*.md"), recursive=True)
    documents = []

    for file_path in md_files:
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                content = file.read()

                # Create document object with content and metadata
                document = {
                    'content': content,
                    'source_file': file_path,
                    'relative_path': os.path.relpath(file_path, docs_dir)
                }
                documents.append(document)
                print(f"Processed: {file_path}")
        except Exception as e:
            print(f"Error reading file {file_path}: {e}")

    return documents

def split_documents(documents: List[dict], chunk_size: int = 1000) -> List[dict]:
    """
    Split documents into chunks of specified size using RecursiveCharacterTextSplitter
    """
    text_splitter = RecursiveCharacterTextSplitter(
        chunk_size=chunk_size,
        chunk_overlap=200,  # Overlap to maintain context
        length_function=len,
    )

    chunks = []
    for doc in documents:
        content = doc['content']
        source_file = doc['source_file']
        relative_path = doc['relative_path']

        # Split the content
        split_texts = text_splitter.split_text(content)

        for i, chunk in enumerate(split_texts):
            chunk_doc = {
                'content': chunk,
                'source_file': source_file,
                'relative_path': relative_path,
                'chunk_index': i
            }
            chunks.append(chunk_doc)

    return chunks

def get_openai_embedding(text: str) -> List[float]:
    """
    Get embedding for text using OpenAI's text-embedding-3-small model
    """
    try:
        # Use the OpenAI embeddings
        embedding_vector = embeddings.embed_query(text)
        return embedding_vector
    except Exception as e:
        print(f"Error getting embedding for text: {e}")
        return []

def upload_to_qdrant(chunks: List[dict], collection_name: str = "my_book_collection"):
    """
    Upload chunks to Qdrant collection
    """
    # Initialize Qdrant client
    qdrant_url = os.getenv("QDRANT_URL", "http://localhost:6333")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    if qdrant_api_key:
        client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
    else:
        client = QdrantClient(url=qdrant_url)

    # Check if collection exists, create if it doesn't
    try:
        client.get_collection(collection_name)
        print(f"Collection '{collection_name}' exists")
    except:
        print(f"Creating collection '{collection_name}'")
        # OpenAI text-embedding-3-small has a fixed dimension of 1536
        embedding_size = 1536

        client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(
                size=embedding_size,
                distance=models.Distance.COSINE
            )
        )

    # Prepare points for upload
    points = []
    for i, chunk in enumerate(chunks):
        content = chunk['content']
        source_file = chunk['source_file']
        relative_path = chunk['relative_path']
        chunk_index = chunk['chunk_index']

        # Get embedding for the content
        embedding_vector = get_openai_embedding(content)

        if embedding_vector:  # Only add if embedding was successful
            point = PointStruct(
                id=i,
                vector=embedding_vector,
                payload={
                    "content": content,
                    "source_file": source_file,
                    "relative_path": relative_path,
                    "chunk_index": chunk_index
                }
            )
            points.append(point)

        if (i + 1) % 100 == 0:  # Progress update every 100 chunks
            print(f"Processed {i + 1}/{len(chunks)} chunks")

    # Upload points to Qdrant in batches
    batch_size = 100
    for i in range(0, len(points), batch_size):
        batch = points[i:i + batch_size]
        try:
            client.upsert(collection_name=collection_name, points=batch)
            print(f"Uploaded batch {i//batch_size + 1}/{(len(points) - 1)//batch_size + 1}")
        except Exception as e:
            print(f"Error uploading batch {i//batch_size + 1}: {e}")

    print(f"Successfully uploaded {len(points)} chunks to Qdrant collection '{collection_name}'")

def main():
    """
    Main function to run the ingestion pipeline
    """
    # Define the docs directory
    docs_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "frontend", "docs")

    print("Starting ingestion process...")

    # Step 1: Process all markdown files
    print("Step 1: Processing markdown files...")
    documents = process_markdown_files(docs_dir)
    print(f"Found and processed {len(documents)} markdown files")

    # Step 2: Split documents into chunks
    print("Step 2: Splitting documents into chunks...")
    chunks = split_documents(documents, chunk_size=1000)
    print(f"Split into {len(chunks)} chunks")

    # Step 3: Upload to Qdrant
    print("Step 3: Uploading to Qdrant...")
    upload_to_qdrant(chunks, collection_name="my_book_collection")

    print("Ingestion process completed!")

if __name__ == "__main__":
    main()