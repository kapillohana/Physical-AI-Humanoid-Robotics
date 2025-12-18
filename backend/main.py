from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional
import os
import openai
from qdrant_client import QdrantClient
from qdrant_client.http import models
import logging
from pathlib import Path


# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Load environment variables
from dotenv import load_dotenv
load_dotenv()

# Initialize OpenAI client
openai_client = openai.OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

# Initialize Qdrant client
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL", "http://localhost:6333"),
    api_key=os.getenv("QDRANT_API_KEY")
)

# Create FastAPI app
app = FastAPI(title="RAG Chatbot API", version="1.0.0")

# Add CORS middleware to allow frontend communication
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],  # Allow Docusaurus frontend
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Request model for the chat endpoint
class ChatRequest(BaseModel):
    query: str
    selected_text: Optional[str] = None

# Response model for the chat endpoint
class ChatResponse(BaseModel):
    response: str
    context_used: str

@app.post("/api/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """
    Chat endpoint that accepts a query and optional selected_text.
    If selected_text is provided, use it as context.
    Otherwise, search Qdrant for relevant chunks.
    """
    try:
        query = request.query
        selected_text = request.selected_text

        # Determine context based on whether selected_text is provided
        if selected_text:
            context = selected_text
            logger.info("Using provided selected_text as context")
        else:
            # Search Qdrant for relevant chunks
            logger.info("Searching Qdrant for relevant chunks")
            context = search_qdrant_context(query)

        # Generate response using GPT-4o-mini
        response = generate_response(query, context)

        return ChatResponse(response=response, context_used=context)

    except Exception as e:
        logger.error(f"Error in chat endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

def search_qdrant_context(query: str) -> str:
    """
    Search Qdrant for the top 3 relevant chunks using text-embedding-3-small
    """
    try:
        # Validate that OpenAI API key is set before making API call
        openai_api_key = os.getenv("OPENAI_API_KEY")
        if not openai_api_key or openai_api_key.strip() == "":
            raise ValueError("OPENAI_API_KEY is not set in environment variables")

        # Create embedding for the query using OpenAI
        response = openai_client.embeddings.create(
            input=query,
            model="text-embedding-3-small"
        )
        query_embedding = response.data[0].embedding

        # Query Qdrant for the top 3 most relevant chunks using the modern API
        from qdrant_client.http import models
        search_result = qdrant_client.query_points(
            collection_name="my_book_collection",
            query=query_embedding,
            limit=3,
            with_payload=True
        ).points

        # Extract content from the search results with source information
        context_chunks = []
        for result in search_result:
            if result.payload and 'content' in result.payload:
                content = result.payload['content']
                source_file = result.payload.get('relative_path', 'unknown')
                # Include source information to help the AI reference chapters
                chunk_with_source = f"[Source: {source_file}]\n{content}"
                context_chunks.append(chunk_with_source)

        # Combine the chunks into a single context string
        context = "\n\n".join(context_chunks)
        print(f"Retrieved {len(context_chunks)} context chunks from Qdrant")  # Debug print
        logger.info(f"Retrieved {len(context_chunks)} context chunks from Qdrant")

        return context

    except Exception as e:
        logger.error(f"Error searching Qdrant: {str(e)}")
        raise e

def generate_response(query: str, context: str) -> str:
    """
    Generate response using GPT-4o-mini with the provided context
    """
    try:
        # Validate that OpenAI API key is set before making API call
        openai_api_key = os.getenv("OPENAI_API_KEY")
        if not openai_api_key or openai_api_key.strip() == "":
            raise ValueError("OPENAI_API_KEY is not set in environment variables")

        # Create a prompt that includes the context and query
        prompt = f"""
        You are a Physical AI Expert Mentor helping students learn about Physical AI & Humanoid Robotics.
        Use the following context to answer the user's question.
        If the context doesn't contain relevant information, say so.

        Context:
        {context}

        Question: {query}

        Answer:
        """

        # Generate response using GPT-4o-mini
        response = openai_client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {"role": "system", "content": "You are a Physical AI Expert Mentor for the Physical AI & Humanoid Robotics textbook. Always be encouraging and supportive to students. When providing information from the textbook, mention which chapter/module the information comes from based on the file path in the context. Use a friendly, educational tone and encourage students in their learning journey. Be concise but informative, and help them understand complex concepts in Physical AI and Robotics."},
                {"role": "user", "content": prompt}
            ],
            max_tokens=500,
            temperature=0.7
        )

        # Extract the response text
        answer = response.choices[0].message.content.strip()
        logger.info("Successfully generated response with GPT-4o-mini")

        return answer

    except Exception as e:
        logger.error(f"Error generating response: {str(e)}")
        raise e

@app.get("/")
def read_root():
    return {"message": "RAG Chatbot API is running!"}

@app.get("/health")
def health_check():
    return {"status": "healthy"}

# For Vercel deployment, export the FastAPI app
# The app will be served by Vercel's Python runtime
# The if __name__ == "__main__" block is only for local development
app_instance = app

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)