import os
import glob
import requests
import chromadb
from chromadb.utils import embedding_functions

CLASS_FILES_DIR = "class_files"
EMBEDDING_MODEL = "nomic-embed-text:latest"
LLM_MODEL = "tinyllama:latest"
OLLAMA_BASE_URL = "http://localhost:11434"  # Default Ollama URL
EMBEDDING_DIMENSION = 768  # Dimension of nomic-embed-text embeddings (adjust if needed)
CHROMA_PERSIST_DIR = "chroma_db"
COLLECTION_NAME = "class_files_collection"
N_RESULTS = 10

def read_rst_file(filepath):
    """Reads an RST file and returns its content."""
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            return f.read()
    except Exception as e:
        print(f"Error reading file {filepath}: {e}")
        return None

def get_embedding(text):
    """Gets the embedding for the given text using Ollama."""
    url = f"{OLLAMA_BASE_URL}/api/embeddings"
    data = {
        "model": EMBEDDING_MODEL,
        "prompt": text
    }
    try:
        response = requests.post(url, json=data)
        response.raise_for_status()
        result = response.json()
        return result['embedding']
    except requests.exceptions.RequestException as e:
        print(f"Error getting embedding from Ollama: {e}")
        return None

def query_llm(prompt):
    """Queries the LLM using Ollama."""
    url = f"{OLLAMA_BASE_URL}/api/generate"
    data = {
        "model": LLM_MODEL,
        "prompt": prompt,
        "stream": False  # Set to True for streaming output
    }
    try:
        response = requests.post(url, json=data)
        response.raise_for_status()
        result = response.json()
        return result['response']
    except requests.exceptions.RequestException as e:
        print(f"Error querying LLM from Ollama: {e}")
        return None

def chunk_text(text, chunk_size=1000, chunk_overlap=100):
    """Simple text chunking function."""
    chunks = []
    start = 0
    while start < len(text):
        end = min(start + chunk_size, len(text))
        chunk = text[start:end]
        chunks.append(chunk)
        start += chunk_size - chunk_overlap
    return chunks

def create_chroma_client(persist_directory=CHROMA_PERSIST_DIR):
    """Creates and returns a ChromaDB client."""
    return chromadb.PersistentClient(path=persist_directory)

def get_chroma_collection(client, collection_name=COLLECTION_NAME):
    """Gets or creates a ChromaDB collection."""
    return client.get_or_create_collection(name=collection_name)

def limit_response(text, max_sentences=2):
    """Limit response to a specific number of sentences."""
    import re
    # Split on sentence endings followed by space and capital letter
    sentences = re.split(r'[.!?]\s+(?=[A-Z])', text)
    # Return limited sentences, rejoining with the punctuation and space
    limited = sentences[:max_sentences]
    # Add final period if missing
    if limited and not limited[-1].endswith(('.', '!', '?')):
        limited[-1] += '.'
    return '. '.join(limited)

# Create ChromaDB client and collection
client = create_chroma_client()
collection = get_chroma_collection(client)

# Index the RST files
if not os.path.exists(CLASS_FILES_DIR):
    print(f"Error: Directory '{CLASS_FILES_DIR}' not found. Please create it and add your RST files.")
else:
    rst_files = glob.glob(os.path.join(CLASS_FILES_DIR, "*.rst"))
    if not rst_files:
        print(f"No RST files found in '{CLASS_FILES_DIR}'.")
    else:
        print("Indexing RST files...")
        for filepath in rst_files:
            filename = os.path.basename(filepath)
            content = read_rst_file(filepath)
            if content:
                chunks = chunk_text(content)
                for i, chunk in enumerate(chunks):
                    embedding = get_embedding(chunk)
                    if embedding:
                        doc_id = f"{filename}_chunk_{i}"
                        collection.add(
                            ids=[doc_id],
                            embeddings=[embedding],
                            documents=[chunk],
                            metadatas={"source": filename, "chunk": i}
                        )
        print("Indexing complete.")

query = input("Ask a question about the class files: ")

# Get embedding for the query
query_embedding = get_embedding(query)
retrieved_chunks = None

if query_embedding:
    # Search ChromaDB for relevant documents
    results = collection.query(
        query_embeddings=[query_embedding],
        n_results=N_RESULTS
    )

    if results and results['documents'] and results['documents'][0]:
        retrieved_chunks = results['documents'][0]
        print("\nRetrieved Chunks:")
        # for i, chunk in enumerate(retrieved_chunks):
        #     print(f"--- Chunk {i+1} ---")
        #     print(chunk)
        context = "\n\n".join(retrieved_chunks)
    else:
        print("No relevant documents found for your query.")
else:
    print("Could not generate embedding for your query.")

if retrieved_chunks:
    # Formulate the prompt for the LLM
    prompt = f"""You are a helpful assistant. Use the following context from class files to answer the user's question. If the answer cannot be found in the context, explicitly state "I don't have enough information to answer this question based on the provided context." DO NOT make up information or use your general knowledge to answer. Please limit your responses to 1 or 2 sentences. 

Context:
{context}

Question: {query}"""

    # Query the LLM
    print("\nGenerating answer...")
    url = f"{OLLAMA_BASE_URL}/api/generate"
    data = {
        "model": LLM_MODEL,
        "prompt": prompt,
        "stream": False  # Set to True for streaming output
    }
    response = requests.post(url, json=data)
    response.raise_for_status()
    result = response.json()
    answer = result['response']
    if answer:
        print("\nAnswer:")
        answer = limit_response(answer)
        print(answer)
    else:
        print("Could not get an answer from the LLM.")