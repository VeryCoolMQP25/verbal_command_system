import os
import glob
import requests
import chromadb
import time
import re

class RAG:
    def __init__(self, 
                 DATA_FILES_DIR="data_files",
                 EMBEDDING_MODEL="nomic-embed-text:latest",
                 LLM_MODEL="llama3.2:latest",
                 OLLAMA_BASE_URL="http://localhost:11434",
                 CHROMA_PERSIST_DIR="chroma_db",
                 COLLECTION_NAME="data_files_collection",
                 N_RESULTS=10):
        """Initialize the LLM_RAG class with configurable parameters."""
        # Configuration
        self.DATA_FILES_DIR = DATA_FILES_DIR
        self.EMBEDDING_MODEL = EMBEDDING_MODEL
        self.LLM_MODEL = LLM_MODEL
        self.OLLAMA_BASE_URL = OLLAMA_BASE_URL
        self.CHROMA_PERSIST_DIR = CHROMA_PERSIST_DIR
        self.COLLECTION_NAME = COLLECTION_NAME
        self.N_RESULTS = N_RESULTS
        self.client = self.create_chroma_client()
        self.collection = self.get_chroma_collection()
        self.index_files()
    
    def read_rst_file(self, filepath):
        """Reads a file and returns its content."""
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                return f.read()
        except Exception as e:
            print(f"Error reading file {filepath}: {e}")
            return None
    
    def get_embedding(self, text):
        """Gets the embedding for the given text using Ollama."""
        url = f"{self.OLLAMA_BASE_URL}/api/embeddings"
        data = {
            "model": self.EMBEDDING_MODEL,
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
    
    def chunk_text(self, text, chunk_size=1000, chunk_overlap=100):
        """Simple text chunking function."""
        chunks = []
        start = 0
        while start < len(text):
            end = min(start + chunk_size, len(text))
            chunk = text[start:end]
            chunks.append(chunk)
            start += chunk_size - chunk_overlap
        return chunks
    
    def create_chroma_client(self):
        """Creates and returns a ChromaDB client."""
        return chromadb.PersistentClient(path=self.CHROMA_PERSIST_DIR)
    
    def get_chroma_collection(self):
        """Gets or creates a ChromaDB collection."""
        return self.client.get_or_create_collection(name=self.COLLECTION_NAME)

    def index_files(self):
        """Index the files in the data directory."""
        if not os.path.exists(self.DATA_FILES_DIR):
            print(f"Warning: Directory '{self.DATA_FILES_DIR}' not found.")
            return
        
        files = glob.glob(os.path.join(self.DATA_FILES_DIR, "*.*"))
        if not files:
            print(f"No files found in '{self.DATA_FILES_DIR}'.")
            return
        
        print(f"Indexing {len(files)} files...")
        for filepath in files:
            filename = os.path.basename(filepath)
            content = self.read_rst_file(filepath)
            if content:
                chunks = self.chunk_text(content)
                for i, chunk in enumerate(chunks):
                    embedding = self.get_embedding(chunk)
                    if embedding:
                        doc_id = f"{filename}_chunk_{i}"
                        self.collection.add(
                            ids=[doc_id],
                            embeddings=[embedding],
                            documents=[chunk],
                            metadatas={"source": filename, "chunk": i}
                        )
        print("Indexing complete.")
    
    def limit_response(self, text, max_sentences=3):
        """Limit response to a specific number of sentences."""
        # Split on sentence endings followed by space and capital letter
        sentences = re.split(r'[.!?]\s+(?=[A-Z])', text)
        # Return limited sentences, rejoining with the punctuation and space
        limited = sentences[:max_sentences]
        # Add final period if missing
        if limited and not limited[-1].endswith(('.', '!', '?')):
            limited[-1] += '.'
        return '. '.join(limited)
    
    def clean_llm_response(self, response):
        """Clean up the LLM response to remove any tags."""
        # Split the response into lines
        lines = response.split('\n')
        
        # Filter out lines with the assistant tag
        cleaned_lines = [line for line in lines if '<|assistant|>' not in line]
        
        # Join lines back together
        cleaned_response = ' '.join(cleaned_lines).strip()
        return cleaned_response
    
    def generate_response(self, query, max_sentences=3):
        """Generate a response to the query using RAG."""
        # Get embedding for the query
        query_embedding = self.get_embedding(query)
        if not query_embedding:
            return "I'm sorry, I couldn't process your question."
        
        # Search ChromaDB for relevant documents
        results = self.collection.query(
            query_embeddings=[query_embedding],
            n_results=self.N_RESULTS
        )
        
        if not results or not results['documents'] or not results['documents'][0]:
            return "I don't have enough information to answer this question based on the provided context."
        
        # Compile the context from retrieved documents
        retrieved_chunks = results['documents'][0]
        context = "\n\n".join(retrieved_chunks)
        
        # Formulate the prompt for the LLM
        prompt = f"""You are Tori, a helpful tour guide robot in Unity Hall. Use the following context to answer the user's question. If the answer cannot be found in the context, explicitly state "I don't have enough information to answer this question based on the provided context." Be concise but informative. Only answer in one or two concise sentences. Do not end with questions.

Context:
{context}

Question: {query}"""
        
        # Query the LLM
        url = f"{self.OLLAMA_BASE_URL}/api/generate"
        data = {
            "model": self.LLM_MODEL,
            "prompt": prompt,
            "stream": False
        }
        
        response = requests.post(url, json=data)
        response.raise_for_status()
        result = response.json()
        answer = result['response']
        
        if answer: 
            # Clean and limit the response
            answer = self.clean_llm_response(answer)
            answer = self.limit_response(answer, max_sentences)
            return answer
        else: 
            print(f"Could not get an answer from the LLM.")
            return "I'm sorry, I'm having trouble processing your question right now." 

if __name__ == "__main__":
    llm = RAG()
    test_phrases = [
        "who are you?", 
        "how many labs are in unity hall?"
    ]
    for phrase in test_phrases:
        response = llm.generate_response(phrase)
        print(f"Generated response: {response}")
