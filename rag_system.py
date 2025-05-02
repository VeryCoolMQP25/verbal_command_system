import os
import glob
import requests
import chromadb
import time
import re
from tqdm import tqdm
import subprocess
import json
from io import BytesIO
import simpleaudio as sa
from pydub import AudioSegment
from pyt2s.services import stream_elements
import threading
import queue

class RAG:
    def __init__(self, DATA_FILES_DIR, EMBEDDING_MODEL, LLM_MODEL, OLLAMA_BASE_URL, CHROMA_PERSIST_DIR, COLLECTION_NAME, N_RESULTS):
        """Initialize the LLM_RAG class with configurable parameters."""
        print("Starting ollama docker container...",end='\t')
        subprocess.run(['jetson-containers', 'run', '--detach', '--name', 'ollama', 'dustynv/ollama:r36.4.0'])
        print("Done")
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
        # self.index_files()
        
    def init_speech_system(self):
        self.audio_queue = queue.Queue()
        self.speech_thread = threading.Thread(target=self._speech_worker, daemon=True)
        self.speech_thread.start()

    def _speech_worker(self):
        while True:
            text = self.audio_queue.get()
            if text == "__STOP__":
                break
            try:
                data = stream_elements.requestTTS(text, stream_elements.Voice.Joanna.value)
                audio = AudioSegment.from_mp3(BytesIO(data))
                play_obj = sa.play_buffer(
                    audio.raw_data,
                    num_channels=audio.channels,
                    bytes_per_sample=audio.sample_width,
                    sample_rate=audio.frame_rate
                )
                play_obj.wait_done()
            except Exception as e:
                print(f"TTS Error: {e}")
            self.audio_queue.task_done()

    def enqueue_speech(self, text):
        if text.strip():
            self.audio_queue.put(text.strip())

    def shutdown_speech_system(self):
        self.audio_queue.put("__STOP__")
        self.speech_thread.join()

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
        for filepath in tqdm(files):
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
    
    def limit_response(self, text, max_sentences=4):
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
        print("Generating embeddings for query...",end='\t')
        query_embedding = self.get_embedding(query)
        print("Done")
        if not query_embedding:
            return "I'm sorry, I couldn't process your question."
        
        # Search ChromaDB for relevant documents
        print("Searching ChromaDB...")
        results = self.collection.query(
            query_embeddings=[query_embedding],
            n_results=self.N_RESULTS
        )
        print("Done")
        
        if not results or not results['documents'] or not results['documents'][0]:
            return "I don't have enough information to answer this question"
        
        # Compile the context from retrieved documents
        retrieved_chunks = results['documents'][0]
        context = "\n\n".join(retrieved_chunks)
        
        # Formulate the prompt for the LLM
        prompt = f"""You are Tori, a helpful tour guide robot in Unity Hall. Use the following context to answer the user's question. If the answer cannot be found in the context, explicitly state "I don't have enough information to answer this question." Be concise but informative. Only answer in one or two concise sentences. Do not end with questions.

Context:
{context}

Question: {query}"""
        
        # Query the LLM
        url = f"{self.OLLAMA_BASE_URL}/api/generate"
        data = {
            "model": self.LLM_MODEL,
            "prompt": prompt,
            "stream": True
        }
        print("Sending request to LLM for generation...", end='\t')
        
        full_response = ""
        speak_buffer = ""
        word_buffer = ""
        
        # Keep track of speech jobs to avoid overlapping speech
        active_speech = None
        self.init_speech_system()

        with requests.post(url, json=data, stream=True, timeout=60) as response:
            response.raise_for_status()
            for line in response.iter_lines():
                if line:
                    try:
                        chunk = json.loads(line)
                        if 'response' in chunk:
                            token = chunk['response']
                            full_response += token
                            word_buffer += token
                            print(token, end='', flush=True)
                            
                            # Process words for speaking
                            if ' ' in word_buffer or any(p in word_buffer for p in ['.', '!', '?', ',', ';', ':', '-']):
                                # Extract words from buffer
                                words = word_buffer.split()
                                if words:
                                    speak_buffer += word_buffer
                                    word_buffer = ""
                                    if any(p in speak_buffer for p in ['.', '!', '?']) or len(speak_buffer) > 150:
                                        self.enqueue_speech(speak_buffer.strip())
                                        speak_buffer = ""
                            
                    except json.JSONDecodeError:
                        continue
        
        # Speak any remaining text
        if speak_buffer.strip() or word_buffer.strip():
            remaining_text = (speak_buffer + word_buffer).strip()
            if remaining_text:
                self.enqueue_speech(remaining_text)
        self.audio_queue.join()  # Wait for speech to finish
        self.shutdown_speech_system()
        print("\nDone")
        
        if full_response:
            # Clean and limit the response
            answer = self.clean_llm_response(full_response)
            answer = self.limit_response(answer, max_sentences)
            return answer
        else:
            print(f"Could not get an answer from the LLM.")
            return "I'm sorry, I'm having trouble processing your question right now."