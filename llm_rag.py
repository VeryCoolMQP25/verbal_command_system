from transformers import AutoTokenizer, AutoModelForCausalLM
from sentence_transformers import SentenceTransformer
import numpy as np
import requests
from bs4 import BeautifulSoup
import torch
import importlib
import warnings
import faiss

class LLM_RAG:
    def __init__(self):
        # Check if CUDA is available on the Jetson
        self.use_gpu = torch.cuda.is_available()
        print(f"CUDA available: {self.use_gpu}")
        if self.use_gpu:
            print(f"GPU: {torch.cuda.get_device_name(0)}")
        
        self.faiss_available = False
        try:
            self.faiss = faiss
            self.faiss_available = True
            print("FAISS is available and working correctly")
        except (ImportError, AttributeError) as e:
            warnings.warn(f"FAISS not available or not working correctly: {e}. Using numpy for vector similarity instead.")
            self.faiss = None

        self.tokenizer = AutoTokenizer.from_pretrained("TinyLlama/TinyLlama-1.1B-Chat-v1.0")        
        self.model = AutoModelForCausalLM.from_pretrained("TinyLlama/TinyLlama-1.1B-Chat-v1.0")
        
        # Move model to GPU if available
        if self.use_gpu:
            self.model = self.model.to('cuda')
            # Half precision to save memory
            self.model = self.model.half()
        
        self.embedding_model = SentenceTransformer('all-MiniLM-L6-v2')
        if self.use_gpu:
            self.embedding_model = self.embedding_model.to('cuda')

    def data_scrape(self, url):
        response = requests.get(url)
        data = BeautifulSoup(response.text, 'html.parser')
        contents = []
        for elem in data.find_all(['p', 'h1']):
            text = elem.get_text().strip()
            if text:
                contents.append(f"{elem.name}: {text}")
        return contents
    
    # Numpy alternative if GPU doesn't work 
    def numpy_similarity_search(self, query_vector, document_vectors, k=3):
        distances = np.linalg.norm(document_vectors - query_vector, axis=1)        
        indices = np.argsort(distances)[:k]
        distances = distances[indices]
        # Return in FAISS-like format
        return np.array([distances]), np.array([indices])

    def embedding(self, url):
        documents = self.data_scrape(url)
        
        # Batch process documents if there are many
        batch_size = 1
        document_embeddings = []
        
        for i in range(0, len(documents), batch_size):
            batch = documents[i:i+batch_size]
            with torch.no_grad():
                batch_embeddings = self.embedding_model.encode(batch)
                document_embeddings.append(batch_embeddings)
        
        document_embeddings = np.vstack(document_embeddings)
        
        if self.faiss_available:
            # Create FAISS index
            dimension = document_embeddings.shape[1]
            index = self.faiss.IndexFlatL2(dimension)
            index.add(document_embeddings.astype(np.float32))
            print("Using FAISS index")
        else:
            # Just store the embeddings for numpy similarity search
            index = document_embeddings
            print("Using NumPy for similarity search")
        
        return documents, index

    def query(self, query_text, documents, index):
        # Encode the query
        query_embedding = self.embedding_model.encode([query_text])
        
        # Convert to numpy array if it's a tensor
        if isinstance(query_embedding, torch.Tensor):
            query_embedding = query_embedding.cpu().numpy()
        
        # Search for similar documents
        k = min(3, len(documents))  # Ensure k is not larger than the number of documents
        
        if self.faiss_available:
            distances, indices = index.search(query_embedding.astype(np.float32), k)
        else:
            distances, indices = self.numpy_similarity_search(query_embedding[0], index, k)
        
        # Get retrieved documents
        retrieved_docs = []
        for i, dist in zip(indices[0], distances[0]):
            if i < len(documents):  # Ensure index is valid
                weight = 1 / (1 + dist)
                retrieved_docs.append(f"{documents[i]}")
        
        context = "\n".join(retrieved_docs)
        
        # Prompt engineering 
        prompt = f"""Context: {context}
Question: {query_text}
Instructions: Only answer questions about Unity Hall. If you are not sure of the answer and it is not in the dataset, just say you don't know. Do not reference other schools or universities outside of Worcester Polytechnic Institute (WPI). If asked about other universities, say you do not know. Answer directly without using lists or numbers. Provide a natural flowing response. If you are not sure, just say you don't know.
Answer:"""
        
        # Generate response
        inputs = self.tokenizer(prompt, return_tensors="pt")
        
        if self.use_gpu:
            inputs = {k: v.to('cuda') for k, v in inputs.items()}
        
        with torch.no_grad():
            outputs = self.model.generate(
                **inputs,
                max_new_tokens=100,
                temperature=0.2, # Predictability/creativity 
                top_p=0.5, # Length of response  
                do_sample=True,
                repetition_penalty=1.2,
                eos_token_id=self.tokenizer.convert_tokens_to_ids(['.'])[0],
                pad_token_id=self.tokenizer.eos_token_id
            )
        
        response = self.tokenizer.decode(outputs[0], skip_special_tokens=True)
        answer = response.split("Answer:")[-1].strip()
        return answer

    def generate_response(self, input_text):
        url = "https://aashigoel03.github.io/unity-hall/index.html"
        
        # Cache the embeddings and index
        if not hasattr(self, 'documents') or not hasattr(self, 'index'):
            self.documents, self.index = self.embedding(url)
        
        response = self.query(input_text, self.documents, self.index)
        print("\nAnswer:", response)
        return response

if __name__ == "__main__":
    llm = LLM_RAG()
    test_phrases = [
        "How many stairs are in Unity Hall?"
    ]
    for phrase in test_phrases:
        result = llm.generate_response(phrase)