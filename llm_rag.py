from transformers import AutoTokenizer, AutoModelForCausalLM
from sentence_transformers import SentenceTransformer
import faiss
import numpy as np
import requests
from bs4 import BeautifulSoup

class LLM_RAG:
    def data_scrape(self, url):
        response = requests.get(url)
        data = BeautifulSoup(response.text, 'html.parser') # returns html content of website 
        
        contents = []
        for elem in data.find_all(['p', 'h1']):
            text = elem.get_text().strip()
            if text:
                contents.append(f"{elem.name}: {text}")
        return contents

    def embedding(self, url):
        model = AutoModelForCausalLM.from_pretrained("TinyLlama/TinyLlama-1.1B-Chat-v1.0")
        tokenizer = AutoTokenizer.from_pretrained("TinyLlama/TinyLlama-1.1B-Chat-v1.0")
        embedding_model = SentenceTransformer('all-MiniLM-L6-v2')
        
        documents = self.data_scrape(url)
        document_embeddings = embedding_model.encode(documents) #converts docs to vector embeddings

        index = faiss.IndexFlatL2(document_embeddings.shape[1]) #set up FAISS index for similarity search 
        index.add(document_embeddings)
        
        return model, tokenizer, embedding_model, documents, index

    def query(self, query, model, tokenizer, embedding_model, documents, index):
        query_embedding = embedding_model.encode([query]) #converts query to vector embeddings 
        distances, indices = index.search(query_embedding, k=3) #searches faiss for 3 most similar docs 
        
        retrieved_docs = [] #calculated weights for each doc
        for i, dist in zip(indices[0], distances[0]):
            weight = 1 / (1 + dist)
            retrieved_docs.append(f"{documents[i]}")
        
        context = "\n".join(retrieved_docs)
        
        # prompt engineering 
        prompt = f"""Context: {context}
        Question: {query}
        Instructions: Answer without using lists or numbers. Provide a natural flowing response. If you are not sure, just say you don't know. 
        Answer:"""

        inputs = tokenizer(prompt, return_tensors="pt")
        
        outputs = model.generate(
            **inputs,
            max_new_tokens=100,
            temperature=0.7,
            top_p=0.9,
            repetition_penalty=1.2,
            eos_token_id=tokenizer.convert_tokens_to_ids(['.'])[0], # stop after 1 sentence 
            pad_token_id=tokenizer.eos_token_id
        )
        
        response = tokenizer.decode(outputs[0], skip_special_tokens=True)
        answer = response.split("Answer:")[-1].strip() # return answer after "Answer:" 
        return answer 
    
    def generate_response(self, input): 
        url = "https://aashigoel03.github.io/unity-hall/index.html"
        model, tokenizer, embedding_model, documents, index = self.embedding(url)
        response = self.query(input, model, tokenizer, embedding_model, documents, index)
        print("\nAnswer:", response)
        return response
    
if __name__ == "__main__":
    llm = LLM_RAG() 
    
    test_phrases = [
        "What is the best part about Unity Hall?"
    ]
    
    for phrase in test_phrases:
        result = llm.generate_response(phrase)