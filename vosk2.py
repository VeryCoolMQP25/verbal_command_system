import time
import json
import string
import vosk
import logging
from rag_system import RAG 
import pyaudio
import re
from pyt2s.services import stream_elements
from pydub import AudioSegment
from io import BytesIO
import simpleaudio as sa
import random 
from multiprocessing import Queue

# Configure logging
logging.getLogger('vosk').setLevel(logging.ERROR)
#  Initialize with the new LLM_RAG implementation
llm = RAG(
    DATA_FILES_DIR="data_files",  # Directory containing your data files
    EMBEDDING_MODEL="nomic-embed-text:latest",
    LLM_MODEL="llama3.2:latest",
    OLLAMA_BASE_URL="http://localhost:11434",
    CHROMA_PERSIST_DIR="chroma_db",
    COLLECTION_NAME="unity_hall_data",
    N_RESULTS=10 # Number of relevant chunks to retrieve
)

def say(text):
    try:
        data = stream_elements.requestTTS(text, stream_elements.Voice.Joanna.value)
        audio = AudioSegment.from_mp3(BytesIO(data))
        play_obj = sa.play_buffer(
            audio.raw_data, 
            num_channels=audio.channels, 
            bytes_per_sample=audio.sample_width, 
            sample_rate=audio.frame_rate
        )
        # Wait until audio finishes playing
        play_obj.wait_done()
        time.sleep(0.2)
    except Exception as e:
        print(f"TTS Error: {e}")

def clean_text(text):
    if not text:
        return ""
    
    cleaned_text = text.strip().lower()
    cleaned_text = cleaned_text.translate(str.maketrans('', '', string.punctuation))
    print(f"User: '{cleaned_text}'")    
    return cleaned_text

def initialize_vosk_recognizer(model_path, sample_rate=16000, chunk_size=4000):
    model = vosk.Model(model_path)
    rec = vosk.KaldiRecognizer(model, sample_rate)
    
    pyaudio_instance = pyaudio.PyAudio()
    stream = pyaudio_instance.open(
        format=pyaudio.paInt16, 
        channels=1, 
        rate=sample_rate, 
        input=True, 
        frames_per_buffer=chunk_size
    )
    return pyaudio_instance, stream, rec, sample_rate, chunk_size

def listen_for_text(stream, rec, sample_rate, chunk_size, messageQ=Queue(), timeout=None):
    text = ""
    start_time = time.time()
    
    while timeout is None or time.time() - start_time < timeout:
        data = stream.read(chunk_size, exception_on_overflow=False)
        
        if len(data) != 0:      
            if rec.AcceptWaveform(data):
                result = json.loads(rec.Result())
                text = result.get('text', '').strip()
        if not len(text):
            if not messageQ.empty():
                print("Queue is not empty!")
                qdat = messageQ.get()
                print("Retrieved: {} from queue".format(qdat))
                return qdat
        else:
            return text
    return None

def exit_check(text, stream):
    if text:
        cleaned_text = clean_text(text)
        exit_words = {"stop", "goodbye", "bye", "exit", "quit", "end"}
        
        for word in cleaned_text.lower():
            if word in exit_words:
                print("Exiting due to word", word)
                stream.stop_stream()  # Stop stream before robot speaks
                say("Goodbye! Say the wake phrase when you want to talk again.")
                stream.start_stream()  # Restart stream to listen for user
                time.sleep(0.5)
                return True
    return False

def wake_word(stream, text):
    if text:
        cleaned_text = clean_text(text)
        wake_phrases = ["hey tori", "hey tory", "hey torry", "hitori", "katori", "a tori", "a tory"]
        
        if any(phrase in cleaned_text for phrase in wake_phrases):
            stream.stop_stream() # Stop audio recording
            say("Hi, I'm Tori, a tour guide robot for Unity Hall. Go ahead and ask me a question.")
            print("finished saying")
            stream.start_stream() # Restart audio recording 
            return True
    return False


def handle_question(stream, rec, sample_rate, chunk_size, llm):
    # Phrases to fill empty space between user asking question and LLM response 
    random_phrases = [
        "Let me think about that for a moment. Please wait.",
        "I'm searching my database for that information. Please wait a moment.",
        "That's a great question. Let me think of an answer for you! ",
        "Give me a moment to think of a response for you! ",
        "Thanks for your question! Let me think of a response for you.", 
    ]
    
    while True:
        print("Ask question")
    #     stream.stop_stream()
    #     say("What is your question?")
    #     stream.start_stream()
    # 
        text = listen_for_text(stream, rec, sample_rate, chunk_size)
        if not text:
            continue
            
        if exit_check(text, stream):
            return False
        
        cleaned_text = clean_text(text)
        print(f"Processing question: '{cleaned_text}'")
        
        stream.stop_stream()
        # say(random.choice(random_phrases))
        
        response = llm.generate_response(cleaned_text)
        print(f"Generated response: {response}")
        
        # say(response)
        time.sleep(1)
        say("Would you like to ask another question?")
        stream.start_stream()
        
        follow_up_answer = False
        MAX_ATTEMPTS = 3
        for _ in range(MAX_ATTEMPTS):
            follow_up = listen_for_text(stream, rec, sample_rate, chunk_size)
            
            if not follow_up:
                continue
                
            if exit_check(follow_up, stream):
                return False
            
            cleaned_follow_up = clean_text(follow_up)
            
            if any(word in cleaned_follow_up for word in ["yes", "yeah", "sure", "okay", "yep", "yup"]):
                follow_up_answer = True
                break
            elif any(word in cleaned_follow_up for word in ["no", "nope", "nah", "stop"]):
                follow_up_answer = False
                break
            else:
                stream.stop_stream()
                say("I didn't get that, please say yes or no.")
                stream.start_stream()
                continue

        if follow_up_answer:
            continue
        else:
            stream.stop_stream()
            say("Thank you for your questions. Please say the wake phrase when you need me again!")
            stream.start_stream()
            time.sleep(0.5)
            return False
    
    return False

def main(messageQ):
    print("Vosk main loop started, Queue object: ",messageQ)
    vosk_model_path = "./vosk-model-en-us-0.22"
    pyaudio_instance, stream, rec, sample_rate, chunk_size = initialize_vosk_recognizer(vosk_model_path)
    
    try:
        while True: # Main wake word loop
            print("Listening for wake word...")
            text = listen_for_text(stream, rec, sample_rate, chunk_size, messageQ=messageQ)
                    
            if not text or exit_check(text, stream):
                continue  # Return to wake word loop
                
            if wake_word(stream, text):
                print("Got wake word")
                handle_question(stream, rec, sample_rate, chunk_size, llm)
    
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
    finally: # Clean shutdown 
        stream.stop_stream()
        stream.close()
        pyaudio_instance.terminate()

if __name__ == "__main__":
    main(Queue())
