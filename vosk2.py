#will not work if named vosk.py 
import pyttsx3
import time
import string
import json
import vosk
import pyaudio
import rclpy
import logging
from nav_classifier import RoomClassifier
from llm_rag import LLM_RAG
from navigation_stack import NavigationNode
import re
from text2digits import text2digits

# Configure logging
logging.getLogger('vosk').setLevel(logging.ERROR)

engine = pyttsx3.init()
t2d = text2digits.Text2Digits()
classifier = RoomClassifier()
llm = LLM_RAG()

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

def listen_for_text(stream, rec, sample_rate, chunk_size, timeout=None):
    start_time = time.time()
    
    while timeout is None or time.time() - start_time < timeout:
        data = stream.read(chunk_size, exception_on_overflow=False)
        
        if len(data) == 0:
            continue
        
        if rec.AcceptWaveform(data):
            result = json.loads(rec.Result())
            text = result.get('text', '').strip()
            
            if text:
                return text
    return None

def wake_word(text):
    if text:
        cleaned_text = clean_text(text)
        wake_phrases = ["hey tori", "hey tory", "hey torry", "hitori"]
        
        if any(phrase in cleaned_text for phrase in wake_phrases):
            engine.say("Hi, I'm Tori, a tour guide robot in Unity Hall. Would you like to say a navigation command or ask me a question?")
            engine.runAndWait()
            time.sleep(7)
            return True
    return False


def convert_number_words(sentence):
    try:
        return t2d.convert(sentence)
    except Exception as e:
        return f"Error: {e}"


def handle_navigation(stream, rec, sample_rate, chunk_size, classifier):
    engine.say("Where do you want to go?")
    engine.runAndWait()
    time.sleep(2)
    
    MAX_ATTEMPTS = 3
    for attempts in range(MAX_ATTEMPTS):
        text = listen_for_text(stream, rec, sample_rate, chunk_size)
        
        if exit_check(text):
            return None
        
        if text:
            cleaned_text = clean_text(text)
            cleaned_text2 = convert_number_words(cleaned_text)
            print(f"Original: '{cleaned_text}', Converted: '{cleaned_text2}'")
            
            response = classifier.get_navigation_response(cleaned_text2)
            print("Navigation Response:", response)
            
            if not response['success']:
                engine.say(response['message'])
                engine.runAndWait()
                
                # if last attempt, reset context
                if attempts == MAX_ATTEMPTS - 1:
                    classifier.reset_context()
                    engine.say("Let's start over. Please say the wake phrase when you're ready.")
                    engine.runAndWait()
                    return False
                continue
            
            var = classifier.extract_location_info(cleaned_text2)
            room_num = var.get('room_number') or var.get('room')
            floor = var.get('floor')
            
            # Ensure floor is a string
            if isinstance(floor, list):
                floor = floor[0]
            
            engine.say(response['message'])
            engine.runAndWait()
            return True, room_num, floor
    return False

def question(stream, rec, sample_rate, chunk_size, llm):
    print("Ask question")
    engine.say("What is your question?")
    engine.runAndWait()
    time.sleep(1)

    text = listen_for_text(stream, rec, sample_rate, chunk_size)
    if exit_check(text):
        return None
    
    if text:
        cleaned_text = clean_text(text)
        response = llm.generate_response(cleaned_text)
        
        engine.say(response)
        engine.runAndWait()
        print(response)
        return True
    return False

def exit_check(text):
    if text:
        cleaned_text = clean_text(text)
        exit_words = {"stop", "goodbye", "bye"}
        
        if any(word in cleaned_text.lower() for word in exit_words):
            engine.say("Goodbye!")
            engine.runAndWait()
            return True
    return False

def main():
    vosk_model_path = "./vosk-model-small-en-us-0.15"
    pyaudio_instance, stream, rec, sample_rate, chunk_size = initialize_vosk_recognizer(vosk_model_path)
    
    try:
        while True: #wake word loop
            print("Please speak...")
            while True:
                text = listen_for_text(stream, rec, sample_rate, chunk_size)
                if exit_check(text):
                    return
                if wake_word(text):
                    break
            
            while True: #command loop 
                print("Please speak...")
                text = listen_for_text(stream, rec, sample_rate, chunk_size)
                if exit_check(text):
                    return
                
                if text:
                    cleaned_text = clean_text(text)
                    
                    if "navigation" in cleaned_text: 
                        nav_result = handle_navigation(stream, rec, sample_rate, chunk_size, classifier)
                        
                        if nav_result is None:
                            return
                        
                        if nav_result[0]:  # successful navigation
                            room_num, floor = nav_result[1], nav_result[2]
                            
                            rclpy.init()
                            nav_stack = NavigationNode()
                            
                            with open('current_navigation.json', 'w') as f:
                                json.dump({
                                    "room": room_num, 
                                    "floor": floor, 
                                    "timestamp": time.time()
                                }, f)
                            
                            try:
                                success = nav_stack.navigate(room_num, floor)
                                if success:
                                    rclpy.spin(nav_stack)
                                else:
                                    nav_stack.get_logger().error("Navigation failed!")
                            finally:
                                nav_stack.destroy_node()
                                rclpy.shutdown()
                            
                            classifier.reset_context()
                            return
                        
                        if not nav_result[0]:  # restart after 3 attempts
                            break
                    
                    elif "question" in cleaned_text:
                        question(stream, rec, sample_rate, chunk_size, llm)
                        return
                
                time.sleep(1)
    
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
    finally:
        stream.stop_stream()
        stream.close()
        pyaudio_instance.terminate()

if __name__ == "__main__":
    main()
    # print(convert_number_words("room one fifty one"))  # Output: "room 151"
    # print(convert_number_words("room three oh five"))  # Output: "room 151"
    # print(convert_number_words("room two forty one"))  # Output: "room 151"
