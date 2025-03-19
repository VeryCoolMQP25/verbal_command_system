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
from pyt2s.services import stream_elements
from pydub import AudioSegment
from io import BytesIO
import simpleaudio as sa

# Configure logging
logging.getLogger('vosk').setLevel(logging.ERROR)

t2d = text2digits.Text2Digits()
classifier = RoomClassifier()
llm = LLM_RAG()

# Using text to speech library (pyt2s) 
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

def exit_check(text):
    if text:
        cleaned_text = clean_text(text)
        exit_words = {"stop", "goodbye", "bye", "exit", "quit", "end"}
        
        if any(word in cleaned_text.lower() for word in exit_words):
            say("Goodbye!")
            return True
    return False

def wake_word(stream, text):
    if text:
        cleaned_text = clean_text(text)
        wake_phrases = ["hey tori", "hey tory", "hey torry", "hitori"]
        
        if any(phrase in cleaned_text for phrase in wake_phrases):
            say("Hi, I'm Tori, a tour guide robot in Unity Hall. Would you like to say a navigation command or ask me a question?")
            stream.stop_stream() # Stop audio recording (so speech to text lib does not pick up Tori's words)
            time.sleep(1)
            stream.start_stream() # Restart audio recording 
            return True
    return False

def convert_number_words(sentence):
    try:
        return t2d.convert(sentence)
    except Exception as e:
        return f"Error: {e}"

def handle_navigation(stream, rec, sample_rate, chunk_size, classifier):
    say("Where do you want to go?")
    time.sleep(1)
    
    MAX_ATTEMPTS = 3
    for attempts in range(MAX_ATTEMPTS):
        text = listen_for_text(stream, rec, sample_rate, chunk_size)
        
        if not text:
            continue
            
        if exit_check(text):
            return None
        
        cleaned_text = clean_text(text)
        cleaned_text2 = convert_number_words(cleaned_text)
        print(f"Original: '{cleaned_text}', Converted: '{cleaned_text2}'")
        
        response = classifier.get_navigation_response(cleaned_text2)
        print("Navigation Response:", response)
        
        if not response['success']:
            say(response['message'])
            
            # If last attempt, reset context
            if attempts == MAX_ATTEMPTS - 1:
                classifier.reset_context()
                say("Let's start over. Please say the wake phrase when you're ready.")
                return False
            continue
        
        var = classifier.extract_location_info(cleaned_text2)
        room_num = var.get('room_number') or var.get('room')
        floor = var.get('floor')
        
        # Ensure floor is a string
        if isinstance(floor, list):
            floor = floor[0]
        
        say(response['message'])
        return True, room_num, floor
    return False

def clean_llm_response(response):
    # Remove <|assistant|> tags
    response = re.sub(r'<\|assistant\|>', '', response).strip()
    
    # Check if the response starts with the question & removes question if so 
    if re.match(r'^.*\?\s*\n', response):
        # Split by first line break and take everything after it
        parts = response.split('\n', 1)
        if len(parts) > 1:
            response = parts[1].strip()
    
    return response

def handle_question(stream, rec, sample_rate, chunk_size, llm):
    while True:
        print("Ask question")
        say("What is your question?")
        time.sleep(1)
    
        text = listen_for_text(stream, rec, sample_rate, chunk_size)
        
        if not text:
            continue
            
        if exit_check(text):
            return None 
        
        cleaned_text = clean_text(text)
        response = llm.generate_response(cleaned_text)
        
        # Clean up the response before saying it
        cleaned_response = clean_llm_response(response)
        say(cleaned_response)
        print(response)  # Still print full response for debugging
        
        say("Would you like to ask another question?")
        stream.stop_stream() # Stop audio recording 
        time.sleep(1)
        stream.start_stream() # Restart audio recording 
        
        # Get user's response about asking another question
        follow_up_answer = False
        MAX_ATTEMPTS = 3
        for _ in range(MAX_ATTEMPTS):
            follow_up = listen_for_text(stream, rec, sample_rate, chunk_size)
            
            if not follow_up:
                continue
                
            if exit_check(follow_up):
                return None  # Exit signal
            
            cleaned_follow_up = clean_text(follow_up)
            
            # Check if user wants to ask another question
            if any(word in cleaned_follow_up for word in ["yes", "yeah", "sure", "okay", "yep", "yup"]):
                follow_up_answer = True
                break
            elif any(word in cleaned_follow_up for word in ["no", "nope", "nah"]):
                follow_up_answer = False
                break
            else:
                say("I didn't understand. Do you want to ask another question? Please say yes or no.")

        # If user wants to ask another question, continue loop
        if follow_up_answer:
            continue  # Restart question loop
        else:
            say("Thank you for your questions. Goodbye!")
            return None 
    return None

def main():
    vosk_model_path = "./vosk-model-small-en-us-0.15"
    pyaudio_instance, stream, rec, sample_rate, chunk_size = initialize_vosk_recognizer(vosk_model_path)
    
    try:
        while True: #wake word loop
            print("Please speak...")
            text = listen_for_text(stream, rec, sample_rate, chunk_size)
            
            if not text:
                continue
                
            if exit_check(text):
                break
                
            if wake_word(stream, text):
                # Enter command loop
                while True:
                    print("Please speak...")
                    text = listen_for_text(stream, rec, sample_rate, chunk_size)
                    
                    if not text:
                        continue
                        
                    if exit_check(text):
                        return

                    cleaned_text = clean_text(text)
                    
                    if "navigation" in cleaned_text: 
                        nav_result = handle_navigation(stream, rec, sample_rate, chunk_size, classifier)
                        
                        if nav_result is None:  # Exit word said 
                            return
                        
                        if nav_result and nav_result[0]:  # Successful navigation
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
                            break  # Return to wake word loop
                        
                        if not nav_result:  # Restart after 3 attempts
                            break  # Return to wake word loop
                    
                    elif "question" in cleaned_text:
                        question_result = handle_question(stream, rec, sample_rate, chunk_size, llm)
                        
                        if question_result is None:  # Either user said exit word or doesn't want more questions
                            return  # Exit the program completely
                        
                        break  # Return to wake word loop
                    
                    time.sleep(0.5)
    
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
    finally:
        stream.stop_stream()
        stream.close()
        pyaudio_instance.terminate()

if __name__ == "__main__":
    main()