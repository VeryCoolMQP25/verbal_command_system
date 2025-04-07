import time
import string
import json
import vosk
import pyaudio
import rclpy
from rclpy.node import Node
import logging
from nav_classifier import RoomClassifier
from llm_rag import LLM_RAG
from navigation_stack import NavigationNode
from goal_proximity import GoalProximityNode
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

def exit_check(text, stream):
    if text:
        cleaned_text = clean_text(text)
        exit_words = {"stop", "goodbye", "bye", "exit", "quit", "end"}
        
        if any(word in cleaned_text.lower() for word in exit_words):
            stream.stop_stream()  # Stop stream before robot speaks
            say("Goodbye! Say the wake phrase when you want to talk again.")
            # time.sleep(0.5)
            stream.start_stream()  # Restart stream to listen for user
            time.sleep(0.5)
            return True
    return False


def wake_word(stream, text):
    if text:
        cleaned_text = clean_text(text)
        wake_phrases = ["hey tori", "hey tory", "hey torry", "hitori", "katori"]
        
        if any(phrase in cleaned_text for phrase in wake_phrases):
            stream.stop_stream() # Stop audio recording (so speech to text lib does not pick up Tori's words)
            say("Hi, I'm Tori, a tour guide robot in Unity Hall. Would you like to say a navigation command or ask me a question?")
            stream.start_stream() # Restart audio recording 
            return True
    return False

def convert_number_words(sentence):
    try:
        return t2d.convert(sentence)
    except Exception as e:
        return f"Error: {e}"

def handle_navigation(stream, rec, sample_rate, chunk_size, classifier):
    stream.stop_stream()
    say("Where do you want to go?")
    # time.sleep(1)
    stream.start_stream()
    
    MAX_ATTEMPTS = 3
    for attempts in range(MAX_ATTEMPTS):
        text = listen_for_text(stream, rec, sample_rate, chunk_size)
        
        if not text:
            continue
            
        if exit_check(text, stream):
            return False  # Return to wake word loop
        
        cleaned_text = clean_text(text)
        cleaned_text2 = convert_number_words(cleaned_text)
        print(f"Original: '{cleaned_text}', Converted: '{cleaned_text2}'")
        
        # Get navigation response which will update and use the context
        response = classifier.get_navigation_response(cleaned_text2)
        print("Navigation Response:", response)
        
        if not response['success']:
            stream.stop_stream()
            say(response['message'])
            time.sleep(0.5)
            stream.start_stream()
            
            # If last attempt, reset context
            if attempts == MAX_ATTEMPTS - 1:
                classifier.reset_context()
                stream.stop_stream()
                say("Let's start over. Please say the wake phrase when you're ready.")
                time.sleep(0.5)
                stream.start_stream()
                return False
            continue
        
        # If navigation is successful, extract the room and floor information
        room_info = classifier.extract_location_info(cleaned_text2)
        room_num = room_info.get('room_number') or room_info.get('room') or classifier.context.get('room')
        floor = room_info.get('floor') or classifier.context.get('floor')
        
        # Ensure floor is a string
        if isinstance(floor, list):
            floor = floor[0]
        
        stream.stop_stream()
        say(response['message'])
        time.sleep(0.5)
        stream.start_stream()
        return True, room_num, floor
    
    return False


def clean_llm_response(response):
    # Split the response into lines
    lines = response.split('\n')
    
    # Filter out lines with the assistant tag
    cleaned_lines = [line for line in lines if '<|assistant|>' not in line]
    
    # Take only the first line (typically the direct answer)
    if cleaned_lines:
        cleaned_response = cleaned_lines[0].strip()
        return cleaned_response
    
    # Fallback if no lines remain
    return response.strip()

def handle_question(stream, rec, sample_rate, chunk_size, llm):
    while True:
        print("Ask question")
        stream.stop_stream()  # Stop stream before robot speaks
        say("What is your question?")
        # time.sleep(1)
        stream.start_stream()  # Start stream to listen for user's question
    
        text = listen_for_text(stream, rec, sample_rate, chunk_size)
        
        if not text:
            continue
            
        if exit_check(text, stream):
            return False  # Return to wake word loop
        
        cleaned_text = clean_text(text)
        response = llm.generate_response(cleaned_text)
        
        # Clean up the response before saying it
        cleaned_response = clean_llm_response(response)
        stream.stop_stream()  # Stop stream before robot speaks
        say(cleaned_response)
        print(response)  # Still print full response for debugging
        time.sleep(2)
        say("Would you like to ask another question?")
        # time.sleep(1)
        stream.start_stream()  # Restart stream to listen for user's response
        
        # Get user's response about asking another question
        follow_up_answer = False
        MAX_ATTEMPTS = 3
        for _ in range(MAX_ATTEMPTS):
            follow_up = listen_for_text(stream, rec, sample_rate, chunk_size)
            
            if not follow_up:
                continue
                
            if exit_check(follow_up, stream):
                return False  # Return to wake word loop
            
            cleaned_follow_up = clean_text(follow_up)
            
            # Check if user wants to ask another question
            if any(word in cleaned_follow_up for word in ["yes", "yeah", "sure", "okay", "yep", "yup"]):
                follow_up_answer = True
                break
            elif any(word in cleaned_follow_up for word in ["no", "nope", "nah"]):
                follow_up_answer = False
                break
            else:
                stream.stop_stream()  # Stop stream before robot speaks
                say("I didn't understand. Do you want to ask another question?")
                # time.sleep(0.5)
                stream.start_stream()  # Restart stream to listen for user
                time.sleep(0.5)

        # If user wants to ask another question, continue loop
        if follow_up_answer:
            continue  # Restart question loop
        else:
            stream.stop_stream()  # Stop stream before robot speaks
            say("Thank you for your questions. Please say the wake phrase when you need me again!")
            # time.sleep(0.5)
            stream.start_stream()  # Restart stream to listen for user
            time.sleep(0.5)
            return False  # Return to wake word loop
    
    return False  # Return to wake word loop

def main():
    vosk_model_path = "./vosk-model-small-en-us-0.15"
    pyaudio_instance, stream, rec, sample_rate, chunk_size = initialize_vosk_recognizer(vosk_model_path)
    
    try:
        while True: # Main wake word loop
            print("Listening for wake word...")
            text = listen_for_text(stream, rec, sample_rate, chunk_size)
            
            if not text:
                continue
                
            if exit_check(text, stream):
                continue  # Return to wake word loop
                
            if wake_word(stream, text):
                # Enter command loop
                while True:
                    print("Waiting for command...")
                    text = listen_for_text(stream, rec, sample_rate, chunk_size)
                    
                    if not text:
                        continue
                        
                    if exit_check(text, stream):
                        break  # Return to wake word loop

                    cleaned_text = clean_text(text)
                    
                    if "navigation" in cleaned_text: 
                        nav_result = handle_navigation(stream, rec, sample_rate, chunk_size, classifier)
                        
                        if nav_result is False:  # Exit word said or navigation failed
                            break  # Return to wake word loop
                        
                        if nav_result and nav_result[0]:  # Successful navigation
                            room_num, floor = nav_result[1], nav_result[2]
                            
                            if not rclpy.ok():
                                rclpy.init(args=None)
                    
                            nav_stack = NavigationNode()
                            proximity_node = None 
                            
                            with open('current_navigation.json', 'w') as f:
                                json.dump({
                                    "room": room_num, 
                                    "floor": floor, 
                                    "timestamp": time.time()
                                }, f)
                            
                            try:
                                stream.stop_stream()
                                say(f"Starting navigation to room {room_num} on floor {floor}.")
                                stream.start_stream()
                                success = nav_stack.navigate(room_num, floor)
                                
                                if success:
                                    proximity_node = GoalProximityNode(audio_stream=stream)
                                    
                                    max_wait_time = 120  # Maximum time to wait in seconds
                                    start_time = time.time()
                                    
                                    # Process messages until arrival or timeout
                                    while (not proximity_node.arrived) and (time.time() - start_time < max_wait_time):
                                        rclpy.spin_once(proximity_node, timeout_sec=0.1)
                                        time.sleep(0.1)
                                    
                                    if proximity_node.arrived:
                                        time.sleep(2)
                                        stream.stop_stream()
                                        say("We have arrived at your destination!")
                                        stream.start_stream()
                                    else:
                                        stream.stop_stream()
                                        say("Please wait for the robot to reach your destination.")
                                        stream.start_stream()
                                    
                                else:
                                    stream.stop_stream()
                                    say("Navigation failed. Please try again.")
                                    stream.start_stream()
                                    nav_stack.get_logger().error("Navigation failed!")
                            except Exception as e:
                                stream.stop_stream()
                                say(f"Error during navigation: {str(e)}")
                                stream.start_stream()
                                print(f"Navigation error: {e}")
                            finally:
                                # Cleanup
                                if rclpy.ok():
                                    # Safety check to ensure nav_stack exists before destroying it 
                                    if 'nav_stack' in locals() and nav_stack is not None:
                                        nav_stack.destroy_node()
                                    if proximity_node is not None:
                                        proximity_node.destroy_node()
                                    rclpy.shutdown()
                                
                                classifier.reset_context()
                                stream.stop_stream()
                                say("Please say the wake phrase when you need me again!")
                                stream.start_stream()
                            break
                    
                    elif "question" in cleaned_text:
                        # Handle questions and return to wake word loop afterward
                        handle_question(stream, rec, sample_rate, chunk_size, llm)
                        break
                    else:
                        stream.stop_stream()
                        say("I didn't understand. Please say 'navigation command' or 'question'.")
                        stream.start_stream()
                    time.sleep(0.5)
    
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
    finally: # Clean shutdown 
        stream.stop_stream()
        stream.close()
        pyaudio_instance.terminate()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()