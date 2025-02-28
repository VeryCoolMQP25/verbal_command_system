import pyttsx3
import time
import string
import subprocess
from RealtimeSTT import AudioToTextRecorder
from websocket_server import WebsocketServer
import json
from nav_classifier import RoomClassifier
from llm_rag import LLM_RAG
import asyncio
import websockets

classifier = RoomClassifier()
llm = LLM_RAG()
engine = pyttsx3.init()

room_num = None #global variable 

async def send_room_and_floor(room_number, floor_number):
    print(f"Sending room number: {room_number}, floor number: {floor_number}")
    uri = "ws://localhost:8765"  # WebSocket server URI
    data = {
        "room_number": room_number,
        "floor_number": floor_number
    }
    
    # Convert the dictionary to a JSON string
    json_data = json.dumps(data)
    print(json_data)
    
    # Connect to the WebSocket server
    async with websockets.connect(uri) as websocket:
        await websocket.send(json_data)  # Send the JSON data to the WebSocket server
        print(f"Sent room number: {room_number} and floor number: {floor_number}")

def clean_text(text):
    cleaned_text = text.strip().lower()
    cleaned_text = cleaned_text.translate(str.maketrans('', '', string.punctuation))
    print(f"User: '{cleaned_text}'")    
    return cleaned_text  

def wake_word(text):
    if text:
        cleaned_text = clean_text(text)
        if "hey tori" in cleaned_text or "hey tory" in cleaned_text or "hey torry" in cleaned_text:
            engine.say("Hi, I'm Tori, a tour guide robot in Unity Hall. Would you like to say a navigation command or ask me a question?")
            engine.runAndWait()
            time.sleep(7)
            return True
    return False

def handle_navigation(recorder, classifier):
    engine.say("Where do you want to go?")
    engine.runAndWait()
    time.sleep(2)
    
    attempts = 0
    MAX_ATTEMPTS = 3
    
    while attempts < MAX_ATTEMPTS:
        text = recorder.text()
        if exit(text):
            return None
            
        if text:
            cleaned_text = clean_text(text)
            response = classifier.get_navigation_response(cleaned_text)
            
            if response['success']:
                engine.say(response['message'])
                engine.runAndWait()
                print("Location info:", classifier.extract_location_info(cleaned_text))
                var = classifier.extract_location_info(cleaned_text) 
                room_num = var['room_number']
                print(room_num)
                if room_num is None: 
                    room_num = var['room']
                floor = var['floor']
                return True, room_num, floor
            
            attempts += 1
            if attempts < MAX_ATTEMPTS:
                if response['missing'] == 'both':
                    prompt = "I need both a room and floor. Please specify where you want to go."
                elif response['missing'] == 'room':
                    prompt = "Which room are you looking for?"
                elif response['missing'] == 'floor':
                    prompt = "Which floor is that on?"
                else:
                    prompt = "I didn't understand. Please try again."
                
                engine.say(prompt)
                engine.runAndWait()
            else:
                engine.say("I'm still having trouble understanding. Let's start over. Please say the wake phrase when you're ready.")
                engine.runAndWait()
                classifier.reset_context() #reset context for next run 
                return False 
    return False

def question(recorder): 
    print("Ask question")
    engine.say("What is your question?")
    engine.runAndWait()
    time.sleep(1)

    text = recorder.text()
    if exit(text):
        return None
    if text:
        cleaned_text = clean_text(text)
        response = cleaned_text
    response = llm.generate_response(cleaned_text) 
    engine.say(response)
    engine.runAndWait()
    print(response)
    return True 

def exit(text): 
    if text:
        cleaned_text = clean_text(text)
        exit_words = {"stop", "goodbye", "bye"}
        if any(word in cleaned_text.lower() for word in exit_words):
            engine.say("Goodbye!")
            engine.runAndWait()
            return True
    return False

def main(): 
    recorder = AudioToTextRecorder()
    classifier = RoomClassifier()
    
    try:
        while True:
            print("Listening for the wake phrase...")
            engine.say("Please say a wake word when you are ready")
            engine.runAndWait()
            time.sleep(1)
            
            while True:  # wake word loop
                text = recorder.text()
                if exit(text):
                    return
                if wake_word(text):
                    break
            
            while True: #command loop 
                text = recorder.text()
                if exit(text):
                    return
                if text:
                    cleaned_text = clean_text(text)
                    if "navigation" in cleaned_text:
                        nav_result, room_num, floor = handle_navigation(recorder, classifier)
                        if exit(text):
                            return
                        if nav_result:  # successful navigation 
                            print("get the location to the gui somehow")
                            print(floor)
                            print(room_num)
                            asyncio.run(send_room_and_floor(room_num, floor))
                            classifier.reset_context()  # reset context after navigation
                            return
                        if not nav_result:  # restart after 3 attempts
                            break 
                    elif "question" in cleaned_text:
                        question(recorder)
                        return
                time.sleep(1)

    except KeyboardInterrupt:
        print("Keyboard Interrupt")

if __name__ == "__main__":
    main()
    # asyncio.run(send_room_and_floor("UH400", "4"))
    # print("server running")

#what if user wants to go to its current location? Does this need to be a case in the code in case Tori is already at localized location? 

#if no audio for 10 seconds, it will turn off functionality needed 
