#third time IS the charm 
import pyttsx3
import time
import string
from RealtimeSTT import AudioToTextRecorder
from nav_classifier import RoomClassifier

classifier = RoomClassifier()
engine = pyttsx3.init()

def clean_text(text):
    cleaned_text = text.strip().lower()
    cleaned_text = cleaned_text.translate(str.maketrans('', '', string.punctuation))
    print(f"User: '{cleaned_text}'")    
    return cleaned_text  

def wake_word(text):
    if text:
        cleaned_text = clean_text(text)
        if "hey tori" in cleaned_text or "hey tory" in cleaned_text:
            engine.say("Hi, I'm Tori, a tour guide robot in Unity Hall. Would you like to say a navigation command or ask me a question?")
            engine.runAndWait()
            time.sleep(0.5)
            return True
    return False

def nav_command(): 
    print("Nav command")
    engine.say("Where do you want to go?")
    engine.runAndWait()
    time.sleep(1) 
    return True

def question(): 
    print("Ask question")
    #insert LLM logic here 
    return True

def num_attempts(recorder, classifier):
    attempts = 0
    MAX_ATTEMPTS = 3 
    
    while attempts < MAX_ATTEMPTS:
        text = recorder.text()
        if exit(text):
            return None
        if not text:
            time.sleep(1)
            continue
            
        cleaned_text = clean_text(text)
        response = classifier.get_navigation_response(cleaned_text)
        
        if response['success']:
            engine.say(response['message'])
            engine.runAndWait()
            print("Location info:", classifier.extract_location_info(cleaned_text))
            return True
        else:
            attempts += 1
            if attempts == MAX_ATTEMPTS: #de-initializes once it reaches max requests 
                message = "I'm still having trouble understanding. Let's start over. Please say the wake phrase when you're ready."
                engine.say(message)
                engine.runAndWait()
                print(message)
                return False
    return False

def exit(text): 
    if text:
        cleaned_text = clean_text(text)
        exit_words = {"stop", "goodbye", "bye"}
        if any(word in cleaned_text.lower() for word in exit_words):
            engine.say("Goodbye!")
            engine.runAndWait()
            return True
    return False


########## MAIN ##########
def main(): 
    recorder = AudioToTextRecorder()  
    
    try:
        while True:  
            print("Listening for the wake phrase...")
            
            while True: # waits for wake word 
                text = recorder.text()
                if exit(text):
                    return
                if wake_word(text):
                    break
            
            while True: # nav command or question 
                text = recorder.text()
                if exit(text):
                    return
                if text:
                    cleaned_text2 = clean_text(text)
                    if "navigation command" in cleaned_text2:
                        nav_command()
                        if num_attempts(recorder, classifier):
                            continue  # insert nav stack code here 
                        break  # restart if unable to understand location 
                    elif "question" in cleaned_text2:
                        question()
                        break  # restart after question is answered 
                time.sleep(0.1) 


    except KeyboardInterrupt:
        print("Keyboard Interrupt")

if __name__ == "__main__":
    main() 


#what if user wants to go to its current location? Does this need to be a case in the code in case Tori is already at localized location? 