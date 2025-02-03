import speech_recognition as sr
import time
import pyttsx3 
import re
from nav_classifier import RoomClassifier
# from nav_stack import GoalPublisherNode

recognizer = sr.Recognizer()
engine = pyttsx3.init()
classifier = RoomClassifier()

# list of wake phrases and exit words
WAKE_PHRASES = ["hey tori", "hi tori", "hello tori", "hello"]
EXIT_WORDS = ["goodbye", "exit", "bye", "stop"]
NAV_KEYWORDS = ["go to", "navigate to", "take me to", "where is", "need to find", "how do I get", "tell me where"]
#might later incorporate a question of navigation or question instead or if-else logic with nav keywords, whichever works better 

def clean_text(text): # make text lowercase + remove unwanted characters 
    return re.sub(r'[^\w\s]', '', text.lower())

def wake_word():
    with sr.Microphone() as source:
        print("Listening for wake phrase...")

        while True:
            try:
                audio = recognizer.listen(source)
                recognized_text = recognizer.recognize_google(audio)
                print(f"Heard: {recognized_text}")

                cleaned_text = clean_text(recognized_text)

                if any(clean_text(phrase) in cleaned_text for phrase in WAKE_PHRASES):
                    print("Wake phrase detected!")
                    engine.say("How can I assist you?")
                    engine.runAndWait()
                    return True  
                
            except sr.UnknownValueError:
                continue
            except sr.RequestError:
                print("Could not request results from the speech recognition service")
                break

def commands(): #after wake_word() returns true
    with sr.Microphone() as source:
        print("Listening for a command...")

        while True:
            try:
                audio = recognizer.listen(source)
                recognized_text = recognizer.recognize_google(audio)
                print(f"Command received: {recognized_text}")

                cleaned_command = clean_text(recognized_text)

                if any(clean_text(exit_word) in cleaned_command for exit_word in EXIT_WORDS):
                    engine.say("Toodles") 
                    engine.runAndWait()
                    return False 

                if any(keyword in cleaned_command for keyword in NAV_KEYWORDS):
                    print("Nav stack triggered") 
                    # print(recognized_text)
                    result = classifier.get_navigation_response(recognized_text)
                    engine.say(f"{result['message']}")
                    print(classifier.extract_location_info(recognized_text)) #testing purposes
                    engine.runAndWait()
                    #insert nav stack code...

            except sr.UnknownValueError:
                continue
            except sr.RequestError:
                print("Could not request results from the speech recognition service")
                break

def main():
    while True:
        if wake_word():  # once the wake word is detected, proceed to commands
            if not commands():  # if exit word is detected, end the loop
                break

if __name__ == "__main__":
    main()
