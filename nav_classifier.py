import nltk
import re
import json 
import os
from nltk.tokenize import word_tokenize
from nltk.stem import WordNetLemmatizer

class RoomClassifier:
    def __init__(self):
        # Initialize the lemmatizer from NLTK
        self.lemmatizer = WordNetLemmatizer()
        
        # Define room patterns with fixed floor associations
        self.room_patterns = { 
            "Elevator": {
                "patterns": ["elevator", "lift"], 
                "fixed_floor": None  
            },
            "Stairs": {
                "patterns": ["stairs", "stair"], 
                "fixed_floor": None
            },
            "Restrooms": {
                "patterns": ["bathroom", "restroom", "toilet", "washroom"],
                "fixed_floor": None
            },
            "curtain area": {
                "patterns": ["curtain area", "curtain", "curtains"],
                "fixed_floor": "2"
            },
            "Career_Development_Center": {
                "patterns": ["career center", "career services", "career office", "CDC", "career development center", "career area", "heebner CDC", "heebner career development center"],
                "fixed_floor": "5" 
            }, 
            "Study_Lounge": {
                "patterns": ["lounge", "window"],
                "fixed_floor": "None" 
            }, 
            "Study_Area": {
                "patterns": ["study", "study area"],
                "fixed_floor": "None" 
            }, 
            "UH100": {
                "patterns": ["pear lab", "soft robotics lab"],
                "fixed_floor": "1" 
            }
        }

        # Define floor patterns
        self.floor_patterns = {
            "1": ["first floor", "1st floor", "floor 1", "floor one", "bottom floor", "ground floor", "first level"],
            "2": ["second floor", "2nd floor", "floor 2", "floor second", "second level"],
            "3": ["third floor", "3rd floor", "floor 3", "floor three", "middle floor", "third level"],
            "4": ["fourth floor", "4th floor", "floor 4", "floor four", "fourth level"], 
            "5": ["fifth floor", "5th floor", "floor 5", "floor five", "top floor", "fifth level", "fifth"]
        }

        self.reset_context()

    def reset_context(self): #reset stored context
        self.context = {
            'room': None,
            'room_number': None,
            'floor': None
        }

    def extract_room_number(self, text): #determine floor based on room number
        room_numbers = re.findall(r'\b[1-5][0-9]{2}\b', text)
        
        if not room_numbers:
            return None, None
        
        room_number = room_numbers[0]
        floor_number = room_number[0]
        
        return room_number, floor_number

    def preprocess_text(self, text):
        text = text.lower()
        text = re.sub(r'[^\w\s]', ' ', text)
        
        # Tokenize the text using NLTK
        tokens = word_tokenize(text)
        
        # Lemmatize each token
        lemmatized_tokens = []
        for token in tokens:
            try:
                # Try to lemmatize 
                lemmatized_token = self.lemmatizer.lemmatize(token)
                lemmatized_tokens.append(lemmatized_token)
            except:
                # Fallback if lemmatization fails
                lemmatized_tokens.append(token)
        
        return ' '.join(lemmatized_tokens)

    def update_context(self, new_info):
        for key in self.context:
            if new_info[key] is not None:
                self.context[key] = new_info[key]

    def get_combined_text(self, text):
        combined = text
        if self.context['room'] and "room" not in text.lower():
            combined = f"{self.context['room']} {combined}"
        if self.context['room_number'] and not any(char.isdigit() for char in text):
            combined = f"room {self.context['room_number']} {combined}"
        return combined

    def extract_location_info(self, text): # determine room/floor based on input 
        processed_text = self.preprocess_text(text)
        
        room_number, floor_from_number = self.extract_room_number(processed_text)
        if room_number:
            return {
                'room': 'room',
                'room_number': f"UH{room_number}",
                'floor': floor_from_number
            }
        
        room_type = None
        fixed_floor = None
        
        for room, room_info in self.room_patterns.items():
            for pattern in room_info["patterns"]:
                if pattern in processed_text:
                    room_type = room
                    fixed_floor = room_info["fixed_floor"]
                    break
            if room_type:
                break
        
        floor_type = fixed_floor
        
        if not fixed_floor:
            for floor, variants in self.floor_patterns.items():
                for variant in variants:
                    if variant in processed_text:
                        floor_type = floor
                        break
                if floor_type:
                    break
        
        return {
            'room': room_type,
            'room_number': None,
            'floor': floor_type,
            'fixed_floor': fixed_floor
        }

    def json(self, text):
        flag = 0

        if not text or text.get('floor') is None:
            print("No floor information provided")
            return flag

        json_path = os.path.join(os.path.dirname(__file__), '../public/Unity_coords.json')
        with open(json_path, 'r') as file:
            json_data = json.load(file)
        
        floor = text['floor']
        if isinstance(floor, list):
            floor = floor[0]
        
        floor_key = f"floor_{floor}"
        
        room_key = text.get('room_number') or text.get('room')
        
        if not room_key:
            print("No room information provided")
            return flag

        print(f"Checking floor: {floor_key}, Room: {room_key}")

        if floor_key in json_data:
            print(f"{floor_key} exists in JS data")
            
            if room_key in json_data[floor_key]:
                print(f"Room {room_key} found! Data matches.")
                flag = 1
                return flag
            else:
                print(f"Room {room_key} not found in {floor_key}.")
        else:
            print(f"{floor_key} not found in JS data.")
        
        return flag

    def get_navigation_response(self, text):
        new_info = self.extract_location_info(text)
        self.update_context(new_info)
        combined_text = self.get_combined_text(text)        
        info = self.extract_location_info(combined_text)
        print(info)
        
        # if room or floor is None 
        if info['room'] is None and info['room_number'] is None:
            return {
                'success': False,
                'message': "I couldn't understand which room you're looking for. Could you specify a room number or name?",
                'missing': 'room'
            }
        
        if info['floor'] is None:
            return {
                'success': False,
                'message': "I couldn't understand which floor the room is on. Could you specify a floor?",
                'missing': 'floor'
            }
        
        # if both room and floor are present
        try:
            json_result = self.json(info)
            
            if info['room_number']: 
                if json_result == 1: 
                    return {
                        'success': True,
                        'message': f"I'll take you to room {info['room_number']} on floor number {info['floor']}.",
                        'missing': None
                    }
                else: 
                    return {
                        'success': False,
                        'message': "That room does not exist. Please try again.",
                        'missing': 'both'
                    }
            
            if info['room']: 
                if json_result == 1: 
                    return {
                        'success': True,
                        'message': f"I'll take you to {info['room']} on floor number {info['floor']}.",
                        'missing': None
                    }
                else: 
                    return {
                        'success': False,
                        'message': "That room does not exist. Please try again.",
                        'missing': 'both'
                    }
        
        except Exception as e:
            print(f"Error in navigation response: {e}")
            return {
                'success': False,
                'message': "I'm having trouble processing your request. Could you please try again?",
                'missing': 'both'
            }

if __name__ == "__main__":
    classifier = RoomClassifier()
    # text = {'room': 'room', 'room_number': '300', 'floor': '3'}
    # test = classifier.json()


    test_phrases = [
        "How do I get to the career center on the fifth floor?",  
        # "Where's the elevator on the third floor?",  
        # "I need to find the career center on the third floor",  #will correct to fifth floor 
        # "Can you show me where the conference room is on the second floor?",  
        # "Navigate to the curtain area",  #will automatically use second floor 
        # "Navigate to the restrooms", #unknown floor 
        # "Can you show me where room 156 is?",
        "Navigate to unity 500.", #buggy, need to rdebug 5th floor 
        # "Take me to tech suite 316 please.", 
        # "Take me to blah blah blah", #doesn't exist 
        "Take me to the pear lab on the second floor", 
        "Take me to the stairs on the fifth floor"
        # "Take me to room 301"
    ]
    
    for phrase in test_phrases:
        print(classifier.extract_location_info(phrase))
        result = classifier.get_navigation_response(phrase)
        print(f"\nInput: {phrase}")
        # print(f"Response: {result['message']}")