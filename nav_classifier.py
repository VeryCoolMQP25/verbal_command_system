import spacy
import re
import json 
import subprocess 

class RoomClassifier:
    def __init__(self):
        # Load English language model
        self.nlp = spacy.load("en_core_web_sm")
        
        # Define room patterns with fixed floor associations
        self.room_patterns = { 
            "elevator": {
                "patterns": ["elevator", "lift"], 
                "fixed_floor": None  
            },
            "stairs": {
                "patterns": ["stairs", "stair"], #weird bug, but each pattern needs at least 2 or it won't work 
                "fixed_floor": None
            },
            "restrooms": {
                "patterns": ["bathroom", "restroom", "toilet", "washroom"],
                "fixed_floor": None
            },
            "curtain area": {
                "patterns": ["curtain area", "curtain", "curtains"],
                "fixed_floor": "2"
            },
            "career development center": {
                "patterns": ["career center", "career services", "career office", "CDC", "career development center", "career area", "heebner CDC", "heebner career development center"],
                "fixed_floor": "5" 
            }, 
            "study lounge": {
                "patterns": ["lounge", "window"],
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
        room_numbers = re.findall(r'\b[1-4][0-9]{2}\b', text)
        
        if not room_numbers:
            return None, None
        
        room_number = room_numbers[0]
        floor_number = room_number[0]
        
        floor_mapping = {
            '1': '1',
            '2': '2',
            '3': '3',
            '4': '4',
            '5': '5'
        }
        
        return room_number, floor_mapping.get(floor_number)

    def preprocess_text(self, text):
        text = text.lower()
        text = re.sub(r'[^\w\s]', ' ', text)
        doc = self.nlp(text)
        return ' '.join([token.lemma_ for token in doc])

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

        with open('Unity_coords.json', 'r') as file:
            json_data = json.load(file)
        floor_key = "floor_" + str(text['floor'][0]) 
        room_key = text['room_number']
        if room_key is None: 
            room_key = text['room']
        # print(floor_key)
        # print(room_key)

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


    def get_navigation_response(self, text):
        new_info = self.extract_location_info(text)
        self.update_context(new_info)
        combined_text = self.get_combined_text(text)        
        info = self.extract_location_info(combined_text)
        json = self.json(info)
        print(f"flag: {json}")

        if info['room_number']: 
            if json == 1: 
                return {
                    'success': True,
                    'message': f"I'll take you to room {info['room_number']} on floor number {info['floor']}.",
                    'missing': None
                }
            else: 
                return{
                    'success': False,
                    'message': "That room does not exist. Please try again.",
                    'missing': 'both'
                }
        
        if info['room'] is None:
            if info['floor'] is None:
                return {
                    'success': False,
                    'message': "I couldn't understand which room or floor you're looking for. Please be more specific.",
                    'missing': 'both'
                }
            else:
                return {
                    'success': False,
                    'message': "I couldn't understand which room you're looking for. Please be more specific.",
                    'missing': 'room'
                }
        
        if info['floor'] is None:
            room_desc = self.context['room_number'] if self.context['room_number'] else self.context['room']
            return {
                'success': False,
                'message': f"I couldn't understand which floor the {room_desc} is on. Please specify the floor.",
                'missing': 'floor'
            }
        
if __name__ == "__main__":
    classifier = RoomClassifier()
    # text = {'room': 'room', 'room_number': '300', 'floor': 'third'}
    # test = classifier.json()


    test_phrases = [
        "How do I get to the career center on the fifth floor?",  
        # "Where's the elevator on the third floor?",  
        # "I need to find the career center on the third floor",  #will correct to fifth floor 
        # "Can you show me where the conference room is on the second floor?",  
        # "Navigate to the curtain area",  #will automatically use second floor 
        # "Navigate to the restrooms", #unknown floor 
        # "Can you show me where room 156 is?",
        # "Navigate to room 425.", 
        # "Take me to tech suite 316 please.", 
        # "Take me to blah blah blah", #doesn't exist 
        "Take me to the pear lab on the second floor", 
        "Take me to the stairs on the second floor"
        # "Take me to room 301"
    ]
    
    for phrase in test_phrases:
        print(classifier.extract_location_info(phrase))
        result = classifier.get_navigation_response(phrase)
        print(f"\nInput: {phrase}")
        # print(f"Response: {result['message']}")
