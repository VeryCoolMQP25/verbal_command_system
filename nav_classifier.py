import spacy
import re

class RoomClassifier:
    def __init__(self):
        # Load English language model
        self.nlp = spacy.load("en_core_web_sm")
        
        # Define room patterns with fixed floor associations
        self.room_patterns = {
            "elevator": {
                "patterns": ["elevator", "lift"],
                "fixed_floor": None  # any floor
            },
            "stairs": {
                "patterns": ["stairs"],
                "fixed_floor": None
            },
            "restrooms": {
                "patterns": ["bathroom", "restroom", "toilet", "washroom"],
                "fixed_floor": None
            },
            "curtain area": {
                "patterns": ["curtain area", "curtain", "curtains"],
                "fixed_floor": "second"
            },
            "career_center": {
                "patterns": ["career center", "career services", "career office", "CDC", "career development center", "career area", "heebner CDC", "heebner career development center"],
                "fixed_floor": "fifth" 
            }, 
            "lounge": {
                "patterns": ["lounge", "window"],
                "fixed_floor": "None" 
            }
        }
        
        #still trying to figure out how to organize this, especially if they are on multiple floors but not all floors...:  
        # "classroom": ["classroom", "lecture room", "room", "lecture hall", "unity", "UH", "you ache"],
        # "study area": ["study area", "study corner", "study tables", "study pods"], 
        # "tech suites": ["tech suites"], 
        # "Unity 100": ["Unity Hall 100", "Unity 100", "UH 100", "you ache 100", "mqp lab"], 
        # "Unity 105": ["Unity Hall 105", "Unity 105", "UH 105", "you ache 105", "pear lab", "soft robotics lab"], 
        # "offices": ["offices", "professor's offices"], 


        # Define floor patterns
        self.floor_patterns = {
            "first": ["first floor", "1st floor", "floor 1", "floor one", "bottom floor", "ground floor", "first level"],
            "second": ["second floor", "2nd floor", "floor 2", "floor second", "second level"],
            "third": ["third floor", "3rd floor", "floor 3", "floor three", "middle floor", "third level"],
            "fourth": ["fourth floor", "4th floor", "floor 4", "floor four", "fourth level"], 
            "fifth": ["fifth floor", "5th floor", "floor 5", "floor five", "top floor", "fifth level"]
        }

    def extract_room_number(self, text):
        """Extract room number from text and determine floor"""
        room_numbers = re.findall(r'\b[1-4][0-9]{2}\b', text)
        
        if not room_numbers:
            return None, None
        
        room_number = room_numbers[0]
        floor_number = room_number[0]
        
        floor_mapping = {
            '1': 'first',
            '2': 'second',
            '3': 'third',
            '4': 'fourth',
            '5': 'fifth'
        }
        
        return room_number, floor_mapping.get(floor_number)

    #uses natural language processing 
    def preprocess_text(self, text):
        text = text.lower()
        text = re.sub(r'[^\w\s]', ' ', text)
        doc = self.nlp(text)
        return ' '.join([token.lemma_ for token in doc])

    #extract room/floor information 
    def extract_location_info(self, text):
        processed_text = self.preprocess_text(text)
        
        #check for room number first 
        room_number, floor_from_number = self.extract_room_number(processed_text)
        if room_number:
            return {
                'room': 'room',
                'room_number': room_number,
                'floor': floor_from_number
            }
        
        #if no room number, find room type
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
        
        #if no room number, find floor (if not fixed)
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

    def get_navigation_response(self, text):
        info = self.extract_location_info(text)
        
        if info['room_number']:
            return {
                'success': True,
                'message': f"I'll take you to room {info['room_number']} on the {info['floor']} floor.",
            }
        
        if info['room'] is None:
            if info['floor'] is None: # if room and floor don't exist 
                return {
                'success': False,
                'message': "I couldn't understand which room or floor you're looking for. Please be more specific.",
            }
            else: #if just room doesn't exist 
                return {
                    'success': False,
                    'message': "I couldn't understand which room you're looking for. Please be more specific.",
                }
        
        if info['floor'] is None:
            return {
                'success': False,
                'message': "I couldn't understand which floor you're looking for. Please be more specific.",
            }
        
        response = f"I'll take you to the {info['room'].replace('_', ' ')} "
        
        if info['fixed_floor'] and info['floor'] != info['fixed_floor']:
            response += f"which is located on the {info['fixed_floor']} floor. "
            if info['floor']:
                response += f"(Note: The {info['room'].replace('_', ' ')} is only on the {info['fixed_floor']} floor, "
                response += f"not the {info['floor']} floor.) "
        else:
            response += f"on the {info['floor']} floor. "
        
        return {
            'success': True,
            'message': response,
        }
    
if __name__ == "__main__":
    classifier = RoomClassifier()
    
    test_phrases = [
        "How do I get to the career center on the fifth floor?",  
        "Where's the elevator on the third floor?",  
        "I need to find the career center on the third floor",  #will correct to fifth floor 
        "Can you show me where the conference room is on the second floor?",  
        "Navigate to the curtain area",  #will automatically use second floor 
        "Navigate to the restrooms", #unknown floor 
        "Can you show me where room 156 is?",
        "Navigate to room 425", 
        "Take me to tech suite 316", 
        "Take me to blah blah blah" #doesn't exist 
    ]
    
    for phrase in test_phrases:
        result = classifier.get_navigation_response(phrase)
        print(f"\nInput: {phrase}")
        print(f"Response: {result['message']}")
