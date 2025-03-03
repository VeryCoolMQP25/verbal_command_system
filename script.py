import time
import subprocess
import os 

def run_other_script():
    if os.path.exists('current_navigation.json'):
        os.remove('current_navigation.json')
    try:
        subprocess.run(['python3', 'realtimestt3.py'], check=True)
        print("Script executed successfully!")
    except subprocess.CalledProcessError as e:
        print(f"An error occurred while running the script: {e}")

if __name__ == "__main__":
    run_other_script()