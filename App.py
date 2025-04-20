from flask import Flask, jsonify
from flask_cors import CORS
import subprocess
import json
import os
import logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)
app = Flask(__name__)
CORS(app)

@app.route('/api/run-script', methods=['GET'])
def run_script():
    try:
        subprocess.run(['python3', 'script.py'], check=True)
        
        # Check if a navigation has been started
        if os.path.exists('current_navigation.json'):
            with open('current_navigation.json', 'r') as f:
                navigation_data = json.load(f)
                
            return jsonify({
                "message": "Script executed successfully!",
                "output": f"Navigating to {navigation_data['room']} on floor {navigation_data['floor']}",
                "room": navigation_data['room'],
                "floor": navigation_data['floor']
            })
        else:
            return jsonify({
                "message": "Script executed, but no navigation data found",
                "output": "No active navigation"
            })
            
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route('/api/navigation-status', methods=['GET'])
def navigation_status():
    if os.path.exists('current_navigation.json'):
        with open('current_navigation.json', 'r') as f:
            navigation_data = json.load(f)
            
        return jsonify({
            "status": "active",
            "room": navigation_data['room'],
            "floor": navigation_data['floor'],
            "started_at": navigation_data['timestamp']
        })
    else:
        return jsonify({
            "status": "inactive",
            "message": "No active navigation"
        })

if __name__ == '__main__':
    app.run(debug=False)
