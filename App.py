#!/usr/bin/python3
from flask import Flask, jsonify
from flask_cors import CORS
import subprocess
import json
import os
import logging
from vosk2 import main as vosk
from multiprocessing import Process, Queue
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)
app = Flask(__name__)
CORS(app)
messageQ = Queue()

@app.route('/api/run-script', methods=['GET'])
def run_script():
    print("Sending wake to messageQ")
    messageQ.put("hey tori")
    return jsonify({
        "message": "Wake invoked",
        "output": "No active navigation"
    })


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
    print("Starting...")
    voskInstance = Process(target=vosk, args=[messageQ])
    voskInstance.start()
    app.run(debug=False)
