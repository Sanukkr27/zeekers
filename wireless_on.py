from flask import Flask, render_template, request, jsonify
import RPi.GPIO as GPIO
import time

app = Flask(__name__)

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO_PIN = 17  # Change to your desired GPIO pin
GPIO.setup(GPIO_PIN, GPIO.OUT)

@app.route('/')
def index():
    return render_template('index.html')
    
@app.route('/toggle', methods=['POST'])
def toggle():
    print("Received data:", request.form)  # Debugging line
    state = request.form.get('state')  

    if state is None:
        return jsonify({"error": "Missing 'state' parameter"}), 400  

    if state == 'on':
            GPIO.output(GPIO_PIN, GPIO.HIGH)
    else:
        GPIO.output(GPIO_PIN, GPIO.LOW)

    return jsonify({"message": f"LED turned {state}"}), 200

    return jsonify({"message": f"LED turned {state}"}), 200  # JSON response

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)

