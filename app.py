from flask import Flask, render_template, jsonify, request
import paho.mqtt.client as mqtt
import sqlite3
import random
from datetime import datetime

app = Flask(__name__, static_folder='static', static_url_path='/static')

# MQTT Setup
mqtt_broker = "silentprince839.cloud.shiftr.io"
mqtt_port = 1883
mqtt_user = "silentprince839"
mqtt_password = "pOSBE5CfXH1Oaz6z"

# Device states
device_states = {
    "fan": False,
    "light": False
}

# Store latest sensor values
latest_sensor_data = {
    "temperature": None,
    "humidity": None,
    "sound": None,
    "light_intensity": None
}

# SQLite database setup
def init_db():
    with sqlite3.connect('smart_device.db') as conn:
        c = conn.cursor()
        c.execute('''CREATE TABLE IF NOT EXISTS sensor_data (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            temperature REAL,
            humidity REAL,
            timestamp DATETIME DEFAULT CURRENT_TIMESTAMP
        )''')
        c.execute('''CREATE TABLE IF NOT EXISTS device_states (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            device TEXT,
            state TEXT,
            timestamp DATETIME DEFAULT CURRENT_TIMESTAMP
        )''')
        c.execute('''CREATE TABLE IF NOT EXISTS sound_data (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            sound_level REAL,
            timestamp DATETIME DEFAULT CURRENT_TIMESTAMP
        )''')
        c.execute('''CREATE TABLE IF NOT EXISTS light_data (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            light_level REAL,
            timestamp DATETIME DEFAULT CURRENT_TIMESTAMP
        )''')
        c.execute('''CREATE INDEX IF NOT EXISTS idx_sensor_data_timestamp ON sensor_data(timestamp)''')
        c.execute('''CREATE INDEX IF NOT EXISTS idx_sound_data_timestamp ON sound_data(timestamp)''')
        c.execute('''CREATE INDEX IF NOT EXISTS idx_light_data_timestamp ON light_data(timestamp)''')
        conn.commit()

# MQTT callbacks
def on_connect(client, userdata, flags, rc):
    print(f"Connected to MQTT broker with code {rc}")
    client.subscribe([("fan", 0), ("light", 0), ("temperature", 0), ("humidity", 0), ("sound", 0), ("light_intensity", 0)])

def on_message(client, userdata, msg):
    topic = msg.topic
    payload = msg.payload.decode()
    print(f"Received {topic}: {payload}")

    with sqlite3.connect('smart_device.db') as conn:
        c = conn.cursor()
        if topic in ["temperature", "humidity"]:
            try:
                value = float(payload)
                if topic == "temperature":
                    latest_sensor_data["temperature"] = value
                elif topic == "humidity":
                    latest_sensor_data["humidity"] = value
                
                if latest_sensor_data["temperature"] is not None and latest_sensor_data["humidity"] is not None:
                    c.execute("INSERT INTO sensor_data (temperature, humidity) VALUES (?, ?)",
                              (latest_sensor_data["temperature"], latest_sensor_data["humidity"]))
                    print(f"Inserted sensor data: Temp={latest_sensor_data['temperature']}, Hum={latest_sensor_data['humidity']}")
                    latest_sensor_data["temperature"] = None
                    latest_sensor_data["humidity"] = None
            except ValueError:
                print(f"Invalid {topic} value: {payload}")
        elif topic == "sound":
            try:
                value = float(payload)
                latest_sensor_data["sound"] = value
                c.execute("INSERT INTO sound_data (sound_level) VALUES (?)", (value,))
                print(f"Inserted sound data: Sound={value}")
            except ValueError:
                print(f"Invalid sound value: {payload}")
        elif topic == "light_intensity":
            try:
                value = float(payload)
                if value >= 0:  # Validate non-negative light values
                    latest_sensor_data["light_intensity"] = value
                    c.execute("INSERT INTO light_data (light_level) VALUES (?)", (value,))
                    print(f"Inserted light data: Light={value}")
                else:
                    print(f"Invalid negative light value: {payload}")
            except ValueError:
                print(f"Invalid light intensity value: {payload}")
        elif topic in ["fan", "light"]:
            state = payload == "on"
            device_states[topic] = state
            c.execute("INSERT INTO device_states (device, state) VALUES (?, ?)", (topic, payload))
            print(f"Inserted device state: {topic}={payload}")
        conn.commit()

# Initialize MQTT client
client = mqtt.Client(client_id="FlaskServer_" + str(random.randint(0, 0xffff)))
client.username_pw_set(mqtt_user, mqtt_password)
client.on_connect = on_connect
client.on_message = on_message
client.connect(mqtt_broker, mqtt_port, 60)
client.loop_start()

# Initialize database
init_db()

def publish_device_state(device, state):
    message = "on" if state else "off"
    client.publish(device, message)
    print(f"Published {device}: {message}")

@app.route('/')
def index():
    with sqlite3.connect('smart_device.db') as conn:
        c = conn.cursor()
        c.execute("SELECT temperature, humidity FROM sensor_data ORDER BY timestamp DESC LIMIT 1")
        row = c.fetchone()
        c.execute("SELECT sound_level FROM sound_data ORDER BY timestamp DESC LIMIT 1")
        sound_row = c.fetchone()
        c.execute("SELECT light_level FROM light_data ORDER BY timestamp DESC LIMIT 1")
        light_row = c.fetchone()
        sensor_data = {
            'temperature': round(row[0], 2) if row and row[0] is not None else '--',
            'humidity': round(row[1], 2) if row and row[1] is not None else '--',
            'sound': round(sound_row[0], 2) if sound_row and sound_row[0] is not None else '--',
            'light_intensity': round(light_row[0], 2) if light_row and light_row[0] is not None else '--'
        }
    return render_template('index.html', device_states=device_states, sensor_data=sensor_data)

@app.route('/device/<device>/<state>', methods=['POST'])
def control_device(device, state):
    if device in device_states and state in ["on", "off"]:
        device_states[device] = state == "on"
        publish_device_state(device, device_states[device])
        with sqlite3.connect('smart_device.db') as conn:
            c = conn.cursor()
            c.execute("INSERT INTO device_states (device, state) VALUES (?, ?)", (device, state))
            conn.commit()
        return jsonify({'status': 'success', 'device': device, 'state': state})
    else:
        return jsonify({'status': 'error', 'message': 'Invalid device or state'}), 400

@app.route('/sensors')
def get_sensors():
    with sqlite3.connect('smart_device.db') as conn:
        c = conn.cursor()
        c.execute("SELECT temperature, humidity FROM sensor_data ORDER BY timestamp DESC LIMIT 1")
        row = c.fetchone()
        c.execute("SELECT sound_level FROM sound_data ORDER BY timestamp DESC LIMIT 1")
        sound_row = c.fetchone()
        c.execute("SELECT light_level FROM light_data ORDER BY timestamp DESC LIMIT 1")
        light_row = c.fetchone()
        return jsonify({
            'temperature': round(row[0], 2) if row and row[0] is not None else '--',
            'humidity': round(row[1], 2) if row and row[1] is not None else '--',
            'sound': round(sound_row[0], 2) if sound_row and sound_row[0] is not None else '--',
            'light_intensity': round(light_row[0], 2) if light_row and light_row[0] is not None else '--'
        })

@app.route('/sensor_history')
def get_sensor_history():
    with sqlite3.connect('smart_device.db') as conn:
        c = conn.cursor()
        c.execute("SELECT temperature, humidity, timestamp FROM sensor_data ORDER BY timestamp DESC LIMIT 50")
        rows = c.fetchall()
        c.execute("SELECT light_level, timestamp FROM light_data ORDER BY timestamp DESC LIMIT 50")
        light_rows = c.fetchall()
        return jsonify([
            {
                'temperature': row[0] if row[0] is not None else '--',
                'humidity': row[1] if row[1] is not None else '--',
                'light_intensity': light_rows[i][0] if i < len(light_rows) and light_rows[i][0] is not None else '--',
                'timestamp': row[2]
            } for i, row in enumerate(rows)
        ])

@app.route('/sound_history')
def get_sound_history():
    with sqlite3.connect('smart_device.db') as conn:
        c = conn.cursor()
        c.execute("SELECT sound_level, timestamp FROM sound_data ORDER BY timestamp DESC LIMIT 30")
        rows = c.fetchall()
        return jsonify([
            {
                'sound_level': row[0] if row[0] is not None else 0,
                'timestamp': row[1]
            } for row in rows
        ])

@app.route('/light_history')
def get_light_history():
    with sqlite3.connect('smart_device.db') as conn:
        c = conn.cursor()
        c.execute("SELECT light_level, timestamp FROM light_data ORDER BY timestamp DESC LIMIT 30")
        rows = c.fetchall()
        return jsonify([
            {
                'light_level': row[0] if row[0] is not None else 0,
                'timestamp': row[1]
            } for row in rows
        ])

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)