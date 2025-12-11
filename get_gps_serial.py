import serial
import requests

BAUDRATE = 9600
COM_PORT = 'COM3'
SERVER_URL = "https://gps-tracker-backend-fqq3.onrender.com/"

def main():
    ser = serial.Serial(COM_PORT, BAUDRATE)

    while True:
        try: 
            line = ser.readline().decode().strip()
            if not line:
                continue

            lat, long, alt, speed, satellites = line.split(',')

            data = {
                "latitude": lat,
                "longitude": long,
                "altitude": alt,
                "speed": speed,
                "satellites": satellites
            }

            response = requests.post(SERVER_URL, json=data)
        except Exception as e:
            print("Error:", e)