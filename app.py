from ast import JoinedStr
import json
from pickle import TRUE
import os
from flask import Flask, request, jsonify, render_template
from flask_sqlalchemy import SQLAlchemy
import datetime

app = Flask(__name__)
db_path = os.getenv('GPS_DB_PATH', 'gps_data.db')
app.config['SQLALCHEMY_DATABASE_URI'] = f"sqlite:///{db_path}"
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False
db = SQLAlchemy(app)

class GPSData(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    latitude = db.Column(db.Float, nullable=False)
    longitude = db.Column(db.Float, nullable=False)
    altitude = db.Column(db.Float, nullable=False)
    speed = db.Column(db.Float, nullable=False)
    satellites = db.Column(db.Integer, nullable=False)
    timestamp = db.Column(db.DateTime, default=datetime.datetime.now())

    def as_dict(self):
        return {
            "id": self.id,
            "latitude": self.latitude,
            "longitude": self.longitude,
            "altitude": self.altitude,
            "speed": self.speed,
            "satellites": self.satellites,
            "timestamp": self.timestamp.isoformat()
        }

with app.app_context():
    db.create_all()

@app.route('/')
def home():
    try:
        GPSData.query.delete()
        db.session.commit()
    except Exception as e:
        db.session.rollback()
    return render_template('home.html')

@app.route('/gps', methods=['POST'])
def receive_gps():
    data = request.get_json()
    if not data:
        return jsonify({"error": "No JSON sent"}), 400
    
    try:
        gps_entry = GPSData(
            latitude=data["latitude"],
            longitude=data["longitude"],
            altitude=data["altitude"],
            speed=data["speed"],
            satellites=data["satellites"]
        )
        db.session.add(gps_entry)
        db.session.commit()
        return jsonify({"status": "success", "data": gps_entry.as_dict()})
    except KeyError as e:
        return jsonify({"error": f"Missing field {e}"}), 400
    
@app.route('/gps/latest', methods=['GET'])
def get_latest_gps():
    latest = GPSData.query.order_by(GPSData.timestamp.desc()).first()
    if latest:
        return jsonify(latest.as_dict())
    else:
        return jsonify({"error": "No data found!"}), 404
    
@app.route('/gps/all', methods=['GET'])
def get_all_gps():
    all_data = GPSData.query.order_by(GPSData.timestamp.desc()).all()
    return jsonify([d.as_dict() for d in all_data])

if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True)