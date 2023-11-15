from flask import Flask, render_template, url_for, request
from flask_socketio import SocketIO
import serial
import sys
import threading
import re
import time
app = Flask(__name__)
socketio = SocketIO(app)
@app.route('/Home')
def Home():
        
        try:
            ser = serial.Serial(port='COM8',
                         baudrate=115200,bytesize=8,
                         parity=serial.PARITY_NONE,
                         timeout=20000 )
            print ("Serial port is open")
  
            
            
        except Exception as e:
            return render_template("TruShine3030.html", alert = True)
        # print ("error open serial port: " + str(e))
            exit()
        return render_template("TruShine3030.html")
@app.route('/About')
def About():
    return render_template('About us.html')
@app.route('/Idee')
def Idee():
    return render_template('Idee.html')
@app.route('/Kontakt')
def Kontakt():
    return render_template('Kontakt.html')
@app.route('/Name')
def Name():
    return render_template('Namen.html')


@app.route('/')
def index():
        try:
            ser = serial.Serial(port='COM8',
                         baudrate=115200,bytesize=8,
                         parity=serial.PARITY_NONE,
                         timeout=20000 )
            print ("Serial port is open")
  
            
            
        except Exception as e:
            return render_template("TruShine3030.html", alert = True)
        # print ("error open serial port: " + str(e))
            exit()
        return render_template("TruShine3030.html")

@app.route('/submit', methods=['POST'])
def submit():
    user_name = request.form.get("Name")
    # Bluetooth code here
    
    try:
        ser = serial.Serial(port='COM8',
                         baudrate=115200,bytesize=8,
                         parity=serial.PARITY_NONE,
                         timeout=20000 )
        user_name = user_name.upper()
        ser.write(bytes(user_name, "utf-8"))
        ser.write(b'~')
        print("write data: reboot")
        

        
    
    except Exception as e:
        print ("error communicating...: " + str(e))
    return render_template("Vorschau.html", name = user_name)

@app.route("/ausgabe", methods=['POST'])
def ausgabe():
    #Moving forward code
    ser = serial.Serial(port='COM8',
                         baudrate=115200,bytesize=8,
                         parity=serial.PARITY_NONE,
                         timeout=20000 )
    ser.write(b'>')
    return render_template('TruShine3030.html')

@app.route("/reset", methods=['POST'])
def reset():
    #Moving forward code
    ser = serial.Serial(port='COM8',
                         baudrate=115200,bytesize=8,
                         parity=serial.PARITY_NONE,
                         timeout=20000 )
    ser.write(b'!')
    return render_template('TruShine3030.html')

@app.route("/stop", methods=['POST'])
def stop():
    #Moving forward code
    ser = serial.Serial(port='COM8',
                         baudrate=115200,bytesize=8,
                         parity=serial.PARITY_NONE,
                         timeout=20000 )
    ser.write(b'#')
    return render_template('TruShine3030.html')

@socketio.on('ping')
def handle_ping():
    socketio.emit('pong')

    for i in range(14):
        time.sleep(0.2)
        socketio.emit('colored', i)
        

if __name__ == '__main__':
	socketio.run(app)