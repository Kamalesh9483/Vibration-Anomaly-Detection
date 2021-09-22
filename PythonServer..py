
import json
import socket
import csv
from datetime import datetime
from dateutil.tz import gettz
dtobj = datetime.now(tz=gettz('Asia/Kolkata'))
import smtplib # mail

import sqlite3
# import random
import time
import datetime

con = sqlite3.connect('example.db')
cur = con.cursor()

# Create table
cur.execute('''CREATE TABLE sensorData
               (Time BIGINT, sensorVal REAL)''')


Ax = 0.0
UDP_IP = "0.0.0.0"
UDP_PORT = 8090

sock = socket.socket(socket.AF_INET, # Interne
                    socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

# labels = (dtobj.strftime("%d/%m/%Y, %H:%M:%S"))

# creates SMTP session
s = smtplib.SMTP('smtp.gmail.com', 587)

# start TLS for security
s.starttls()

# Authentication
s.login("kamalesh9483@gmail.com", "Dexiatoung123*")
  
# fields = ['Time', 'SensorVal'] 
# myFile = open('sensorData.csv', 'a', newline='')
# with myFile:
#     writer = csv.writer(myFile)
#     # writer.writeheader()
#     writer.writerow(fields)
#     while True:
#         data , addr = sock.recvfrom(1024) # buffer size is 1024 bytes
#         data = json.loads(data)
#         print(data["sensor"],data["millisec"])
#         writer.writerow([data["millisec"]/1000,data["sensor"]])
#         if(data["sensor"] >= 1.5 or data["sensor"] <=0.75):
#             # message to be sent
#             message = "Anomaly Detected. Sensor Value is {}".format(data["sensor"])
#             # sending the mail
#             s.sendmail("kamalesh9483@gmail.com", "kamalesh3894@gmail.com", message)

while True:
    data , addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    data = json.loads(data)
    print(data["sensor"],data["millisec"])
    cur.execute("INSERT INTO sensorData VALUES (?,?)", (data["millisec"],data["sensor"]))
    #   Save (commit) the changes
    con.commit()
    time.sleep(0.01)
    if(data["sensor"] >= 1.5 or data["sensor"] <=0.75):
#       # message to be sent
        message = "Anomaly Detected. Sensor Value is {}".format(data["sensor"])
#       # sending the mail
        s.sendmail("kamalesh9483@gmail.com", "kamalesh3894@gmail.com", message)
con.close()
    