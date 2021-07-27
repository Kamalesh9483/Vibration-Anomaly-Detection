
import json
import socket
import csv
from datetime import datetime
from dateutil.tz import gettz
dtobj = datetime.now(tz=gettz('Asia/Kolkata'))
import smtplib # mail
# labels = (dtobj.strftime("%d/%m/%Y %H:%M:%S"))

# from threading import Thread, Lock
# mutex = Lock()
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
  
fields = ['Time', 'SensorVal'] 
myFile = open('sensorData.csv', 'a', newline='')
with myFile:
    writer = csv.writer(myFile)
    # writer.writeheader()
    writer.writerow(fields)
    while True:
        data , addr = sock.recvfrom(1024) # buffer size is 1024 bytes
        data = json.loads(data)
        print(data["sensor"],data["millisec"])
        writer.writerow([data["millisec"]/1000,data["sensor"]])
        if(data["sensor"] >= 1.5 or data["sensor"] <=0.75):
            # message to be sent
            message = "Anomaly Detected. Sensor Value is {}".format(data["sensor"])
            # sending the mail
            s.sendmail("kamalesh9483@gmail.com", "kamalesh3894@gmail.com", message)

        


    # info = [data[i:i+2] for i in range(0, len(data), 2)]
    # labels = (dtobj.strftime("%d/%m/%Y %H:%M:%S"))
    # for i in range(0, len(data), 3): 
    #     sen = []
    #     # print(round(((data[i]<<8 | data[i+1])/16384.000),3))
    #     time = round(((data[i+1]<<8 | data[i+2])/1000),3)
    #     sensorData= (round(((data[i+3]<<8 | data[i+4])/16384.000),3)) 
    #     # sensorData= (round(((data[i]<<8 | data[i+1])/16384.000),3))
    #     sen.append(sensorData)
    #     print(sensorData)
    #     if(i >=0):
    #         myFile = open('sensorData.csv', 'a')
    #         with myFile:
    #             writer = csv.writer(myFile)
    #             writer.writerow([time,sen])
    # data = json.loads(output.decode('utf-8'))
    
    # print(data)
    

# @app.route('/', methods=["GET", "POST"])
# def main():
#     return render_template('index.html')


# @app.route('/data', methods=["GET", "POST"])
# def data():
#     global Ax
#     mutex.acquire()
#     data = [time() * 1000, Ax]
#     response = make_response(json.dumps(data))
#     response.content_type = 'application/json'
#     mutex.release()
#     return response
    

# @app.route('/sensorData', methods=["GET", "POST"])
# def sensorData():
#     global Ax
#     mutex.acquire()
#     Ax = float(request.args.get('Ax'))
#     print(Ax)
#     mutex.release()
#     return "0"

# if __name__ == "__main__":
#     app.run(debug=True, host='0.0.0.0', port= 8090)