# Vibration-Anomaly-Detection
This project is about detecting Vibration Anomaly and alerting the user

Wrote a firmware for STM32F401RE microcontroller that would receive the Acceleration Data from MPU6050, displaying on LCD2004A (I2C) and also using ESP32 (UART) connected to STM32F401RE to send the sensor data to server using Python Socket and streaming the dynamic sensor values to the chart in the web. From the time series sensor data available, if any sensor value is above the normal operating vibration limit an alert email is  sent  to the user indicating a anomaly activity has occurred. Also made an attempt to create a ML model to detect anomaly (Failure though).  This project finds application in Predictive Maintenance of the machine tools or any machine element operating under repeated loads.
# Demo 
![ezgif com-gif-maker](https://github.com/Kamalesh9483/Vibration-Anomaly-Detection/assets/80197808/45317c03-f5b0-46f5-b633-5e3545db1a95)
