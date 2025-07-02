# Server Room Environmental Monitoring System

The system is designed to monitor basic environmental parameters in a server room, such as temperature, humidity, and the presence of smoke and flammable gases (including carbon monoxide, CO).

Data is collected using two sensors:
- DHT11 – measures temperature and air humidity
- MQ-2 – measures smoke and carbon monoxide (CO) levels

An Arduino Uno R4 WiFi microcontroller reads data from the sensors and displays it every 2 seconds on a 2x16 LCD screen. The user can switch between display modes using a push button. 

There are three available display modes:
- Temperature and humidity (numeric values)
- Gas levels (text-based description: Normal, Moderate, High, Very High)
- Date and time – synchronized with an NTP server every 5 minutes via the Wi-Fi module and stored in a real-time clock (RTC)

The Arduino sends data to a server using the lightweight MQTT communication protocol, enabling integration with the Node-RED platform. Both environmental data and configuration information (e.g., alarm and fan states) are transmitted.

The Node-RED user interface allows:
- Viewing current sensor readings
- Remote control of the alarm and fans (indicated by LEDs)
- Setting reaction thresholds, after which corresponding mechanisms are triggered — e.g., activating the alarm or ventilation system if they are in auto mode

![image](https://github.com/user-attachments/assets/351de995-41e5-4ae4-bb39-76c9ffc91512)
