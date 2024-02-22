from machine import I2C, Pin, unique_id
from bme680 import *
from umqtt.simple import MQTTClient
from ubinascii import hexlify

i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)  # initializing the I2C method
bme = BME680_I2C(i2c=i2c)

client_id = hexlify(unique_id())
mqtt_server = '192.168.0.208'
client = MQTTClient(client_id, mqtt_server)
client.connect()
while True:
    print(bme.temperature, bme.humidity, bme.pressure, bme.gas)
    
    client.publish(b'Temperature', str(bme.temperature).encode())
    client.publish(b'Humidity', str(bme.humidity).encode())
    client.publish(b'Pressure', str(bme.pressure).encode())
    client.publish(b'Gas', str(bme.gas).encode())
    time.sleep(2)

