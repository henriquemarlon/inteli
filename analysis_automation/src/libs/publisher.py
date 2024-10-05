import time
from paho.mqtt import client as mqtt_client

class Publisher:
    def __init__(self, client_id, topic, broker, port):
        self.client_id = client_id
        self.topic = topic
        self.broker = broker
        self.port = port

    def connect_mqtt(self):
        self.client = mqtt_client.Client(self.client_id)
        self.client.connect(self.broker, self.port)
        return self.client

    def enable_eletromagnet(self):
        #while True:
        time.sleep(1)
        msg = 1
        result = self.client.publish(self.topic, msg)
        status = result[0]
        if status == 0:
            print(f"Send `{msg}` to topic `{self.topic}`")
        else:
            print(f"Failed to send message to topic {self.topic}")

    def disable_eletromagnet(self):
        #while True:
        time.sleep(1)
        msg = 0
        result = self.client.publish(self.topic, msg)
        status = result[0]
        if status == 0:
            print(f"Send `{msg}` to topic `{self.topic}`")
        else:
            print(f"Failed to send message to topic {self.topic}")

# PB = Publisher("publisher", "eletromagnet", "< localIP >", 1883)