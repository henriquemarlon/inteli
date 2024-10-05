import paho.mqtt.client as mqtt
import time
import json

# Configuração do cliente
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, "python_publisher")

# Conecte ao broker
client.connect("localhost", 1891, 60)

def publish_sensor_data():
    sensor_data = {
        "unit": "Celsius",
        "value": 25.5,  # Exemplo de valor de medição
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "location": "51.5074, -0.1278"  # Exemplo de dados de latitude e longitude
    }
    
    client.publish("sensor/data", json.dumps(sensor_data))
    print(f"Publicado: {sensor_data}")

try:
    while True:
        publish_sensor_data()
        time.sleep(5)
except KeyboardInterrupt:
    print("Publicação encerrada")

client.disconnect()
