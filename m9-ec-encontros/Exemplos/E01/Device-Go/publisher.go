package main

import (
	"encoding/json"
	"fmt"
	"time"

	MQTT "github.com/eclipse/paho.mqtt.golang"
)

func main() {
	opts := MQTT.NewClientOptions().AddBroker("tcp://localhost:1891")
	opts.SetClientID("go_publisher")

	client := MQTT.NewClient(opts)
	if token := client.Connect(); token.Wait() && token.Error() != nil {
		panic(token.Error())
	}

	for {
		sensorData := map[string]interface{}{
			"unit":      "Celsius",
			"value":     25.5, // Exemplo de valor de medição
			"timestamp": time.Now().Format(time.RFC3339),
			"location":  "51.5074, -0.1278", // Exemplo de dados de latitude e longitude
		}

		data, err := json.Marshal(sensorData)
		if err != nil {
			fmt.Println("Erro ao serializar os dados do sensor:", err)
			continue
		}

		token := client.Publish("sensor/data", 0, false, data)
		token.Wait()

		fmt.Printf("Publicado: %s\n", data)
		time.Sleep(5 * time.Second)
	}
}
