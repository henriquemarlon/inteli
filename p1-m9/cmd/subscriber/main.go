package main

import (
	"encoding/json"
	"fmt"
	"github.com/henriquemarlon/p1-m9/internal/domain/entity"
	MQTT "github.com/eclipse/paho.mqtt.golang"
)

var messagePubHandler MQTT.MessageHandler = func(client MQTT.Client, msg MQTT.Message) {
	data := entity.Data{}
	payload := msg.Payload()
	if err := json.Unmarshal(payload, &data); err != nil {
		fmt.Println(err)
		return
	}
	switch data.Type {
	case "freezer":
		if data.Temperature > -15 {
			fmt.Printf("%s - Temperature: %v ºC [ALERT High Temperature - Freezer] \n", data.ID, data.Temperature)
		}
		if data.Temperature < -25 {
			fmt.Printf("%s - Temperature: %v ºC [ALERT Low Temperature - Freezer] \n", data.ID, data.Temperature)
		}
		if data.Temperature > -25 && data.Temperature < -15  {
			fmt.Printf("%s - Temperature: %v ºC [OK Freezer] \n", data.ID, data.Temperature)
		}
	case "refrigerator":
		if data.Temperature > 10 {
			fmt.Printf("%s - Temperature: %v ºC [ALERT High Temperature - Refrigerator] \n", data.ID, data.Temperature)
		}
		if data.Temperature < 2 {
			fmt.Printf("%s - Temperature: %v ºC [ALERT Low Temperature - Refrigerator] \n", data.ID, data.Temperature)
		}
		if data.Temperature > 2 && data.Temperature < 10  {
			fmt.Printf("%s - Temperature: %v ºC [OK Refrigerator] \n", data.ID, data.Temperature)
		}
	}
}

func main() {
	opts := MQTT.NewClientOptions().AddBroker("tcp://broker:1891")
	opts.SetClientID("subscriber")
	opts.SetDefaultPublishHandler(messagePubHandler)

	client := MQTT.NewClient(opts)
	if token := client.Connect(); token.Wait() && token.Error() != nil {
		panic(token.Error())
	}

	if token := client.Subscribe("/sectors", 1, nil); token.Wait() && token.Error() != nil {
		fmt.Println(token.Error())
		return
	}
	
	select {}
}