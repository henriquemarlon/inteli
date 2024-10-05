package entity

import (
	"fmt"
	"math/rand"
	MQTT "github.com/eclipse/paho.mqtt.golang"
)

type Data struct {
	ID          string `json:"id"`
	Type        string `json:"type"`
	Temperature float64 `json:"temperature"`
	TimeStamp   string `json:"timestamp"`
}

type Interval struct {
	Minimum float64
	Maximum float64
}

func ConnectMQTT(seed rand.Source, url string) MQTT.Client {
	opts := MQTT.NewClientOptions().AddBroker(url)
	opts.SetClientID(fmt.Sprintf("station-%d", seed.Int63()))
	client := MQTT.NewClient(opts)
	if session := client.Connect(); session.Wait() && session.Error() != nil {
		panic(session.Error())
	}
	return client
}
