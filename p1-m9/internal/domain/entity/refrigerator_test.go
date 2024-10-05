package entity

import (
	"fmt"
	"math/rand"
	"testing"
	"time"

	MQTT "github.com/eclipse/paho.mqtt.golang"
)

func TestGenerateRefrigeratorPayload(t *testing.T) {
	data := GenerateRefrigeratorPayload()
	if data < -0 || data > 12 {
		t.Errorf("Temperature must be between -30 and -10")
	}
}

func TestConnectRefrigeratorMQTT(t *testing.T) {
	client := ConnectMQTT(rand.NewSource(time.Now().UnixNano()), "tcp://localhost:1891")
	defer client.Disconnect(500)
	if !client.IsConnected() {
		t.Errorf("Unable to connect to MQTT broker\x1b[0m")
	}
}

func TestRefrigeratorMessageTransmissionAndQOS(t *testing.T) {
	var receipts []string

	var handler MQTT.MessageHandler = func(client MQTT.Client, msg MQTT.Message) {
		receipts = append(receipts, fmt.Sprintf("New message on topic %s: %s", msg.Topic(), msg.Payload()))
		if msg.Qos() != 1 {
			t.Errorf("QoS must be 1")
		}
	}

	opts := MQTT.NewClientOptions().AddBroker("tcp://localhost:1891")
	opts.SetClientID("test")
	opts.SetDefaultPublishHandler(handler)

	client := MQTT.NewClient(opts)
	defer client.Disconnect(500)

	if session := client.Connect(); session.Wait() && session.Error() != nil {
		panic(session.Error())
	}

	done := make(chan bool)

	go func() {
		StartRefrigerator("ST-1", "tcp://localhost:1891")
		done <- true
	}()

	go func() {
		if token := client.Subscribe("/sectors", 1, nil); token.Wait() && token.Error() != nil {
			t.Logf("Error subscribing: %s", token.Error())
			return
		}
	}()

	time.Sleep(2 * time.Second)

	if len(receipts) == 0 {
		t.Errorf("No messages received")
	} else {
		for _, receipt := range receipts {
			t.Log(receipt)
		}
	}
}
