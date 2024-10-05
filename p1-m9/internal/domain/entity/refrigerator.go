package entity

import (
	"encoding/json"
	"fmt"
	"math/rand"
	"time"
	"math"
)

var RefrigeratorTemperature = map[string]Interval{
	"temperature": {0, 12},
}

func RefrigeratorPayloadEntropy(key string) float64 {
	rand.NewSource(time.Now().UnixNano())
	max := RefrigeratorTemperature[key].Maximum
	min := RefrigeratorTemperature[key].Minimum
	value := rand.Float64()*(max-min) + min
	return math.Round(value)
}

func GenerateRefrigeratorPayload() float64 {
	data := RefrigeratorPayloadEntropy("temperature")
	return data
}

func StartRefrigerator(id string, url string) {
	client := ConnectMQTT(rand.NewSource(time.Now().UnixNano()), url)

	for {
		data := Data{
			ID:          id,
			Type:        "refrigerator",
			Temperature: GenerateRefrigeratorPayload(),
			TimeStamp:   time.Now().String(),
		}
		payload, err := json.Marshal(data)
		if err != nil {
			fmt.Println("Error converting to JSON:", err)
			return
		}
		token := client.Publish("/sectors", 1, false, string(payload))
		token.Wait()
		time.Sleep(2 * time.Second)
	}
}
