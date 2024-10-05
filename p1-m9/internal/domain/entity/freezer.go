package entity

import (
	"encoding/json"
	"fmt"
	"math/rand"
	"time"
	"math"
)

var FreezerTemperature = map[string]Interval{
	"temperature": {-30, -10},
}

func FreezerPayloadEntropy(key string) float64 {
	rand.NewSource(time.Now().UnixNano())
	max := FreezerTemperature[key].Maximum
	min := FreezerTemperature[key].Minimum
	value := rand.Float64()*(max-min) + min
	return math.Round(value)
}

func GenerateFreezerPayload() float64 {
	data := FreezerPayloadEntropy("temperature")
	return data
}

func StartFreezer(id string, url string) {
	client := ConnectMQTT(rand.NewSource(time.Now().UnixNano()), url)

	for {
		data := Data{
			ID:          id,
			Type:        "freezer",
			Temperature: GenerateFreezerPayload(),
			TimeStamp:   time.Now().String(),
		}
		payload, err := json.Marshal(data)
		if err != nil {
			fmt.Println("Error converting to JSON:", err)
			return
		}
		token := client.Publish("/sectors", 1, false, string(payload))
		token.Wait()
		time.Sleep(5 * time.Second)
	}
}
