package main

import (
	"fmt"
	"sync"
	"github.com/henriquemarlon/p1-m9/internal/domain/entity"
)

func main() {
	numStations := 5
	var wg sync.WaitGroup

	for i := 0; i < numStations; i++ {
		wg.Add(1)
		go func(stationID int) {
			defer wg.Done()
			stationName := fmt.Sprintf("ST-%v", stationID)
			entity.StartFreezer(stationName, "tcp://broker:1891")
		}(i)
	}
	wg.Wait()
}
