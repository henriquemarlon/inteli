package usecase

import (
	"log"
	"time"

	"github.com/henriquemarlon/hipercongo/internal/domain/entity"
)

type CreateSensorLogUseCase struct {
	SensorRepository entity.SensorRepository
}

type CreateSensorLogInputDTO struct {
	ID        string                 `json:"sensor_id"`
	Data      map[string]interface{} `json:"data"`
	Timestamp time.Time              `json:"timestamp"`
}

func NewCreateSensorLogUseCase(sensorRepository entity.SensorRepository) *CreateSensorLogUseCase {
	return &CreateSensorLogUseCase{SensorRepository: sensorRepository}
}

func (c *CreateSensorLogUseCase) Execute(input CreateSensorLogInputDTO) error {
	logData := entity.NewLog(input.ID, input.Data, input.Timestamp)
	err := c.SensorRepository.CreateSensorLog(logData)
	if err != nil {
		log.Printf("Error creating sensor log: %v", err)
	}
	return nil
}
