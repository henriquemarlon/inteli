package usecase

import (
	"github.com/henriquemarlon/ENG-COMP-M9/P01-04/internal/domain/entity"
	"go.mongodb.org/mongo-driver/bson/primitive"
)


type CreateSensorUseCase struct {
	SensorRepository entity.SensorRepository
}

type CreateSensorInputDTO struct {
	Name      string  `json:"name"`
	Latitude  float64 `json:"latitude"`
	Longitude float64 `json:"longitude"`
}

type CreateSensorOutputDTO struct {
	ID        primitive.ObjectID `json:"_id"`
	Name      string             `json:"name"`
	Latitude  float64            `json:"latitude"`
	Longitude float64            `json:"longitude"`
}

func NewCreateSensorUseCase(sensorRepository entity.SensorRepository) *CreateSensorUseCase {
	return &CreateSensorUseCase{SensorRepository: sensorRepository}
}

func (c *CreateSensorUseCase) Execute(input CreateSensorInputDTO) (*CreateSensorOutputDTO, error) {
	sensor := entity.NewSensor(input.Name, input.Latitude, input.Longitude)
	id, err := c.SensorRepository.CreateSensor(sensor)
	if err != nil {
		return nil, err
	}
	return &CreateSensorOutputDTO{
		ID:        id.InsertedID.(primitive.ObjectID),
		Name:      sensor.Name,
		Latitude:  sensor.Latitude,
		Longitude: sensor.Longitude,
	}, nil
}
