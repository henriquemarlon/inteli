package usecase

import (
	"github.com/henriquemarlon/pond4-modulo10/internal/domain/entity"
)

type UpdateUserInputDTO struct {
	Id       uint
	Name     string
	Email    string
	Password string
}

type UpdateUserUseCase struct {
	UserRepository entity.UserRepository
}

func NewUpdateUserUseCase(userRepository entity.UserRepository) *UpdateUserUseCase {
	return &UpdateUserUseCase{
		UserRepository: userRepository,
	}
}

func (uc *UpdateUserUseCase) Execute(input UpdateUserInputDTO) error {
	user := &entity.User{
		Id:       input.Id,
		Name:     input.Name,
		Email:    input.Email,
		Password: input.Password,
	}
	err := uc.UserRepository.UpdateUser(user)
	if err != nil {
		return err
	}
	return nil
}
