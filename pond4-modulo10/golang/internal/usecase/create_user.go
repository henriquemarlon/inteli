package usecase

import (
	"github.com/henriquemarlon/pond4-modulo10/internal/domain/entity"
)

type CreateUserInputDTO struct {
	Name     string
	Email    string
	Password string
}

type CreateUserOutputDTO struct {
	ID    uint
	Name  string
	Email string
}

type CreateUserUseCase struct {
	UserRepository entity.UserRepository
}

func NewCreateUserUseCase(userRepository entity.UserRepository) *CreateUserUseCase {
	return &CreateUserUseCase{
		UserRepository: userRepository,
	}
}

func (uc *CreateUserUseCase) Execute(input CreateUserInputDTO) (*CreateUserOutputDTO, error) {
	user := entity.NewUser(input.Name, input.Email, input.Password)
	err := uc.UserRepository.CreateUser(user)
	if err != nil {
		return nil, err
	}
	return &CreateUserOutputDTO{
		ID:    user.Id,
		Name:  user.Name,
		Email: user.Email,
	}, nil
}