package usecase

import (
	"github.com/henriquemarlon/pond4-modulo10/internal/domain/entity"
)

type DeleteUserInputDTO struct {
	Id uint
}

type DeleteUserUseCase struct {
	UserRepository entity.UserRepository
}

func NewDeleteUserUseCase(userRepository entity.UserRepository) *DeleteUserUseCase {
	return &DeleteUserUseCase{
		UserRepository: userRepository,
	}
}

func (uc *DeleteUserUseCase) Execute(input DeleteUserInputDTO) error {
	user := &entity.User{
			Id: input.Id,
	}
	err := uc.UserRepository.DeleteUser(user)
	if err != nil {
			return err
	}
	return nil
}


