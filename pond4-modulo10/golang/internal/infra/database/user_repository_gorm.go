package database

import (
	"database/sql"
	"fmt"
	"log"

	"github.com/henriquemarlon/pond4-modulo10/internal/domain/entity"
	"gorm.io/gorm"
)

type UserRepositoryGorm struct {
	Db *gorm.DB
}

func NewUserRepositoryGorm(db *gorm.DB) *UserRepositoryGorm {
	return &UserRepositoryGorm{Db: db}
}

func (r *UserRepositoryGorm) CreateUser(user *entity.User) error {
	err := r.Db.Table("usuarios").Create(user).Error
	if err != nil {
		return fmt.Errorf("error creating user: %w", err)
	}
	return nil
}

func (r *UserRepositoryGorm) FindUserById(id uint) (*entity.User, error) {
	var user entity.User
	log.Printf("id: %v", id)
	err := r.Db.Table("usuarios").Where("id = ?", id).First(&user).Error
	if err != nil {
		if err == sql.ErrNoRows {
			return nil, nil
		}
		return nil, fmt.Errorf("error finding user: %w", err)
	}
	return &user, nil
}

func (r *UserRepositoryGorm) FindAllUsers() ([]*entity.User, error) {
	var users []*entity.User
	err := r.Db.Table("usuarios").Find(&users).Error
	if err != nil {
		return nil, fmt.Errorf("error finding users: %w", err)
	}
	return users, nil
}

func (r *UserRepositoryGorm) UpdateUser(user *entity.User) error {
	err := r.Db.Table("usuarios").Save(user).Error
	if err != nil {
		return fmt.Errorf("error updating user: %w", err)
	}
	return nil
}

func (r *UserRepositoryGorm) DeleteUser(user *entity.User) error {
	err := r.Db.Table("usuarios").Delete(user).Error
	if err != nil {
		return fmt.Errorf("error deleting user: %w", err)
	}
	return nil
}