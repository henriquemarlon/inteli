package entity

import "time"

type UserRepository interface {
	CreateUser(user *User) error
	FindAllUsers() ([]*User, error)
	FindUserById(id uint) (*User, error)
	UpdateUser(user *User) error
	DeleteUser(user *User) error
}

type User struct {
	Id        uint      `json:"id" gorm:"primaryKey"`
	Name      string    `json:"name" gorm:"column:nome"`
	Email     string    `json:"email" gorm:"column:email"`
	Password  string    `json:"password" gorm:"column:senha"`
	CreatedAt time.Time `json:"created_at" gorm:"column:data_criacao"`
	UpdatedAt time.Time `json:"updated_at" gorm:"column:data_modificacao"`
}


func NewUser(name, email, password string) *User {
	return &User{
		Name:     name,
		Email:    email,
		Password: password,
	}
}
