package handler

import (
	"net/http"
	"strconv"

	"github.com/gin-gonic/gin"
	"github.com/henriquemarlon/pond4-modulo10/internal/domain/entity"
	"github.com/henriquemarlon/pond4-modulo10/internal/usecase"
)

type UserHandlers struct {
	UserRepository entity.UserRepository
}

func NewUserHandlers(userRepository entity.UserRepository) *UserHandlers {
	return &UserHandlers{
		UserRepository: userRepository,
	}
}

// CreateUser creates a new user.
// @Summary Create new user
// @Description Create a new user with the input payload.
// @Tags user
// @Accept  json
// @Produce  json
// @Param   user body usecase.CreateUserInputDTO true "Create User"
// @Success 201 {object} usecase.CreateUserOutputDTO
// @Failure 500 {object} map[string]interface{}
// @Router /user [post]
func (uh *UserHandlers) CreateUser(c *gin.Context) {
	var input usecase.CreateUserInputDTO
	if err := c.ShouldBindJSON(&input); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}
	output, err := usecase.NewCreateUserUseCase(uh.UserRepository).Execute(input)
	if err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()})
		return
	}
	c.JSON(http.StatusCreated, output)
}

// FindUserById finds a user by their user ID.
// @Summary Find user by ID
// @Description Retrieve a user by their unique ID.
// @Tags user
// @Accept  json
// @Produce  json
// @Param   id path int true "User ID"
// @Success 200 {object} usecase.FindUserByIdOutputDTO
// @Failure 500 {object} map[string]interface{}
// @Router /user/{id} [get]
func (uh *UserHandlers) FindUserById(c *gin.Context) {
	id, _ := strconv.ParseUint(c.Param("id"), 10, 32)
	output, err := usecase.NewFindUserByIdUseCase(uh.UserRepository).Execute(usecase.FindUserByIdInputDTO{Id: uint(id)})
	if err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()})
		return
	}
	c.JSON(http.StatusOK, output)
}

// FindAllUsers retrieves all users.
// @Summary Retrieve all users
// @Description Get a list of all registered users.
// @Tags user
// @Accept  json
// @Produce  json
// @Success 200 {array} usecase.FindAllUsersOutputDTO
// @Failure 500 {object} map[string]interface{}
// @Router /user [get]
func (uh *UserHandlers) FindAllUsers(c *gin.Context) {
	output, err := usecase.NewFindAllUsersUseCase(uh.UserRepository).Execute()
	if err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()})
		return
	}
	c.JSON(http.StatusOK, output)
}

// UpdateUser updates an existing user.
// @Summary Update user
// @Description Update an existing user's information.
// @Tags user
// @Accept  json
// @Produce  json
// @Param   id path int true "User ID"
// @Param   user body usecase.UpdateUserInputDTO true "Update User"
// @Success 200 {object} map[string]interface{}
// @Failure 500 {object} map[string]interface{}
// @Router /user/{id} [put]
func (uh *UserHandlers) UpdateUser(c *gin.Context) {
	var input usecase.UpdateUserInputDTO
	id, _ := strconv.ParseUint(c.Param("id"), 10, 32)
	if err := c.ShouldBindJSON(&input); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}
	input.Id = uint(id)
	if err := usecase.NewUpdateUserUseCase(uh.UserRepository).Execute(input); err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()})
		return
	}
	c.JSON(http.StatusOK, gin.H{"message": "user updated"})
}

// DeleteUser deletes a user by ID.
// @Summary Delete user
// @Description Delete a user by their unique ID.
// @Tags user
// @Accept  json
// @Produce  json
// @Param   id path int true "User ID"
// @Success 200 {object} map[string]interface{}
// @Failure 500 {object} map[string]interface{}
// @Router /user/{id} [delete]
func (uh *UserHandlers) DeleteUser(c *gin.Context) {
	id, _ := strconv.ParseUint(c.Param("id"), 10, 32)
	if err := usecase.NewDeleteUserUseCase(uh.UserRepository).Execute(usecase.DeleteUserInputDTO{Id: uint(id)}); err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()})
		return
	}
	c.JSON(http.StatusOK, gin.H{"message": "user deleted"})
}
