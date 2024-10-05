package user

import (
	"encoding/json"
	"net/http"
	userModel "github.com/m10-p1/async/models/user"
)

type User struct {
	ID     string  `json:"id"`
	Name   string  `json:"name"`
	Email  string  `json:"email"`
	Password string `json:"password"`
}

func CreateUser(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	var user User
	_ = json.NewDecoder(r.Body).Decode(&user)
	err := userModel.NewUser(user.Name, user.Email, user.Password)

	if err != nil {
        w.WriteHeader(http.StatusInternalServerError)
        message := "Erro ao cadastrar usuário. Email já cadastrado."
        w.Write([]byte(message))
        return
		
    } else {
		w.WriteHeader(http.StatusOK)
		message := "Usuario cadastrado com sucesso"
		w.Write([]byte(message))
		return
	}
}

func Login(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	var user User
	_ = json.NewDecoder(r.Body).Decode(&user)
	token, userID := userModel.Login(user.Email, user.Password)
	response := map[string]interface{}{
        "token": token,
        "userID": userID,
    }
	w.WriteHeader(http.StatusOK)
	json.NewEncoder(w).Encode(response)
}