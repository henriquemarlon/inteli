package main

import (
	user "github.com/m10-p1/async/controllers/user"
	task "github.com/m10-p1/async/controllers/task"
	"log"
	"net/http"
)

func main() {

	mux := http.NewServeMux()
	mux.HandleFunc("POST /api/v1/createUser", user.CreateUser)
	mux.HandleFunc("POST /api/v1/login", user.Login)
	mux.HandleFunc("GET /api/v1/getTasks/{userID}", task.GetTasksById)
	mux.HandleFunc("POST /api/v1/createTask/{userID}", task.CreateTask)
	mux.HandleFunc("PUT /api/v1/editTask", task.EditTask)
	mux.HandleFunc("DELETE /api/v1/deleteTask", task.DeleteTask)
	log.Fatal(http.ListenAndServe(":8000", mux))
}
