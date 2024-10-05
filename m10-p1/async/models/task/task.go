package task

import (
    models "github.com/m10-p1/async/models"
	_ "github.com/lib/pq"
	"errors"
	"fmt"
)

type Task struct {
	ID     string  `json:"id"`
	UserID   string  `json:"user_id"`
	Title  string  `json:"title"`
	Description string `json:"description"`
}

func GetAllTasksById(userID string) ([]Task, error){
	db := models.ConnectDb()

	rows, err := db.Query("SELECT * FROM tasks WHERE user_id = $1", userID)
    if err != nil {
        return nil, err
    }
    defer rows.Close()

    var tasks []Task
    for rows.Next() {
        var task Task
		err := rows.Scan(&task.ID, &task.UserID, &task.Title, &task.Description)
        if err != nil {
            return nil, err
        }
        tasks = append(tasks, task)
    }
	
	db.Close()
	return tasks, nil
}

func NewTask(userID string, title string, description string) error {
	db := models.ConnectDb()

	var count int
    err := db.QueryRow("SELECT COUNT(*) FROM users WHERE id = $1", userID).Scan(&count)
    if err != nil {
        return err
    }
    if count == 0 {
        return errors.New("o usuário com o ID fornecido não existe")
    }

	stmt, err := db.Prepare("INSERT INTO tasks(user_id, title, description) VALUES($1, $2, $3)")
	if err != nil {
		panic(err)
	}
	defer stmt.Close()

	_, err = stmt.Exec(userID, title, description)
	if err != nil {
		panic(err)
	}

	fmt.Println("Tarefa inserida com sucesso!")
	db.Close()
    return nil
}

func EditTask(taskID string, title string, description string) error {
	db := models.ConnectDb()

	stmt, err := db.Prepare("UPDATE tasks SET title = $1, description = $2 WHERE id = $3")
	if err != nil {
		panic(err)
	}
	defer stmt.Close()

	_, err = stmt.Exec(title, description, taskID)
	if err != nil {
		panic(err)
	}

	fmt.Println("Tarefa editada com sucesso!")
	db.Close()
	return nil
}

func DeleteTask(taskID string) error {
	db := models.ConnectDb()

	stmt, err := db.Prepare("DELETE FROM tasks WHERE id = $1")
	if err != nil {
		panic(err)
	}
	defer stmt.Close()

	_, err = stmt.Exec(taskID)
	if err != nil {
		panic(err)
	}

	fmt.Println("Tarefa deletada com sucesso!")
	db.Close()
	return nil
}