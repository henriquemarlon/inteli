package user

import (
	"fmt"
    models "github.com/m10-p1/async/models"
	_ "github.com/lib/pq"
    "database/sql"
    "errors"
    
)


func NewUser(name string, email string, password string) error {
	db := models.ConnectDb()

    var count int
    db.QueryRow("SELECT COUNT(*) FROM users WHERE email = $1", email).Scan(&count)
    if count > 0 {
        fmt.Println("Um usuário com este email já está cadastrado.")
        return errors.New("um usuário com este email já está cadastrado")
    }

	stmt, err := db.Prepare("INSERT INTO users(name, email, password) VALUES($1, $2, $3)")
	if err != nil {
		panic(err)
	}
	defer stmt.Close()

	_, err = stmt.Exec(name, email, password)
	if err != nil {
		panic(err)
	}

	fmt.Println("Dados inseridos com sucesso!")
	db.Close()
    return nil
}


func Login(email string, password string) (string, int64) {
    db := models.ConnectDb()

    stmt, err := db.Prepare("SELECT id FROM users WHERE email = $1 AND password = $2")
    if err != nil {
        panic(err)
    }
    defer stmt.Close()

    var userID int64
    err = stmt.QueryRow(email, password).Scan(&userID)
    if err != nil {
        if err == sql.ErrNoRows {
            panic("Usuário não encontrado")
        }
        panic(err)
    }

    tokenString, err := models.CreateToken(email)

    fmt.Println("Login realizado com sucesso!")
    db.Close()

    return tokenString, userID
}

