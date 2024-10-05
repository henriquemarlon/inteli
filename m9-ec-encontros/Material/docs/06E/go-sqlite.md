---
title: SQLite com Go
sidebar_position: 2
sidebar_class_name: opcional
slug: /go-sqlite
---

# SQlite com Go

## 1. Criando um novo projeto

Assim como praticamente qualquer outro tutorial da linguagem Go, vamos começar
criando um novo módulo:

```bash
go mod init goSqlite
```

## 2. Acessando o `db` com Go

A seguir, crie um arquivo de código fonte de Go e preencha-o com o seguinte:

```go showLineNumbers title="teste_sqlite.go"
package main

import (
	"database/sql"
	_ "github.com/mattn/go-sqlite3"
	"log"
)

func main() {
	db, _ := sql.Open("sqlite3", "./exemplo.db")
	defer db.Close() // Defer Closing the database

	// Criando a tabla
	sqlStmt := `
  CREATE TABLE IF NOT EXISTS usuarios
  (id INTEGER PRIMARY KEY, nome TEXT, idade INTEGER)
  `
	// Preparando o sql statement de forma segura
	command, err := db.Prepare(sqlStmt)
	if err != nil {
		log.Fatal(err.Error())
	}

	// Executando o comando sql
	command.Exec()

	// Criando uma função para inserir usuários
	insertUser := func(db *sql.DB, nome string, idade int) {
		stmt := `INSERT INTO usuarios(nome, idade) VALUES (?, ?)`
		statement, err := db.Prepare(stmt)
		if err != nil {
			log.Fatalln(err.Error())
		}
		_, err = statement.Exec(nome, idade)
		if err != nil {
			log.Fatalln(err.Error())
		}
	}

	insertUser(db, "Alice", 30)
	insertUser(db, "Bob", 25)

	displayUsers(db)
}

func displayUsers(db *sql.DB) {
	row, err := db.Query("SELECT * FROM usuarios ORDER BY nome")
	if err != nil {
		log.Fatal(err)
	}
	defer row.Close()
	for row.Next() {
		var id int
		var name string
		var age string
		row.Scan(&id, &name, &age)
		log.Println("Usuário: ", id, " ", name, " ", age)
	}
}
```

Por se tratar de uma linguagem de mais baixo nível que o Python, com Go você
vai precisar interagir com o db linha a linha. Na verdade, a biblioteca de
sqlite de Go é bastante similar à de C. Ponto para o Python pela maior
simplicidade, mas nada de outro mundo a implementação em Go.

## 3. Baixando as dependências e executando

Como já ~devíamos estar~ estamos acostumados, vamos instalar a nível de projeto
todas as dependências usando:

```bash
go mod tidy
```

E, para rodar:

```
go run teste_sqlite.go
```

:::warning

Vai demorar mesmo para rodar pela primeira vez, é normal. Não se desespere. Go
é uma linguagem compilada e suas bibliotecas também precisam ser compiladas.
Sqlite é uma biblioteca razoavelmente grande, portanto vai demorar um pouquinho
mesmo. A partir da segunda vez que rodar, o compilador de Go já vai entender
que nada mudou na biblioteca e usar uma versão cacheada. Outro ponto para o
Python, que usa uma versão pré-compilada do sqlite.

:::
