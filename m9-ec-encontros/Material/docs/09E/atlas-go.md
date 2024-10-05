---
title: MongoDB Atlas com Go
sidebar_position: 3
sidebar_class_name: opcional
slug: /altas-go
---

import Admonition from '@theme/Admonition';

# Utilizando o MongoDB Atlas c/ Go

Para interagir com um banco de dados MongoDB Atlas usando Go, você pode usar o
pacote `mongo-go-driver`. Abaixo está um exemplo simples que mostra como se
conectar a um cluster do MongoDB Atlas, inserir um documento em uma coleção e
recuperar documentos da coleção.

Primeiro, certifique-se de ter o `mongo-go-driver` instalado. Se não tiver,
você pode instalá-lo usando o seguinte comando após copiar o código abaixo no
seu módulo:

```bash
go mod tidy
```

Em seguida, use o seguinte código Go como exemplo:

```go
package main

import (
	"context"
	"fmt"
	"log"
	"os"

	"github.com/joho/godotenv"
	"go.mongodb.org/mongo-driver/bson"
	"go.mongodb.org/mongo-driver/mongo"
	"go.mongodb.org/mongo-driver/mongo/options"
)

func main() {
	// Carregar variáveis de ambiente do arquivo .env
	err := godotenv.Load()
	if err != nil {
		log.Fatal("Erro ao carregar o arquivo .env")
	}

	// Recuperar usuário e senha do arquivo .env
	mongoUser := os.Getenv("MONGO_USER")
	mongoPassword := os.Getenv("MONGO_PASSWORD")

	// Use the SetServerAPIOptions() method to set the version of the Stable API on the client
	serverAPI := options.ServerAPI(options.ServerAPIVersion1)
	opts := options.Client().ApplyURI(fmt.Sprintf("mongodb+srv://%s:%s@cluster-teste.q2pkwbd.mongodb.net/?retryWrites=true&w=majority&appName=Cluster-teste", mongoUser, mongoPassword)).SetServerAPIOptions(serverAPI)

	// Create a new client and connect to the server
	client, err := mongo.Connect(context.TODO(), opts)
	if err != nil {
		panic(err)
	}

	defer func() {
		if err = client.Disconnect(context.TODO()); err != nil {
			panic(err)
		}
	}()

	// Send a ping to confirm a successful connection
	if err := client.Database("admin").RunCommand(context.TODO(), bson.D{{"ping", 1}}).Err(); err != nil {
		panic(err)
	}
	fmt.Println("Pinged your deployment. You successfully connected to MongoDB!")
}
```

:::warning

Note que, para rodar o código acima, você precisa configurar um `.env` e usar a
biblioteca `godotenv`.

:::
