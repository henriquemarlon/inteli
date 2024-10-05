package main

import (
	"context"
	"fmt"
	"log"

	"go.mongodb.org/mongo-driver/bson"
	"go.mongodb.org/mongo-driver/mongo"
	"go.mongodb.org/mongo-driver/mongo/options"
)

func main() {
	// Conectar ao servidor MongoDB (por padrão, na porta 27017 do localhost)
	client, err := mongo.Connect(context.TODO(), options.Client().ApplyURI("mongodb://localhost:27017/"))
	if err != nil {
		log.Fatal(err)
	}

	// Selecionar o banco de dados 'meu_banco_de_dados'
	db := client.Database("meu_banco_de_dados")

	// Selecionar a coleção 'minha_colecao'
	colecao := db.Collection("minha_colecao")

	// Canal para sincronizar a inserção e a busca
	done := make(chan bool)

	// Inserir um documento na coleção de forma assíncrona
	go func() {
		documento := bson.D{{"nome", "Fulano"}, {"idade", 30}}
		resultadoInsercao, err := colecao.InsertOne(context.TODO(), documento)
		if err != nil {
			log.Fatal(err)
		}
		fmt.Printf("Documento inserido com o ID: %v\n", resultadoInsercao.InsertedID)
		done <- true
	}()

	// Aguardar a inserção ser concluída
	<-done

	// Buscar o documento inserido de forma assíncrona
	go func() {
		var documentoEncontrado bson.M
		err = colecao.FindOne(context.TODO(), bson.D{{"nome", "Fulano"}}).Decode(&documentoEncontrado)
		if err != nil {
			log.Fatal(err)
		}
		fmt.Printf("Documento encontrado: %v\n", documentoEncontrado)
		done <- true
	}()

	// Aguardar a busca ser concluída
	<-done

	// Fechar a conexão com o banco de dados
	err = client.Disconnect(context.TODO())
	if err != nil {
		log.Fatal(err)
	}
}
