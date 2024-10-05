---
title: Go
sidebar_position: 4
sidebar_class_name: opcional
slug: /go
---

# Configuração do Go

## 1. Instalando o Go

Antes de mais nada faça o [download](https://go.dev/dl/) do Go. Procure pelo
arquivo zipado do seu sistema operacional e arquitetura adequadas (esse
tutorial vai assumir que está usando Linux amd64/x86_64).

Após baixar o arquivo, navegue até a pasta onde ele está e rode o seguinte
comando, que deleta qualquer versão antiga do Go que possa ter no seu sistema e
descompacta o arquivo zip que você baixou em `/usr/local/go`.

```bash
sudo rm -rf /usr/local/go && sudo tar -C /usr/local -xzf go1.21.6.linux-amd64.tar.gz
```

Calma que ainda não acabou! Para que você consiga acessar o binário do Go, você
precisa adicioná-lo ao **PATH** do sistema operacional. Isso é possível com o
comando:

```bash
export PATH=$PATH:/usr/local/go/bin
```

:::warning

Note que o comando acima **NÃO** é persistente (se você reiniciar o computador,
vai ter que fazer de novo). Para que se torne persistente, adicione a linha
acima em um arquivo de configuração de ambiente (`/etc/environment`, `.bashrc`,
`.zshrc`, `.zshenv`)

:::

Se tudo deu certo, o comando abaixo vai rodar sem erro

```bash
go version
```

## 2. Primeiros passos Go

### 2.1. Ola mundo

Vamos criar um primeiro programa com Go? Para isso, crie um diretório novo para
ser a raíz do projeto.

```bash
mkdir hello
cd hello
```

A seguir, vamos utilizar a ferramenta `go mod` para criar a estrutura do
projeto.

```bash
go mod init example/hello
```

A seguir, crie um arquivo chamado `hello.go` e adicione o seguinte conteúdo:

```go showLineNumbers title="hello.go"
package main

import "fmt"

func main() {
    fmt.Println("Hello, World!")
}
```

Para rodar o projeto, use:

```bash
go run .
```

### 2.2. Usando bibliotecas externas

Uma das maiores vantagens de se utilizar o Go é a **facilidade** para utilizar
bibliotecas externas. Vamos modificar nosso código para em vez de `Hello,
World` ele exibir uma citeção usando a biblioteca `rsc.io/quote`. Para
adicioná-la ao projeto, basta atualizar o código e usar um comando para baixar
todas as dependências.

```go showLineNumbers title="hello.go"
package main

import "fmt"

// highlight-next-line
import "rsc.io/quote"

func main() {
    // highlight-next-line
    fmt.Println(quote.Go())
}
```

Para instalar todas as dependências do projeto automaticamente, use

```bash
go mod tidy
```

Esse comando consegue procurar e baixar qualquer biblioteca que esteja no
domínio **golang.org** ou no **Github** (sim, os módulos publicados para Go são
só repositório do github e o `go tidy` ou `go get` baixam e configuram
automaticamente o módulo). Agora basta só rodar de novo o módulo `hello`:

```bash
go run .
```

## 3. Tutorial Go + TDD

Nesse momento não tenho material próprio sobre Go, então segue um livro aberto
sobre Go e TDD (vocês vão notar que Go é uma linguagem extremamente receptiva
ao TDD):

https://quii.gitbook.io/learn-go-with-tests/
