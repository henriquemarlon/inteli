# TO-DO Tasks: API Async

## Como rodar 

1. No mesmo diretório rode o comando abaixo:

```bash
make run
```

## Endpoints criados

| Tipo de Requisição | Endpoint                 | Descrição                   |
|--------------------|--------------------------|-----------------------------|
| POST               | /api/v1/createUser       | Criar um novo usuário      |
| POST               | /api/v1/login            | Fazer login                |
| GET                | /api/v1/getTasks/{userID}| Obter tarefas por ID       |
| POST               | /api/v1/createTask/{userID}| Criar uma nova tarefa    |
| PUT                | /api/v1/editTask         | Editar uma tarefa          |
| DELETE             | /api/v1/deleteTask       | Excluir uma tarefa         |

## Teste da aplicação utilizando o Insomnia

Acesse o teste em [link](https://drive.google.com/file/d/10pIgqP_aOaqBWBJEJ3LrzfDjPqGKpFAW/view?usp=sharing)

- `controllers` : Possuem funções intermediárias entre as requisições feitas e a manipulação do banco de dados. Os controllers foram divididos em task, que possui funções de manipulação das tarefas e user, que possui funções relacionadas aos usuários.
- `models`: Dentro de models existem os arquivos reponsáveis pela manipulação direta do banco de dados, sendo eles dividos na mesma lógica dos controllers, entre user e task. Além disso, nessa pasta existe um subpasta chamada migrations que possui o SQL utilizado para a criação das tabelas no banco de dados. Por fim, também existe um arquivo models.go que possui funções auxiliares de comunicação com banco de dados e criação de tokens JWT.
- `static`: Possui a collection criada validação da API pelo Insomnia.
- `Dockerfile` e `compose.yaml`: O `Dockerfile` cria a imagem da API e o `compose.yaml` orquestra a imagem da API e de um postgress em containers.
- `main.go`: Ponto de início da API. Possui a definição das rotas criadas.
- `go.mod`e `go.sum`:  Arquivos de configuração do ambiente de um projeto Golang.