# Backend

## Descrição

Este código é um servidor simples escrito em Rust que lida com operações de usuário e postagem. Ele utiliza autenticação JWT, bcrypt para hashing de senha e se conecta a um banco de dados PostgreSQL para armazenar informações de usuário e postagem.

## Dependências

- `bcrypt`: Para hashing e verificação de senhas.
- `jsonwebtoken`: Para criar e validar tokens JWT.
- `postgres`: Para conectar e interagir com o banco de dados PostgreSQL.
- `serde_derive`: Para serialização e desserialização de estruturas.
- `dotenv_codegen`: Para carregar variáveis de ambiente.

## Estruturas

- `RequestData`: Representa os dados da solicitação com cabeçalho e corpo.
- `Claims`: Representa as reivindicações do token JWT.
- `User`: Modelo para informações do usuário com id, nome, email e senha.
- `Post`: Modelo para informações da postagem com id, título, conteúdo e id do autor.

## Constantes

- `SECRET`: Chave secreta para JWT.
- `DB_URL`: URL do banco de dados.
- Respostas HTTP padrão como `OK_RESPONSE`, `NOT_FOUND`, `INTERNAL_SERVER_ERROR` e `UNAUTHORIZED_RESPONSE`.

## Funções Principais

- `main()`: Inicializa o banco de dados e começa a ouvir as solicitações.
- `handle_client()`: Lida com solicitações de clientes e roteia para a função apropriada.
- `create_user()`: Cria um novo usuário no banco de dados.
- `login()`: Autentica um usuário e retorna um token JWT.
- `create_post()`: Cria uma nova postagem no banco de dados.
- `get_all_posts_by_author()`: Recupera todas as postagens de um autor específico.
- `set_database()`: Configura o banco de dados e cria tabelas se não existirem.
- `generate_token()`: Gera um token JWT para um usuário.
- `validate_token()`: Valida um token JWT.
- `store_password()`: Hashes uma senha.
- `verify_password()`: Verifica uma senha contra sua versão hash.
- `get_header_and_body_request()`: Extrai cabeçalho e corpo de uma solicitação.
- `extract_token_from_header()`: Extrai o token JWT do cabeçalho da solicitação.

## Estrutura de pastas:

```
.
├── Cargo.toml
├── Dockerfile
├── .dockerignore
├── .gitignore
├── README.md
└── src
    └── main.rs
```