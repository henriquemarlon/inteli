# Frontend

### Descrição

O frontend é uma página HTML que fornece uma interface para:

- Fazer login
- Criar uma conta de usuário
- Criar uma postagem
- Visualizar todas as postagens feitas

#### Estilos

Os estilos são definidos internamente na página e consistem em classes como `.container`, `.container2`, `.alinha-centro` e estilos para elementos `input` e `button`.

#### Funcionalidades

O JavaScript incorporado na página fornece as seguintes funcionalidades:

- **login()**: Autentica um usuário e armazena o token JWT no armazenamento local.
- **createUser()**: Cria um novo usuário.
- **getAllPosts()**: Recupera todas as postagens feitas pelo usuário autenticado.
- **createPost()**: Permite ao usuário criar uma nova postagem.

### Backend

O backend é escrito em Rust usando o framework Warp. Ele serve o conteúdo HTML e permite CORS de qualquer origem.

#### Funcionalidades

- **html_content**: Serve o conteúdo HTML da página de autenticação.
- **warp::cors**: Permite solicitações CORS de qualquer origem.

### Notas

- O frontend faz solicitações para `http://localhost:8080`, certifique-se de que o serviço correspondente esteja em execução e escutando na porta 8080.
- O backend escuta na porta 3000.
- Lembre-se de configurar adequadamente o CORS se você planeja implantar esta aplicação em produção.

### Estrutura de pastas:

```
.
├── Cargo.toml
├── Dockerfile
├── .dockerignore
├── .gitignore
├── README.md
├── src
│   ├── index.html
    └── main.rs
```