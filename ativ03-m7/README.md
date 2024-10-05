# Notex

Este projeto consiste em três serviços principais: um banco de dados PostgreSQL, um backend e um frontend, todos orquestrados usando Docker Compose.

## Pré-requisitos

- Docker
- Docker Compose

## Configuração

1. **Arquivo .env**:Rode o comando abaixo em um ambiente linux e certifique-se de ter um arquivo `.env` dentro do diretório `notex`. Para isso, rode (a criação deve ser manual para ambiente windows):

```bash
make env
```

2. Adicione como exemplo as seguintes variáveis ao arquivo ```.env``` criado dentro da pasta ```/notex```:

```bash
POSTGRES_DB="db"
POSTGRES_USER="user"
JWT_SECRET_KEY="secret"
POSTGRES_PASSWORD="password"
DATABASE_URL="postgres://user:password@postgres:5432/db"
```

Você pode ajustar os valores conforme necessário.

2. **Imagens Docker**: Este projeto utiliza as seguintes imagens Docker:

- `postgres:latest`
- `henriquemarlon/notex-backend:1.0.1`
- `henriquemarlon/notex-frontend:1.0.0`

Certifique-se de que você tem acesso a essas imagens ou ajuste o arquivo `docker-compose.yml` conforme necessário.

## Como Rodar

1. Navegue até o diretório raiz do projeto.

2. Inicie os serviços usando Docker Compose:

```bash
docker compose up
```

3. Após iniciar os serviços, você pode acessar:

- **Frontend**: `http://localhost:3000`
- **Backend**: `http://localhost:8080`
- **Banco de Dados**: `http://localhost:5432`

4. Para parar os serviços:

```bash
docker compose down -v
```

## Volumes

Este projeto utiliza um volume chamado `postgres_data` para persistir os dados do banco de dados PostgreSQL.

## Arquitetura da Aplicação:
<p align="center">
  <img src="./arquitetura.jpeg" alt="Arquitetura da Solução">
</p>

## Detalhamento:

### Arquitetura da Solução:

#### 1. **PostgreSQL**:
- **Imagem**: A solução utiliza a imagem oficial `postgres:latest` do Docker Hub.
- **Portas**: A porta padrão do PostgreSQL, 5432, é mapeada para a mesma porta no host.
- **Configuração**: As variáveis de ambiente para o serviço PostgreSQL são definidas usando um arquivo `.env` localizado em `./notex/.env`. Se as variáveis não estiverem definidas no arquivo `.env`, valores padrão são usados (por exemplo, `POSTGRES_DB` tem um valor padrão de "db").
- **Persistência**: Um volume chamado `postgres_data` é usado para persistir os dados do banco de dados. Este volume é montado no diretório `/var/lib/postgresql/data` dentro do container.
- **Healthcheck**: Um healthcheck é configurado para verificar a saúde do serviço PostgreSQL. Ele usa o comando `pg_isready` para verificar se o banco de dados está pronto para aceitar conexões.

#### 2. **Backend**:
- **Imagem**: A solução utiliza uma imagem personalizada `henriquemarlon/notex-backend:1.0.1` para o backend.
- **Linguagem**: O backend é escrito em Rust e não utiliza nenhum framework específico. Ele é responsável por interagir com o banco de dados PostgreSQL e fornecer APIs ou endpoints necessários.
- **Portas**: A porta 8080 do container é mapeada para a mesma porta no host.
- **Dependência**: O serviço backend depende do serviço PostgreSQL. Ele só será iniciado quando o serviço PostgreSQL estiver saudável (graças à condição `service_healthy`).

#### 3. **Frontend**:
- **Imagem**: A solução utiliza uma imagem personalizada `henriquemarlon/notex-frontend:1.0.0` para o frontend.
- **Conteúdo**: Esta imagem serve os arquivos estáticos (HTML, JS, CSS) que estão em um arquivo `index.html`.
- **Linguagem e Framework**: O frontend é escrito em Rust e utiliza o framework `warp` para servir a página web e os arquivos estáticos.
- **Portas**: A porta 3000 do container é mapeada para a mesma porta no host.
- **Dependência**: O serviço frontend depende do serviço backend. Ele só será iniciado após o backend estar em execução.

#### **Orquestração com Docker Compose**:
- **Versão**: A versão do Docker Compose usada é a "3.9".
- **Volumes**: Além dos serviços, um volume chamado `postgres_data` é definido no nível raiz do arquivo Docker Compose para persistir os dados do PostgreSQL.

Em resumo, esta arquitetura define uma solução completa de três partes: um banco de dados PostgreSQL, um backend escrito em Rust sem o uso de frameworks, e um frontend escrito em Rust com o framework `warp` servindo conteúdo estático. Tudo isso é orquestrado usando Docker Compose, garantindo que os serviços sejam iniciados na ordem correta e com as dependências necessárias.
