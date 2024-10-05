# README.md

Avaliações 1 módulo 7

Para executar a aplicação:
```bash
docker compose up
```

### Dockerfile do backend:
- Foi escolhido para o backend uma imagem python que instala os requirements, nesse caso o fastapi e o uvicorn. No arquivo docker-compose.yml essa imagem vai rodar na porta 8000.

### Dockerfile do frontend:
- Foi escolhid uma imagem node que instalar todos os pacotes do package.json. Essa imagem vai rodar na porta 3000.

### Docker Compose file:
- O fluxo aqui é simples:
  - Esse arquivo rodas as imagens subidas no dockerhub.
  - Coloca uma dependência de que o fronend só pode ser rodadado após o backend.
