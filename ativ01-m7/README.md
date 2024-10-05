## Ponderada 01 módulo 7

### Descrição:
- O código deste repositório contempla a implmentação de um currículo servido via container por meio do Gunicorn. Além disso ao final desse README está contido o link do docker hub por meio do qual qualuqer pessoa pode fazer pull da imagem.

### Link da imagem no docker hub:
- Acesse: https://hub.docker.com/r/henriquemarlon/portfolio

### Intruções para rodar o projeto a partir do DockerHub:
- Use o comando para dar pull na imagem:
```bash
docker pull henriquemarlon/portfolio:0.1
```

- Use o comando para rodar o container:
```bash
docker run -p 8000:8000 henriquemarlon/portfolio:0.1
```
