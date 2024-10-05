# Prova 01

O repositorío contém o código fonte correnpondente a uma API de grau 2 de maturidade segundo o Modelo de Maturidade de Richardson.

## Como rodar:

Os comandos estão sendo abstraídos por um arquivo Docker Compose que orquestra dois serviços em seus respectivos continers, o a api, identificada como backend, e o banco de dados, identificado como postgres. Para rodar, siga as intruções abaixo:

### Comando:

```bash
$ make run
```

### Output:

```bash
================================================= START OF LOG ===================================================
[+] Running 15/15
 ✔ postgres 14 layers [⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿]      0B/0B      Pulled                                                                12.3s 
   ✔ 09f376ebb190 Pull complete                                                                                               2.8s 
   ✔ 119215dfb3e3 Pull complete                                                                                               0.5s 
   ✔ e02bbc8c8252 Pull complete                                                                                               1.2s 
   ✔ 061f31803c55 Pull complete                                                                                               1.3s 
   ✔ accd4903f49a Pull complete                                                                                               2.3s 
   ✔ 2016ff8e6e3a Pull complete                                                                                               2.3s 
   ✔ 088e651df7e9 Pull complete                                                                                               2.9s 
   ✔ ed155773e5e0 Pull complete                                                                                               2.8s 
   ✔ ffebb35d2904 Pull complete                                                                                               8.0s 
   ✔ 293f0bec643a Pull complete                                                                                               3.4s 
   ✔ 1655a257a5b5 Pull complete                                                                                               3.3s 
   ✔ 4ddba458499d Pull complete                                                                                               3.9s 
   ✔ 90e48ae03559 Pull complete                                                                                               4.1s 
   ✔ 822c1a513e6a Pull complete                                                                                               4.4s 
[+] Building 2.0s (12/12) FINISHED                                                                            docker:desktop-linux
 => [backend internal] load .dockerignore                                                                                     0.0s
 => => transferring context: 667B                                                                                             0.0s
 => [backend internal] load build definition from Dockerfile                                                                  0.0s
 => => transferring dockerfile: 1.57kB                                                                                        0.0s
 => [backend] resolve image config for docker.io/docker/dockerfile:1                                                          0.8s
 => CACHED [backend] docker-image://docker.io/docker/dockerfile:1@sha256:a57df69d0ea827fb7266491f2813635de6f17269be881f696fb  0.0s
 => [backend internal] load metadata for docker.io/library/python:3.10.4-slim                                                 0.5s
 => [backend base 1/5] FROM docker.io/library/python:3.10.4-slim@sha256:557745c5e06c874ba811efe2e002aff21b6cc405b828952fcfa1  0.0s
 => [backend internal] load build context                                                                                     0.0s
 => => transferring context: 623B                                                                                             0.0s
 => CACHED [backend base 2/5] WORKDIR /app                                                                                    0.0s
 => CACHED [backend base 3/5] RUN adduser     --disabled-password     --gecos ""     --home "/nonexistent"     --shell "/sbi  0.0s
 => CACHED [backend base 4/5] RUN --mount=type=cache,target=/root/.cache/pip     --mount=type=bind,source=requirements.txt,t  0.0s
 => [backend base 5/5] COPY . .                                                                                               0.0s
 => [backend] exporting to image                                                                                              0.0s
 => => exporting layers                                                                                                       0.0s
 => => writing image sha256:7b579ff1d8924ef22af69fdd268cf725d39296567d6ec1b060bacfd81319db00                                  0.0s
 => => naming to docker.io/library/p1-m10-backend                                                                             0.0s
[+] Running 4/4
 ✔ Network p1-m10_default         Created                                                                                     0.0s 
 ✔ Volume "p1-m10_postgres_data"  Created                                                                                     0.0s 
 ✔ Container postgres             Healthy                                                                                     0.0s 
 ✔ Container backend              Started                                                                                     0.0s 
================================================== END OF LOG ====================================================
```

## Demonstração do sistema:

[link para o vídeo](https://drive.google.com/file/d/1X0EN9wxSyJNwRjIzyu0Ux5oQ00kxYPc-)