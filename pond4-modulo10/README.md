# Ponderada 4

## O que foi feito

Há dois microserviços de API disponíveis: um desenvolvido em Python baseado no modelo disponibilizado e outro criado em Golang. Nessa ponderada o Nginx foi usado como um gateway para o sistema, permitindo o acesso a ambas as APIs.

Para gerenciar os logs das requisições feitas através do Nginx, foi implementado um sistema baseado no Filebeat e no Elasticsearch. O Filebeat é responsável por coletar os logs e enviá-los ao Elasticsearch, onde são armazenados e indexados no formato chave-valor. Além disso, o Kibana foi configurado para facilitar a visualização desses logs.

## Como rodar o código:
1. Clone o repositório;
3. Rode o comando ```docker compose up```

## Video

[Link para o vídeo](https://drive.google.com/file/d/1AaXEzojvuONY7sjkH74jIQ4laI6xAt6y/view?usp=sharing)
