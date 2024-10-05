---
title: Simulador IoT 1
sidebar_position: 4
sidebar_class_name: ponderada
slug: /ponderada1
---

# Simulador de dispositivos IoT

## 1. Objetivo

Criar um simulador de dispositivos IoT utilizando o protocolo MQTT através do
uso da biblioteca Eclipse Paho.

## 2. Enunciado

A primeira atividade do módulo tem como objetivo a criação de um simulador de
dispositivos IoT capaz de enviar informações em um tópico com o formato de
dados consistente com os seguintes dispositivos de exemplo:

[Sensor de Radiação
Solar](https://sigmasensors.com.br/produtos/sensor-de-radiacao-solar-sem-fio-hobonet-rxw-lib-900)

[SPS30](https://www.alldatasheet.com/view.jsp?Searchword=Sps30%20datasheet&gad_source=1&gclid=CjwKCAiA7t6sBhAiEiwAsaieYhkbnMeIYLDdvsNp4culvHNBwikKTM4QvT16S8ImRQTC9o1Dl6LQ-BoCe8cQAvD_BwE)

[MiCS-6814](https://datasheetspdf.com/pdf/1350171/SGX/MiCS-6814/1)

Escolha ao menos um desses sensores e estude o seu datasheed para reproduzir o
que seria uma mensagem MQTT gerada a partir dos dados aferidos por eles. Foque
na reprodução fidedigna de **taxa de comunicação**, **unidade** e **ordem de
grandeza** das medições. Utilize alguma técnica/biblioteca para que suas
mensagens simuladas não sejam todas iguais (mas todas dentro das especificações
do componente).

Embora não haja o requerimento de criar testes automatizados, o simulador deve
apresentar evidências objetivas de funcionamento.

## 3. Padrão de entrega

:::warning

Esses são os critérios mínimos para que eu considere a atividade como entregue.
Fique atento, pois o não cumprimento de qualquer um desses critérios pode, no
melhor dos casos, gerar um desconto de nota e, no pior deles, invalidar a
atividade.

:::

1. A atividade deve ser feita em um repositório aberto no github. Seu link deve
   ser fornecido no card da adalove;
2. No README do repositório deve ter instruções claras de como instalar e rodar
   o sistema criado, comandos em blocos de código e uma expliação sucinta do
   que fazem;
3. Ainda no README, deve haver um vídeo gravado demonstrando plenamente o
   funcionamento do nó criado;
4. O prazo para a entrega desta atividade é até o dia 18/02/2024 às 23h59min.

## 4. Padrão de qualidade

**[0,0 - 3,0]**
A implementação do simulador apresenta inconsistências técnicas
e/ou está fora do contexto requerido pela atividade.

**[3,0 - 7,0]**
Implementação aparentemente correta de um simulador de dispositivos IoT, mas
sem evidências de funcionamento.

**[7,0 - 9,0]**
Simulador de dispositivos IoT funcional, com evidências de seu funcionamento
claramente apresentadas através de testes manuais ou automatizados. Essas
evidências são apresentadas na documentação da ponderada (vídeo ou README).

**[9,0 - 10,0]**
Além de um simulador comprovadamente funcional, o sistema desenvolvido conta
com abstrações para que seja possível adequar o simulador para outros
dispositivos IoT (diferentes daqueles especificados pelo parceiro) sem a
necessidade de um refatoramento substancial.
