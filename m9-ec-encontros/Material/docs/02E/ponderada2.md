---
title: Simulador IoT 2
sidebar_position: 5
sidebar_class_name: ponderada
slug: /ponderada2
---

# Teste de um simulador de dispositivos IoT

## 1. Enunciado

Utilizando o simulador de dispositivos IoT desenvolvido na atividade passada e
utilizando os conceitos de TDD vistos no decorrer da semana, implemente testes
automatizados para validar o simulador. Seus testes obrigatoriamente devem
abordar os seguintes aspectos:

1. Recebimento - garante que os dados enviados pelo simulador são recebidos
   pelo broker.
2. Validação dos dados - garante que os dados enviados pelo simulador chegam
   sem alterações.
3. Confirmação da taxa de disparo - garante que o simulador atende às
   especificações de taxa de disparo de mensagens dentro de uma margem de erro
   razoável.

## 2. Padrão de entrega

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
   funcionamento do sistema criado;
4. O prazo para a entrega desta atividade é até o dia 25/02/2024 às 23h59min.

## 3. Padrão de qualidade

**[0,0 - 3,0]**
Testes mal implementados e/ou que não validam os requisitos especificados na
atividade.

**[3,0 - 7,0]**
Testes definidos de acordo com os requisitos especificados, mas há uma
quantidade considerável de testes não passam de forma consistente.

**[7,0 - 9,0]**
Testes bem definidos e com a maioria passando consistentemente. Não há nada
implementado para garantir QoS.

**[9,0 - 10,0]**
Testes bem definidos e que passam consistentemente. Há pelo menos um teste
definido para validar o perfil de QoS definido em tempo de execução para o
simulador.

