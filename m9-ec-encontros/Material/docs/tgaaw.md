---
title: Prova 1
sidebar_position: 15
slug: /the-gods-are-always-watching
unlisted: true
---

# P1 Módulo 9 EC 2024

## 1. Enunciado

Você está desenvolvendo uma ferramenta de linha de comando (CLI) para monitorar
a temperatura em uma cadeia de supermercados. Cada loja possui vários sensores
de temperatura espalhados em áreas críticas, como o setor de congelados e o
setor de produtos frescos. Esses sensores publicam regularmente as leituras de
temperatura em tópicos específicos do MQTT. Sua ferramenta deve se inscrever
nesses tópicos para coletar os dados e exibir as temperaturas em tempo real no
terminal. Além disso, deve emitir um alarme no terminal se a temperatura
estiver fora dos limites aceitáveis.

Os limites de temperatura são:
* Setor de Congelados (Freezer): Temperaturas acima de -15°C ou abaixo de -25°C
  devem acionar um alarme.
* Setor de Produtos Frescos (Geladeira): Temperaturas acima de 10°C ou abaixo
  de 2°C devem acionar um alarme.

Para simular a operação do sistema, você deve criar um pequeno simulador
(publisher) capaz de enviar mensagens simulando os sensores de temperatura.
Para tal, considere que o **payload** dos sensores inclui as seguintes informações:

1. O id do dispositivo (por esse id deve ser possível identificar a que loja
   pertence o sensor)
2. O tipo de dispositivo (freezer ou refrigerador)
3. O valor de temperatura medido
4. O timestamp de quando ocorreu a medição

Um exemplo de payload pode ser visto abaixo:

```json
{
  "id": "lj01f01",
  "tipo": "freezer",
  "temperatura": -18,
  "timestamp": "01/03/2024 14:30"
}
```

Exemplo de saída esperada no terminal (use como referência das informações que
precisam estar lá, não precisa ser necessariamente com a mesma formatação):

```bash
Lj 1: Freezer   1 | -18°C
Lj 2: Geladeira 3 |   5°C
Lj 1: Freezer   2 | -26°C [ALERTA: Temperatura BAIXA]
Lj 3: Geladeira 1 |  12°C [ALERTA: Temperatura ALTA]
```

O seu projeto deve obrigatóriamente apresentar **testes** para cada uma das
partes envolvidas no sistema. Os testes específicos que devem ser implementados
podem ser vistos no **padrão de qualidade** desta atividade.

Para esta atividade você pode escolher entre usar a linguagem **Go** ou
**Python**. Para simular a comunicação MQTT, sugere-se o uso do mosquitto. Os
testes desenvolvidos **não precisam** botar o broker para rodar.

## 2. Padrão de entrega

:::warning

Esses são os critérios mínimos para que eu considere a atividade como entregue.
Fique atento, pois o não cumprimento de qualquer um desses critérios pode, no
melhor dos casos, gerar um desconto de nota e, no pior deles, invalidar a
atividade.

:::

1. A atividade deve ser feita em um repositório **aberto** no github. Seu link
   deve ser fornecido na resposta do forms da prova;
2. No README do repositório deve ter instruções claras de como instalar e rodar
   o sistema criado, comandos em blocos de código e uma expliação sucinta do
   que fazem;
3. Ainda no README, deve haver um vídeo/imagens demonstrando plenamente o
   funcionamento do sistema criado;
4. O prazo para a entrega desta atividade é até o dia 08/03/2024 às 11h00min

## 3. Padrão de qualidade

:::warning

Note que metade da nota da atividade é composta por testes. Como vimos nos
conceitos de TDD, é possível e muitas vezes desejável criar testes antes de
desenvolver a aplicação. Sendo assim, o padrão de qualidade da nossa prova
prática permite que você apresente testes corretos sem que a aplicação em si
passe nesses testes. Use essa informação com sabedoria.

:::

### 3.1. Simulador (até 4.0 pontos)

#### 3.1.1. Funcionalidade

**[0,0 - 1,0]**
O simulador desenvolvido não se adequa aos sensores descritos

**[1,0 - 2,0]**
O simulador desenvolvido aparentemente atende os requisitos do payload descrito
e apresenta valores verossímeis.

#### 3.1.2. Testes

**[0,0 - 2,0]**
Foram implementados testes para validar: 

1. O correto envio das mensagens com QoS = 1; 
2. O payload recebido, garantindo que todos os campos necessários estejam
   presentes e preenchidos

### 3.2. Sistema de monitoramento (até 6.0 pontos)

#### 3.2.1. Funcionalidade

**[0,0 - 1,0]**
O sistema de monitoramento não apresenta uma ou mais informações necessárias
descritas no enunciado da atividade.

**[1,0 - 2,0]**
O sistema de monitoramento apresenta todas as informações necessárias descritas
no enunciado, mas os alarmes ainda não foram implementados ou foram
implementados de forma incompleta.

**[2,0 - 3,0]**
Sistema de monitoramento correto e com alarmes aparentemente funcionais.

#### 3.1.2. Testes

**[0,0 - 3,0]**
Foram implementados testes para validar: 

1. O correto recebimento das mensagens com QoS = 1;
2. A integridade das mensagens recebidas, garantindo que o subscriber não
   distorce as informações recebidas; e
3. O correto funcionamento de cada um dos tipos de alarmes necessários para o
   correto funcionamento do sistema (temperatura alta e baixa para os dois
   tipos de sensores).
