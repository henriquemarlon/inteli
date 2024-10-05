---
title: Sprint 4
sidebar_position: 4
sidebar_class_name: artefato
slug: /sprint4
---

# Artefatos Sprint 4 (Computação)

## 1. Gerenciamento de Escalabilidade

### 1.1. Enunciado

O grupo deverá modificar sua solução, adequando o sistema de modo a comportar
um volume de dados compatível com o cenário de cidades inteligentes. Para isso,
deve-se implementar tecnologia de processamento de eventos complexos (e.g.
Kafka). Além disso, deve-se considerar o requisito de escalabilidade
previamente definido e desenvolver testes capazes de demonstrar que o sistema
aguenta o volume de dados especificado.

### 1.2. Padrão de entrega

Esses são os critérios mínimos para que eu considere a atividade como entregue.
Fique atento, pois o não cumprimento de qualquer um desses critérios pode, no
melhor dos casos, gerar um desconto de nota e, no pior deles, invalidar a
atividade.

1. O código fonte da solução deve estar disponível no repositório do grupo no
   Github (na branch 'main'), em uma pasta denominada 'src'.
2. Os testes desenvolvidos devem estar disponíveis na pasta 'tests' e, quando
   possível, aplicados de forma automatizada (e.g., utilizando o Github
   Actions). Quando não for possível essa integração, deve-se descrever os
   testes e os resultados em um arquivo markdown dentro da pasta 'tests'. 
3. As instruções para que o parceiro possa executar o projeto devem estar
   claramente discriminadas na documentação, com um link no README para que a
   seção possa ser prontamente acessada.
4. O projeto em seu estado atual deve estar disponível em um release do Github
   cujo nome deve incluir a numeração da sprint.
5. As questões centrais de desenvolvimento da sprint e os testes devem ser
   apresentadas de forma clara durante o review com o parceiro.

### 1.3. Padrão de qualidade

1. O sistema deve manter a integridade dos dados mesmo quando exposto a um alto
   volume de requisições. Deve-se desenvolver testes pautados pelos requisitos
   não funcionais para comprovar essa característica (até 2,5 pontos);
2. O sistema deve garantir um tempo de resposta adequado mesmo quando exposto a
   um alto volume de requisições. Deve-se desenvolver testes pautados pelos
   requisitos não funcionais para comprovar essa característica (até 2,5
   pontos);
3. O sistema deve manter uma alta taxa de disponibilidade mesmo quando exposto
   a um alto volume de requisições. Deve-se desenvolver testes pautados pelos
   requisitos não funcionais para comprovar essa característica (até 2,5
   pontos);
4. Os testes devem apresentar pré-condição, etapas do teste, e pós-condição
   (2,5 pontos).

## 2. Documentação 

### 2.1. Enunciado

O grupo deverá registrar a evolução de escalabilidade em um relatório técnico.

### 2.2. Padrão de entrega

1. O texto desenvolvido pelo grupo deve estar disponível no repositório do
   Github (branch 'main'), dentro da pasta 'docs' e em formato markdown;
2. As figuras utilizadas no documento devem sempre ser referenciadas no texto,
   com descrições textuais que estimulem a coesão entre o que é apresentado
   visualmente e o restante do texto;
3. Todas as figuras utilizadas na documentação devem estar salvas dentro da
   pasta 'docs', em uma subpasta chamada 'assets'.

### 2.3. Padrão de qualidade

1. O relatório deve apresentar uma versão revisada da arquitetura com uma
   descrição textual e usando o diagrama de implantação UML (até 2,5 pontos);
2. O relatório deve referenciar os testes de cada novo componente implementado,
   e os requisitos associados a cada novo componente (até 2,5 pontos);
3. O relatório deve descrever detalhadamente o procedimento de teste de demanda
   do sistema, revisando as métricas estabelecidas nos requisitos não
   funcionais e garantindo o seu atendimento (até 2,5 pontos);
4. O relatório deve apresentar pelo menos um diagrama de sequência revisado
   mostrando a adição do sistema de processamento de eventos complexos (até 2,5
   pontos).
