---
title: Sprint 2
sidebar_position: 2
sidebar_class_name: artefato
slug: /sprint2
---

# Artefatos Sprint 2 (Computação)

## 1. Versão inicial do sistema

### 1.1. Enunciado

Este artefato contempla a primeira entrega completa do sistema. Sendo assim, a
entrega deve contemplar os seguintes subsistemas:

* Interface de usuário;
* Backend (broker MQTT);
* Armazenamento de dados; e
* Implementação inicial de sistemas de segurança.

Para a interface de usuário, espera-se o desenvolvimento de um dashboard
simples. Não há a necessidade de implementar autenticação ou autorização. Para
o broker MQTT, espera-se o desenvolvimento de um broker já com autenticação e
autorização. Ainda não há restrições para o desenvolvimento do sistema de
armazenamento de dados.

**Tecnologias sugeridas**

* [Streamlit](https://streamlit.io/)
* [HiveMQ](https://www.hivemq.com/)
* [SQLite](https://github.com/eclipse/paho.mqtt.golang)

### 1.2. Padrão de entrega

:::warning

O padrão de entrega do artefato contempla uma série de requisitos para que a
atividade seja considerada concluída. Embora não haja atribuição direta de
pontuação aos itens aqui descritos, entenda-se que o **não cumprimento de
qualquer dos requisitos** pode, no melhor dos casos, acarretar em um **desconto
de nota** e, no pior caso, **invalidar a entrega por completo**. Fique atento!

:::

1. O código fonte da solução deve estar disponível no repositório do grupo no
   Github (na branch `main`), em um diretório denominado `src`. É provável que
   a sua solução não seja monolítica. Se esse for o caso, os módulos que compõe
   o seu código-fonte devem ser divididos em subdiretórios.
2. Os testes desenvolvidos devem estar disponíveis no diretório `tests` e,
   quando possível, aplicados de forma automatizada. Os testes realizados sem
   automatização devem ter seu procedimento de execução e resultados claramente
   descritos em um arquivo contido dentro do diretório de testes.
3. As instruções para que o parceiro possa executar o projeto devem estar
   claramente discriminadas na documentação, com um link no README para que a
   seção possa ser prontamente acessada.
4. O projeto em seu estado atual deve estar disponível em um release do Github
   cujo nome deve incluir a numeração da sprint (e.g. release-sprint-1).
5. As questões centrais de desenvolvimento da sprint e os testes devem ser
   apresentadas de forma clara durante o review com o parceiro.

### 1.3. Padrão de qualidade

#### 1.3.1 Broker MQTT (até 5,0 pontos)

**[0,0 - 1,0]**
O broker desenvolvido está substancialmente incompleto e/ou fora do contexto do
projeto e das especificações do parceiro.

**[1,0 - 2,0]**
Implementação aparentemente correta de um broker MQTT, mas sem evidências de
funcionamento e adequação às especificações do parceiro através de testes
objetivos. Sistema de autenticação e autorização ainda em estado incipiente ou
não implementados corretamente.

**[2,0 - 3,0]**
Broker MQTT funcional e com testes de funcionamento bem definidos e que passam
consistentemente. 

E ao menos uma das duas afirmações a seguir é verdadeira:

Sistemas de autenticação de usuário e autorização e proteção de tópicos
implementados, mas sem evidências de funcionamento e adequação às
especificações.

OU:

Integração entre o broker e os demais subsistemas implementada, mas sem
evidências de funcionamento e adequação às especificações.

**[3,0 - 4,0]**
Broker MQTT funcional e consistentemente passando nos testes de funcionamento.

E ao menos uma das duas afirmações a seguir é verdadeira:

A autenticação de usuários e autorização de acesso a tópicos foram
implementados e com testes que validem sua funcionalidade (passa
consistentemente nos testes). 

OU:

A integração entre o broker e os demais subsistemas foi implementada e conta
com testes que validam sua funcionalidade.

**[4,0 - 5,0]**
Broker MQTT implementado com sua funcionalidade básica, autorização e
autenticação funcionais e validados através de testes. Integração com os outros
subsistemas implementada corretamente e validada através de testes que passam
consistentemente.

#### 1.3.2 Dashboard (até 2,0 pontos)

**[0,0 - 0,5]**
O dashboard desenvolvido está substancialmente incompleto e/ou fora do contexto
do projeto e das especificações do parceiro.

**[0,5 - 1,0]**
O dashboard desenvolvido está ao menos em um nível de execução em que já é
possível compreender de forma clara a visão da equipe para a interface de
usuário, mas ainda não está completo. Não há integração alguma do dashboard com
as fontes de informação do projeto.

**[1,0 - 1,5]**
Dashboard completo e integrado aos outros subsistemas. Há poucos ou nenhum
teste que comprovem a funcionalidade da integração.

**[1,5 - 2,0]**
Dashboard completo e integrado aos outros subsistemas (broker/banco de dados).
Há testes para comprovar a integração e o sistema passa em todos os testes
consistentemente.

#### 1.3.3 Banco de dados (até 3,0 pontos)

**[0,0 - 0,5]**
O sistema de armazenamento de dados desenvolvido está substancialmente
incompleto e/ou fora do contexto do projeto e das especificações do parceiro.

**[0,5 - 1,5]**
Há evidências da modelagem de um sistema de banco de dados relacional, mas a
implementação está incompleta/contém erros não tratados. Há poucos ou nenhum
teste que comprovem o funcionamento correto do banco de dados.

**[1,5 - 2,0]**
Banco de dados modelado, implementado e com testes que comprovem o seu
correto funcionamento. A integração entre o banco de dados e os demais
subsistemas ainda está em fase inicial e/ou contem erros significativos.

**[1,5 - 2,5]**
Banco de dados implementado, passando em todos os testes de funcionamento
consistentemente e com desenvolvimento significativo na integração com os
outros subsistemas, mas com poucos ou nenhum teste que comprovem essa
integração e/ou falhando em um ou mais testes (de forma intermitente ou não).

**[2,5 - 3,0]**
Banco de dados implementado e passando em todos os testes de funcionamento e
integração de forma consistente.


## 2. Documentação 

### 2.1. Enunciado

O grupo deverá descrever a comunicação segura por meio do protocolo MQTT
utilizando diagramas de sequência UML. Também será necessário o registro de
onde os elementos da arquitetura estão implantados utilizando o diagrama de
implantação UML. Os testes de funcionamento, integração e segurança
implementados devem estar claramente documentados.

### 2.2. Padrão de entrega

:::warning

O padrão de entrega do artefato contempla uma série de requisitos para que a
atividade seja considerada concluída. Embora não haja atribuição direta de
pontuação aos itens aqui descritos, entenda-se que o não cumprimento de
qualquer dos requisitos pode, no melhor dos casos, acarretar em um desconto de
nota e, no pior caso, invalidar a entrega por completo. Fique atento!

:::

1. O texto desenvolvido pelo grupo deve estar disponível em uma página estática
   gerada pelo framework Docusaurus. Para tal, deve haver um diretório no
   repositório denominado `docs`, onde ficará a raíz do Docusaurus;
2. A documentação da sprint 2 deve estar inteiramente contida em uma seção
   denominada `Sprint 2`. Cada um dos artefatos deve ter sua descrição contida
   em uma página ou subseção dentro da seção `Sprint 2`.
3. As figuras utilizadas no documento devem sempre ser referenciadas no texto,
   com descrições textuais que estimulem a coesão entre o que é apresentado
   visualmente e o restante do texto; e 
4. Todas as figuras utilizadas na documentação devem estar salvas dentro do
   diretório `docs`, em um subdiretório chamado `static`.

### 2.3. Padrão de qualidade

#### 2.3.1 Qualidade textual (até 5,0 pontos)

:::warning

A qualidade textual à qual esse padrão de qualidade se refere diz respeito ao
que foi documentado no decorrer da sprint atual. No entanto, é prerrogativa do
professor orientador descontar nota por defeitos que persistiram de um sprint

:::

**[0,0 - 1,0]**
A documentação desenvolvida foge do contexto do projeto/está consideravelmente
incompleta.

**[1,0 - 2,0]**
A documentação desenvolvida aborda todos os temas obrigatórios (artefatos), mas
não apresenta-os de forma coesa, de modo que não seja possível vislumbrar o
objetivo conjunto do projeto. O documento apresenta uma linguagem
consistentemente inadequada para um documento de cunho técnico e/ou apresenta
mais de um estilo textual de forma dissonante (colcha de retalhos).

**[2,0 - 4,0]**
Documentação aborda todos os temas e apresenta-os com alguma coesão, dando um
entendimento ao menos parcial do projeto como um todo. Há poucas ou nenhuma
ocorrência de linguagem inadequada e o texto apresenta alguma consistência de
estilo.

**[4,0 - 5,0]**
Documentação completa e coesa do projeto desenvolvido. A linguagem utilizada é
condizente com o tipo de documento produzido.

#### 2.3.2 Testes (até 3,0 pontos)

**[0,0 - 0,5]**
Há pouca ou nenhuma documentação relacionada aos testes definidos e
implementados no decorrer do sprint.

**[0,5 - 2,5]**
Há documentação para os testes, mas estes estão incompletos e/ou não há testes que
abordem todos os requisitos relacionados à etapa atual do projeto.

**[2,5 - 3,0]**
Os testes estão suficientemente documentados e abordam todos os requisitos
relacionados à etapa atual do projeto.

#### 2.3.3 Instruções para execução (até 2,0 pontos)

**[0,0 - 0,5]**
Há pouca ou nenhuma instrução para execução do projeto em sua versão atual.

**[0,5 - 1,0]**
As instruções estão presentes e abordam de modo geral o que deve ser feito para
a execução do projeto, porém faltam detalhes que não podem ser facilmente
inferidos pelo leitor. As instruções apresentam uma falta de exemplos com
comandos claros para execução. Não há informações sobre compatibilidade do
software com versões de linguagens de programação, bibliotecas ou sistemas
operacionais.

**[1,0 - 1,5]**
Instruções completas e com comandos detalhados de execução, porém com poucos ou
nenhum dos exemplos mais comuns de utilização. Informações incompletas sobre
compatibilidade.

**[1,5 - 2,0]**
Instruções completas, com exemplos mais comuns, comandos detalhados de execução
e informações robustas de compatiblidade.
Requisitos contextualizados, com métricas bem definidas e com o planejamento de
testes devidamente documentado e coerente com os requisitos.
para outro.
