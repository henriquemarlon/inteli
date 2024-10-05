---
title: Sprint 3
sidebar_position: 3
sidebar_class_name: artefato
slug: /sprint3
---

# Artefatos Sprint 3 (Computação)

## 1. Evolução do banco de dados e dashboard

### 1.1. Enunciado

Este artefato contempla a melhoria do sistema inicial desenvolvido, dessa vez
focando-se em modificar o banco de dados e o dashboard desenvolvido para as
suas versões finais.

Para o banco de dados, espera-se a utilização de um sistema de armazenamento em
núvem com implementações de segurança. Para o dashboard, espera-se uma evolução
na proposta visual e implementação de autenticação para visualização dos dados.

**Tecnologias sugeridas**

* [MongoDB
  Atlas](https://www.mongodb.com/pt-br/cloud/atlas/lp/try4?utm_source=google&utm_campaign=search_gs_pl_evergreen_atlas_core_prosp-brand_gic-null_amers-br_ps-all_desktop_pt-br_lead&utm_term=mongodb%20atlas&utm_medium=cpc_paid_search&utm_ad=e&utm_ad_campaign_id=20378068769&adgroup=154980291281&cq_cmp=20378068769&gad_source=1&gclid=CjwKCAiAlcyuBhBnEiwAOGZ2Sw-edSjIGdGbw2b-luoyVovhkqu3cLq_AkX_t4KC8XfY_8TWogxUaRoCpu4QAvD_BwE)
* [Metabase](https://www.mongodb.com/pt-br/cloud/atlas/lp/try4?utm_source=google&utm_campaign=search_gs_pl_evergreen_atlas_core_prosp-brand_gic-null_amers-br_ps-all_desktop_pt-br_lead&utm_term=mongodb%20atlas&utm_medium=cpc_paid_search&utm_ad=e&utm_ad_campaign_id=20378068769&adgroup=154980291281&cq_cmp=20378068769&gad_source=1&gclid=CjwKCAiAlcyuBhBnEiwAOGZ2Sw-edSjIGdGbw2b-luoyVovhkqu3cLq_AkX_t4KC8XfY_8TWogxUaRoCpu4QAvD_BwE)

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

#### 1.3.1. Banco de dados em nuvem (até 6,0 pontos)

1. O sistema de armazenamento em nuvem deve demonstrar os seguintes critérios
   de segurança: confidencialidade, integridade e autenticidade. Ao menos um
   teste de ataque deve alvejar cada um desses requisitos (até 2,5 pontos);
2. Os testes devem demonstrar que um dispositivo IoT (real ou simulado) é capaz
de enviar dados periódicamente, validando o armazenamento no banco de dados e a
exibição no dashboard (até 2,5 pontos);
3. Ao menos um teste de ataque deve alvejar o sistema de autenticação do
   dashboard (até 2,5 pontos);
4. Os testes de segurança (ataques) devem apresentar pré-condição, etapas do
   teste, e pós-condição (até 2,5 pontos).

#### 1.3.2. Nova versão da dashboard (até 4,0 pontos)

**[0,0 - 1,0]**
O dashboard desenvolvido está substancialmente incompleto e/ou fora do contexto
do projeto e das especificações do parceiro.

**[1,0 - 2,0]**
O dashboard desenvolvido apresenta uma evolução com relação à entrega anterior,
observando o feedback dado pelo parceiro. A integração da dashboard está feita
de forma correta (com testes que comprovam isso). Não há autenticação para
acesso ao dashboard.

**[2,0 - 3,0]**
Evolução da dashboard com testes de integração e implementação de autenticação,
porém com poucos ou nenhum teste que evidenciem o funcionamento do sistema de
autenticação.

**[3,0 - 4,0]**
Dashboard completa com testes de integração, sistema de autenticação e testes
do sistema de autenticação.

## 2. Documentação 

### 2.1. Enunciado

O grupo deverá registrar a evolução do sistema de armazenamento, apresentando
diagramas de implantação/sequência UML e testes de cada novo componente, para a
adição do armazenamento em nuvem com segurança. Também deve descrever o
desenvolvimento do dashboard feito sob medida e seu sistema de autenticação de
usuários.

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
