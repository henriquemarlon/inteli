---
title: Integração MongoDB Atlas
sidebar_position: 1
sidebar_class_name: autoestudo
slug: /atlas-metabase
---

# Integrando o mongoDB Atlas com o Metabase

:::warning

Para conseguir fazer a integração descrita nessa seção, você deve:

* Ter uma conta no MongoDB Atlas
* Ter uma instância do Metabase rodando (local ou em nuvem)

:::

## 1. Configurando o MongoDB Atlas

1. **Crie um Cluster**: Faça login na sua conta do [MongoDB
   Atlas](https://www.mongodb.com/cloud/atlas/register) e crie um novo cluster
   se ainda não tiver um. Para fazer isso, basta clicar no botão **CREATE** na
   tela inicial do seu mongodb Atlas.

<img
  src="./img/atlas1.png"
  alt="Relações DBs" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '50vh',
    marginRight: 'auto'
  }} 
/>
<br/>

A seguir, defina qual tipo de cluster você vai utilizar. Apenas o **M0** está
no **free tier** do Atlas.

<img 
  src="./img/atlas2.png"
  alt="Relações DBs" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '70vh',
    marginRight: 'auto'
  }} 
/>
<br/>

Pronto! Seu cluster está pronto. Agora, está na hora de:

2. **Definir o usuário que terá acesso ao seu MongoDB**. Note que aqui você
   pode escolher usar usuário/senha ou um certificado de segurança:

<img 
  src="./img/atlas4.png"
  alt="Relações DBs" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '50vh',
    marginRight: 'auto'
  }} 
/>
<br/>

Agora que nosso usuário está cadastrado, vamos:

3. **Configurar seu IP dentro da whitelist**. Aqui, para um setup local
   simples, basta clicar no botão **adicionar meu IP atual**

<img 
  src="./img/atlas5.png"
  alt="Relações DBs" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '50vh',
    marginRight: 'auto'
  }} 
/>
<br/>

:::warning

Note que não há como se conectar usando um VPC ou private endpoint quando está
usando uma free tier do Atlas. Sendo assim, a integração com qualquer serviço
de nuvem utilizado deve ser por um **ip público**. Caso o serviço que esteja
utilizando não forneça um ip público para utilizar, repense a escolha desse
serviço para utilização nesse projeto.

:::

E pronto. Agora você tem um cluster de mongoDB pronto para ser utilizado.

<img 
  src="./img/atlas6.png"
  alt="Relações DBs" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '50vh',
    marginRight: 'auto'
  }} 
/>
<br/>

Agora, vamos criar uma conexão com o nosso cluster. Para isso, vamos precisar
extrair a **URI** do cluster ou a sua **connection string**. Clique em
**Connect**. Na próxima tela, escolha a opção **Drivers**.

<img 
  src="./img/atlas7.png"
  alt="Relações DBs" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '70vh',
    marginRight: 'auto'
  }} 
/>
<br/>

A tela seguinte vai apresentar para vocês uma **connection string** para
utilizar com a sua aplicação. O **URI** pode ser extraído da própria connection
string (a string que vem depois do **@** na connection string).

<img 
  src="./img/atlas8.png"
  alt="Relações DBs" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '80vh',
    marginRight: 'auto'
  }} 
/>
<br/>

## 2. Configurando o Metabase

### 2.1. Configuração convencional

1. Faça login no Metabase e vá para "Admin" > "Databases" > "Add database".
2. Escolha "MongoDB" como o tipo de banco de dados.
3. Preencha os campos com as seguintes informações:
   - **Name**: Dê um nome para a sua conexão.
   - **Host**: Cole a URI para conexão com seu banco de dados. Ela vai ter um
     formato similar a: **nome-do-cluster.seu-id.mongodb.net**. Para
     encontrar seu URI vá para a seção de clusters do Atlas, clique em
     **Connect**, escolha **Drivers** e veja a URI utilizada na string de
     conexão fornecida.
   - **Database name**: O nome do banco de dados que você deseja conectar.
   - **Port**: Tipicamente a porta usada é a 27017
   - **Usuário**: O seu usuário cadastrado no Atlas
   - **Senha**: A senha do seu usuário cadastrado no Atlas
   - **Configure o metabase para usar DNS SRV**: Em **opções avançadas**
     assinale para o metabase usar o DNS SRV.
4. Clique em "Save" para salvar a configuração.

### 2.2. Configuração usando string de conexão do MongoDB Atlas

:::warning

Esse modo de configuração ainda não funcionou para mim. Vou continuar tentando,
mas se não der certo vou só deletar essa subseção.

:::

1. No MongoDB Atlas, vá para a seção "Clusters" e clique no botão "Connect" do
   seu cluster.
2. Escolha "Connect your application" como método de conexão.
3. Selecione o driver correto (geralmente "Java") e copie a string de conexão
   fornecida. Ela deve ter o formato:
   ```
   mongodb+srv://<username>:<password>@<cluster-url>/<dbname>?retryWrites=true&w=majority
   ```
4. Faça login no Metabase e vá para "Admin" > "Databases" > "Add database".
5. Escolha "MongoDB" como o tipo de banco de dados.
6. Escolha a opção **colar uma string de conexão** e cole a sua string que
   extraiu do Atlas.
7. Clique em "Save" para salvar a configuração.
