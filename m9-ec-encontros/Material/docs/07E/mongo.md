---
title: MongoDB
sidebar_position: 2
sidebar_class_name: autoestudo
slug: /mongo
---

import Admonition from '@theme/Admonition';
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Introdução ao MongoDB

<img 
  src="https://images.idgesg.net/images/article/2021/06/document-store-100893897-large.jpg?auto=webp&quality=85,70"
  alt="MongoDB" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '40vh',
    marginRight: 'auto'
  }} 
/>
<br/>

**MongoDB** é um sistema de banco de dados **NoSQL** orientado a
**documentos**, que armazena dados em um formato semelhante ao JSON, chamado
BSON. É conhecido por sua flexibilidade e escalabilidade, tornando-o uma
escolha popular para aplicativos modernos e em tempo real.

## 1. Elementos do MongoDB
- **Database**: É o contêiner de alto nível que armazena coleções de
  documentos. Um único servidor MongoDB pode conter vários bancos de dados.
- **Collections**: São agrupamentos de documentos, semelhantes às tabelas em
  bancos de dados relacionais. Uma coleção pode conter documentos com esquemas
  diferentes.
- **Documentos**: São registros individuais armazenados em uma coleção,
  representados em formato BSON. Cada documento possui um campo "_id" único que
  atua como chave primária.

## 2. Primeiros passos com MongoDB

### 2.1. Instalação

:::tip

Embora haja duas formas de instalação para o mongoDB (container ou pacote de
sistema), sugiro **fortemente** instalar utilizando o container.

:::

#### 2.1.1. Usando containers

Para rodar o mongodb utilizando containers, primeiro precisamos baixar a imagem
do container:

```bash
docker pull mongodb/mongodb-community-server:6.0-ubi8
```

:::warning

Note que o comando acima configura a versão `6.0-ubi8` do mongodb. Tipicamente
utilizamos a tag `latest` para esse tipo de imagem. Infelizmente, essa tag não
está disponível para essa imagem.

<img 
  src="https://criticalhits.com.br/wp-content/uploads/2020/05/Hunter-X-Hunter.jpg"
  alt="Sad Gon =(" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '30vh',
    marginRight: 'auto'
  }} 
/>
<br/>

:::

A seguir, vamos rodar o container com: 

```bash
docker run --name mongodb -d -p 27017:27017 mongodb/mongodb-community-server:6.0-ubi8
```

Isso vai fazer com o que mongodb fique disponível na porta **27017**. A partir
de agora, você pode interagir com a instância do mongodb utilizando o
`mongosh`. A instalação do `mongosh` estará na próxima seção.

#### 2.1.2. Instalando o pacote do mongodb

Usar o mongodb instalando o pacote é tão simples quanto usar o seu gerenciador
de pacotes para instalá-lo e posteriormente iniciar um serviço do **systemd**

Primeiro vamos instalar os pacotes necessários:

<Tabs>
  <TabItem value="arch" label="Arch Linux" default>
  ```bash
  paru -S mongodb-bin mongosh-bin
  ```
  </TabItem>
  <TabItem value="ubuntu" label="Ubuntu">
  ```bash
  # Adiciona chave publica do repositório ao apt
  wget -qO - https://www.mongodb.org/static/pgp/server-3.0.asc | sudo apt-key add -
  # Adiciona o repositório do mongodb à lista de sources
  echo "deb http://repo.mongodb.org/apt/ubuntu precise/mongodb-org/3.0 multiverse" | sudo tee /etc/apt/sources.list.d/mongodb-org-3.0.list
  # Atualiza os repositórios do apt
  sudo apt update
  # Instala o metapacote mongodb-org
  sudo apt install mongodb-org
  ```
  </TabItem>
</Tabs>

A seguir, habilite o serviço do mongodb usando:

```bash
sudo systemctl enable --now mongodb
```

#### 2.1.3. Mongosh

Para testar nossa instalação e seguir para a etapa de operações e comandos
básicos do mongodb, você deve executar o **mongo shell**; esta é a aplicação de
terminal que permite a interação com o seu banco de dados mongodb.

```bash
mongosh
```

### 2.2. Operações básicas

:::warning

Atenção!! Os comandos abaixo só vão funcionar se você estiver rodando o
**mongosh**

Lembrando que, para rodar o **mongosh**, use:

```bash
mongosh
```

:::

#### 2.2.1. **Conectar ao MongoDB:**
   ```shell
   mongosh "mongodb://localhost:27017"
   ```

#### 2.2.2. **Listar bancos de dados:**
   ```javascript
   show dbs
   ```

#### 2.2.3. **Listar coleções:**
   ```javascript
   show collections
   ```

#### 2.2.4. **Sair do `mongosh`:**
    ```shell
    exit
    ```

#### 2.2.5. **Criar um banco de dados:**
   ```javascript
   use nomeDoBancoDeDados
   ```

#### 2.2.6. **Criar uma coleção:**
   ```javascript
   db.createCollection("nomeDaColecao")
   ```

#### 2.2.7. **Inserir documentos:**
   ```javascript
   db.nomeDaColecao.insertOne({ chave: "valor", chave2: "valor2" })
   db.nomeDaColecao.insertMany([{ chave: "valor3" }, { chave2: "valor4" }])
   ```

#### 2.2.8. **Buscar documentos:**
   ```bash
   db.nomeDaColecao.find({})
   db.nomeDaColecao.find({ chave: "valor" })
   db.nomeDaColecao.find( { chave: { $in: [ "oi", "tchau" ] } } )
   db.movies.find( { countries: "Mexico", "imdb.rating": { $gte: 7 } } )
   db.movies.find( {
     year: 2010,
     $or: [ { "awards.wins": { $gte: 5 } }, { genres: "Drama" } ]
   } )
   ```

#### 2.2.6. **Atualizar documentos:**
   ```javascript
   db.nomeDaColecao.updateOne({ chave: "valor" }, { $set: { chave2: "novoValor" } })
   db.nomeDaColecao.updateMany({}, { $set: { chave3: "valorComum" } })
   ```

#### 2.2.7. **Excluir documentos:**
   ```javascript
   db.nomeDaColecao.deleteOne({ chave: "valor" })
   db.nomeDaColecao.deleteMany({ chave3: "valorComum" })
   ```

Lembre-se de substituir `nomeDoBancoDeDados`, `nomeDaColecao`, `chave`,
`valor`, etc., pelos nomes e valores apropriados para o seu caso de uso
específico.

### 2.3. Utilizando o pymongo

Para instalar a biblioteca de python para uso **síncrono** do mongoDB,
instale-a com:

```bash
pip install pymongo
```

A seguir, crie um arquivo `.py` e preencha-o com o código abaixo:

```python showLineNumbers tilte="exemplo_mongo.py"
from pymongo import MongoClient

# Conexão com o MongoDB
client = MongoClient('mongodb://localhost:27017/')
db = client['meu_banco']

# Inserção de documentos
colecao = db['minha_colecao']
colecao.insert_one({'nome': 'João', 'idade': 30})
colecao.insert_many([{'nome': 'Maria', 'idade': 25}, {'nome': 'Pedro', 'idade': 35}])

# Busca de documentos
for doc in colecao.find({'idade': {'$gt': 30}}):
    print(doc)

# Atualização de documentos
colecao.update_one({'nome': 'João'}, {'$set': {'idade': 31}})

# Exclusão de documentos
colecao.delete_one({'nome': 'Pedro'})

# Criação de um índice TTL
colecao.create_index([('data_criacao', 1)], expireAfterSeconds=3600)
```

Neste exemplo, utilizamos a biblioteca PyMongo para interagir com o MongoDB.
Primeiro, conectamos ao banco de dados e criamos uma coleção. Em seguida,
realizamos operações de inserção, busca, atualização e exclusão de documentos.
Por fim, criamos um índice TTL para que os documentos sejam automaticamente
excluídos após uma hora.

## 3. Desempenho

### 3.1. Desempenho de busca (índices)

Indexes são estruturas de dados especiais que melhoram a velocidade das
operações de busca. Sem índices, o MongoDB precisa realizar uma varredura
completa da coleção para encontrar os documentos correspondentes. Com índices,
o MongoDB pode limitar a busca a uma parte menor dos dados.

### 3.2. Index TTL (Time to live)

O Index TTL é um recurso que permite definir um tempo de vida para os
documentos. Após o período especificado, os documentos são automaticamente
excluídos do banco de dados. Isso é útil para dados que têm relevância
temporária, como sessões de usuário ou logs.


## 4. Criando schemas MongoDB

<Admonition 
    type="info" 
    title="Autoestudo">

<div style={{ textAlign: 'center' }}>
    <iframe 
        style={{
            display: 'block',
            margin: 'auto',
            width: '100%',
            height: '50vh',
        }}
        src="https://www.youtube.com/embed/QAqK-R9HUhc" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

</Admonition>

## 5. Modelagem de dados com MongoDB

<Admonition 
    type="info" 
    title="Autoestudo">

<div style={{ textAlign: 'center' }}>
    <iframe 
        style={{
            display: 'block',
            margin: 'auto',
            width: '100%',
            height: '50vh',
        }}
        src="https://www.youtube.com/embed/3GHZd0zv170" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

</Admonition>

