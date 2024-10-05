---
title: SQLite com Python
sidebar_position: 1
sidebar_class_name: opcional
slug: /python-sqlite
---

# Revisão SQLite (Python)

:::note

Não vou abordar como manipular DBs usando ORMs. Vocês já tiveram isso em outros
módulos e eu honestamente tenho nojinho de ORMs. Se quiserem, podem usar, mas
vou fazer meus tutoriais com o bom e velho SQL.

:::

<img 
  src="https://preview.redd.it/2-sql-or-not-2-sql-v0-n7gscgusd4j91.jpg?auto=webp&s=95ef872653409248348da5745fd70407733ba074"
  alt="Camadas do modelo OSI" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '70vh',
    marginRight: 'auto'
  }} 
/>
<br/>


## 1. Instalação do SQLite

SQLite é um banco de dados SQL embutido que não requer um processo de servidor
separado. Ele é frequentemente incluído por padrão com a maioria das
distribuições Linux, incluindo o Ubuntu (e o Arch, btw). Para verificar se o
SQLite já está instalado em seu sistema, execute:

```bash
sqlite3 --version
```

Se o SQLite estiver instalado, você verá a versão do SQLite impressa no
terminal. Caso contrário, você pode instalá-lo usando o seguinte comando:

```bash
sudo apt-get update
sudo apt-get install sqlite3
```

## 2. Instalação do SQLite para Python

Para trabalhar com SQLite em Python, você precisa instalar o módulo `sqlite3`,
que faz parte da biblioteca padrão do Python desde a versão 2.5. Portanto, na
maioria dos casos, **não é necessário instalar nada adicional**. No entanto, se
por algum motivo você precisar instalar ou atualizar o módulo, você pode usar o
pip:

```bash
pip install pysqlite3
```

## 3. Usando SQLite com Python

Aqui está um exemplo de como criar um banco de dados SQLite, criar uma tabela e
inserir alguns dados usando Python:

```python showLineNumbers title="test_sqlite.py"
import sqlite3

# Conectar ao banco de dados SQLite (ou criar se ele não existir)
conn = sqlite3.connect('exemplo.db')

# Criar um objeto cursor
cursor = conn.cursor()

# Criar uma nova tabela
cursor.execute('''CREATE TABLE IF NOT EXISTS usuarios
               (id INTEGER PRIMARY KEY, nome TEXT, idade INTEGER)''')

# Inserir dados na tabela
cursor.execute("INSERT INTO usuarios (nome, idade) VALUES ('Alice', 30)")
cursor.execute("INSERT INTO usuarios (nome, idade) VALUES ('Bob', 25)")

# Salvar (commit) as alterações
conn.commit()

# Consultar os dados
cursor.execute("SELECT * FROM usuarios")

# Buscar e imprimir os resultados
print(cursor.fetchall())

# Fechar a conexão
conn.close()
```

Este exemplo cria um banco de dados chamado `exemplo.db` com uma tabela
`usuarios`. Em seguida, insere dois registros na tabela e os recupera,
imprimindo os resultados.

## 4. Método `executemany`

O método `executemany` é usado para executar um comando SQL várias vezes com
diferentes valores de parâmetros. Isso é útil para inserir, atualizar ou
deletar múltiplos registros de uma vez. O método recebe dois argumentos: a
consulta SQL com placeholders (geralmente `?`) para os parâmetros, e uma lista
ou iterável de tuplas, onde cada tupla contém os valores dos parâmetros para
uma execução da consulta.

```python showLineNumbers title="executemany.py"
import sqlite3

conn = sqlite3.connect('exemplo.db')
cursor = conn.cursor()

# Criar uma nova tabela
cursor.execute('''CREATE TABLE IF NOT EXISTS produtos
               (id INTEGER PRIMARY KEY, nome TEXT, preco REAL)''')

# Lista de produtos para inserir
produtos = [('Teclado', 100.50),
            ('Mouse', 50.25),
            ('Monitor', 450.00),
            ('Gabinete', 200.75)]

# Inserir múltiplos registros usando executemany
cursor.executemany("INSERT INTO produtos (nome, preco) VALUES (?, ?)", produtos)

# Salvar as alterações
conn.commit()

# Consultar os dados inseridos
cursor.execute("SELECT * FROM produtos")
print(cursor.fetchall())

# Fechar a conexão
conn.close()
```

Neste exemplo, a tabela `produtos` é criada e em seguida são inseridos
múltiplos registros utilizando o método `executemany`. Os placeholders `?` na
consulta SQL são substituídos pelos valores de cada tupla na lista `produtos`.

## 5. Método `executescript`

O método `executescript` é usado para executar múltiplos comandos SQL de uma só
vez, separados por ponto e vírgula (`;`). Isso é útil para executar scripts SQL
que contêm várias instruções, como a criação de várias tabelas ou a execução de
várias operações de inserção, atualização ou exclusão.

:::tip

Note que, em nosso exemplo, estamos usando uma string contendo um script SQL.
Utilizar um arquivo `.sql` em vez de uma string hard coded é tão simples quanto
fazer a leitura do arquivo e usar a string resultante como parâmetro para o
método `executescript`.

:::

```python showLineNumbers title="executescript.py"
import sqlite3

conn = sqlite3.connect('exemplo.db')
cursor = conn.cursor()

# Script SQL contendo múltiplas instruções
script_sql = '''
DROP TABLE IF EXISTS clientes;
CREATE TABLE clientes (id INTEGER PRIMARY KEY, nome TEXT, email TEXT);
INSERT INTO clientes (nome, email) VALUES ('Ana Silva', 'ana@example.com');
INSERT INTO clientes (nome, email) VALUES ('Carlos Souza', 'carlos@example.com');
'''

# Executar o script
cursor.executescript(script_sql)

# Salvar as alterações
conn.commit()

# Consultar os dados inseridos
cursor.execute("SELECT * FROM clientes")
print(cursor.fetchall())

# Fechar a conexão
conn.close()
```

Neste exemplo, um script SQL contendo comandos para excluir uma tabela (se
existir), criar uma nova tabela e inserir registros é executado utilizando o
método `executescript`. As instruções no script são separadas por ponto e
vírgula.
