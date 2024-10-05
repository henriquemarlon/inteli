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
