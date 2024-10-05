import sqlite3

conn = sqlite3.connect('exemplo2.db')
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
