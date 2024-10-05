import sqlite3

conn = sqlite3.connect('exemplo3.db')
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
