from pymongo import MongoClient

# Conectar ao servidor MongoDB (por padrão, na porta 27017 do localhost)
client = MongoClient('mongodb://localhost:27017/')

# Selecionar o banco de dados 'meu_banco_de_dados'
db = client['meu_banco_de_dados']

# Selecionar a coleção 'minha_colecao'
colecao = db['minha_colecao']

# Inserir um documento na coleção
documento = {'nome': 'Fulano', 'idade': 30}
resultado_insercao = colecao.insert_one(documento)
print(f'Documento inserido com o ID: {resultado_insercao.inserted_id}')

# Buscar o documento inserido
documento_encontrado = colecao.find_one({'nome': 'Fulano'})
print(f'Documento encontrado: {documento_encontrado}')
