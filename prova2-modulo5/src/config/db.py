# Código para conexão e configuração do banco de dados em SQLite, utilizando SQLAlchemy. O banco de dados é criado automaticamente caso não exista, e as tabelas são criadas caso não existam.
from sqlalchemy import create_engine
from sqlalchemy.orm import Session, sessionmaker
from models.base import Base
from models.games import Games

# Cria banco de dados caso ele não exista e estabelece conexão
engine = create_engine('sqlite+pysqlite:///app.db', echo=True)

# Estabelece sessão
Session = sessionmaker(bind=engine)
session = Session()

# Cria tabelas caso elas não existam
Base.metadata.create_all(engine)
