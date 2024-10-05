# Código para conexão e configuração do banco de dados em SQLite, utilizando SQLAlchemy. O banco de dados é criado automaticamente caso não exista, e as tabelas são criadas caso não existam.
from sqlalchemy import create_engine
from sqlalchemy.orm import Session, sessionmaker
from models.base import Base
from models.coordinates import Coordinates


# Cria banco de dados caso ele não exista e estabelece conexão
engine = create_engine('sqlite+pysqlite:///app.db', echo=True)

# Estabelece sessão
Session = sessionmaker(bind=engine)
session = Session()

# Cria tabelas caso elas não existam
Base.metadata.create_all(engine) 

# creating a user
coordinate = Coordinates(x=2, y=3, z=4, r=5)
session.add(coordinate)
session.commit()



# # editing a user
# user = session.query(Users).filter_by(name='João').first()
# user.email = "joaobobao@gmail.com"
# session.commit()

# # deleting a user
# user = session.query(Users).filter_by(name='João').first()
# session.delete(user)
# session.commit()

# # getting user
# user = session.query(Users).filter_by(name='João').first()
# print(user)
