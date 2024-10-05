from models.base import Base
from sqlalchemy import Column, Integer, String


class Games(Base):  # Estrutura para criar uma tabela
    __tablename__ = "games"

    id = Column(Integer, primary_key=True, autoincrement=True)
    name = Column(String, nullable=False)
    platform = Column(String, nullable=False)
    price = Column(String, nullable=False)
    volume = Column(Integer, nullable=False)

    def __repr__(self):
        return f"Games(id={self.id}, name={self.name}, platform={self.platform}, price={self.price}, volume={self.volume})"

    def return_json(self):
        return {
            "id": self.id,
            "platform": self.platform,
            "price": self.price,
            "volume": self.volume
        }