from models.base import Base
from sqlalchemy import Column, Integer, Float

class Coordinates(Base): #Estrutura para criar uma tabela
   __tablename__ = "coordinates"

   id= Column(Integer, primary_key=True, autoincrement=True)
   x = Column(Float)
   y = Column(Float)
   z = Column(Float)
   r = Column(Float)


   def __repr__(self):
      return f"Coordinates {self.x}, {self.y}, {self.z} e {self.r}"

   def return_json(self):
      return {
         "id": self.id,
         "x": self.x,
         "y": self.y,
         "z": self.z,
         "r": self.r
      }
   
   def list_data(self):
      return [self.id, self.x, self.y, self.z, self.r]