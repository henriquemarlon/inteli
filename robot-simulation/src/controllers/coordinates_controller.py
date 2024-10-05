from models.coordinates import Coordinates
from config.db import session

# get all users
def get_coordinates():
    coordinate = session.query(Coordinates).all()
    return [user.return_json() for user in coordinate]

def create_coordinates(x, y, z, r):
    coordinate = Coordinates(x=x, y=y, z=z, r=r)
    session.add(coordinate)
    session.commit()

def get_last_coordinate():
    coordinate = session.query(Coordinates.id.desc()).first()
    return coordinate



