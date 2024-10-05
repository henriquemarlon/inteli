from models.games import Games
from config.db import session

# getting all games
def get_games():
    games = session.query(Games).all()
    return [games.return_json() for games in games]

# creating games
def create_games(name, platform, price, volume):
    games = Games(name=name, platform=platform, price=price, volume=volume)
    session.add(games)
    session.commit()