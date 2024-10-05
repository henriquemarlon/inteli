from flask import Flask, request, redirect
from controllers import games_controllers

app = Flask(__name__)

@app.route('/game', methods=['GET'])
def user():
    return games_controllers.get_games()

@app.route('/game/create', methods=['POST'])
def create_game():
    name = request.form['name']
    platform = request.form['platform']
    price = request.form['price']
    volume = request.form['volume']
    games_controllers.create_games(name, platform, price, volume)
    return redirect('/game')