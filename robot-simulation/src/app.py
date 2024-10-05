from flask import Flask, request, redirect
from controllers import coordinates_controller
from flask import render_template

app = Flask(__name__)

# Route for coordinates
@app.route('/coordinates', methods=['GET'])
def user():
    return render_template('index.html', coordinates=coordinates_controller.get_coordinates())

# Route for to get last coordinates
@app.route('/coordinates/last', methods=['GET'])
def last_coordinate():
    return coordinates_controller.get_coordinates()[-1]

# Route for create coordinates
@app.route('/coordinates/create', methods=['POST'])
def create_user():
    x = request.form['x']
    y = request.form['y']
    z = request.form['z']
    r = request.form['r']
    coordinates_controller.create_coordinates(x, y, z, r)
    return redirect('/coordinates')