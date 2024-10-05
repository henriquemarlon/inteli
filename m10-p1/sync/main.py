from flask import Flask, request, jsonify
import sqlite3
import jwt
from flask_swagger_ui import get_swaggerui_blueprint
from functools import wraps

SWAGGER_URL = "/docs"
API_URL = "/static/swagger.json"

SWAGGER_BLUEPRINT = get_swaggerui_blueprint(
    SWAGGER_URL,
    API_URL,
    config={
        'app_name': 'To-Do API'
    }
)

app = Flask(__name__)

app.register_blueprint(SWAGGER_BLUEPRINT, url_prefix=SWAGGER_URL)

app.config['SECRET_KEY'] = 'secret'

def token_required(f):
    @wraps(f)
    def decorated(*args, **kwargs):
        token = request.headers.get('Authorization')
        if not token:
            return jsonify({'message': 'Token is missing!'}), 401
        try:
            data = jwt.decode(token.split()[1], app.config['SECRET_KEY'], algorithms=['HS256'])
        except:
            return jsonify({'message': 'Token is invalid!'}), 401

        return f(*args, **kwargs)

    return decorated

@app.route('/login', methods=['POST'])
def login():
    data = request.get_json()
    username = data.get('username')
    password = data.get('password')

    if not username or not password:
        return jsonify({'message': 'Username and Password are required!'}), 401

    con = sqlite3.connect("sqlite.db")
    cur = con.cursor()

    cur.execute("SELECT * FROM users WHERE username = ? AND password = ?", (username, password))
    user = cur.fetchone()

    if user:
        token = jwt.encode({'username': username}, app.config['SECRET_KEY'], algorithm='HS256')
        return jsonify({'token': token})

    return jsonify({'message': 'Invalid Username or Password!'}), 401

@app.route('/register', methods=['POST'])
def register():
    data = request.get_json()
    username = data.get('username')
    password = data.get('password')
    if not username or not password:
        return jsonify({'message': 'Username and Password are required!'}), 400
    con = sqlite3.connect("sqlite.db")
    cur = con.cursor()
    cur.execute("INSERT INTO users (username, password) VALUES (?, ?)", (username, password))
    con.commit()

    return jsonify({"status": "success"})

@app.route('/addTask', methods=['POST'])
@token_required
def addTask():
    con = sqlite3.connect("sqlite.db")
    cur = con.cursor()
    data = request.get_json()
    title = data['title']
    content = data['content']
    cur.execute("INSERT INTO tasks (title, content) VALUES (?, ?)", (title, content))
    con.commit()
    return jsonify({"status": "success"})

@app.route('/getTasks', methods=['GET'])
@token_required
def getTasks():
    con = sqlite3.connect("sqlite.db")
    cur = con.cursor()
    cur.execute("SELECT * FROM tasks")
    tasks = cur.fetchall()
    return jsonify({"tasks": tasks})

@app.route('/deleteTask', methods=['DELETE'])
@token_required
def deleteTask():
    con = sqlite3.connect("sqlite.db")
    cur = con.cursor()
    title = request.args.get('title')
    cur.execute("DELETE FROM tasks WHERE title = ?", (title,))
    con.commit()
    return jsonify({"status": "success"})

@app.route('/updateTask', methods=['PUT'])
@token_required
def updateTask():
    con = sqlite3.connect("sqlite.db")
    cur = con.cursor()
    data = request.get_json()
    title = data['title']
    content = data['content']
    cur.execute("UPDATE tasks SET content = ? WHERE title = ?", (content, title))
    con.commit()
    return jsonify({"status": "success"})

if __name__ == '__main__':
    app.run(debug=True, port=3000, host="0.0.0.0")