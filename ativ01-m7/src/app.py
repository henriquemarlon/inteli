from flask import *

app = Flask(__name__)

@app.route('/')
def hello():
    return render_template("index.html");

@app.route('/assets/<filename>')
def assets(filename):
    return send_from_directory('static', filename);