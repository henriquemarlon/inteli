from models.base import Base
from threading import Thread
from datetime import datetime
from models.report import Report
from sqlalchemy import create_engine
from controllers.pdf_controller import PDF
from sqlalchemy.orm import Session, sessionmaker
from controllers.dobot_controller import Routine
from flask import Flask, render_template, redirect, request

PDF_WIDTH = 210
PDF_HEIGHT = 297
process = None

app = Flask(__name__)
header = []
engine = create_engine("sqlite+pysqlite:///reports.db", echo=True)
Session = sessionmaker(bind=engine)
session = Session()

Base.metadata.create_all(engine)


@app.route('/')
def index():
    return render_template('about.html')


@app.route('/home')
def home():
    return render_template('index.html')


@app.route('/report')
def report():
    return render_template('report.html')


@app.route('/routine')
async def control_on():
    global process
    try:
        process = Thread(target=Routine.start_routine(), args=())
        process.start()
        process.join()
        return redirect('/')
    except Exception as e:
        print("error")
        return e


@app.route('/create_pdf', methods=["POST"])
async def post_form():
    print(request.form)
    header.clear()
    for i in request.form:
        header.append((i.capitalize(), request.form[i]))

    r1 = Report(project=request.form['projeto'], client=request.form['cliente'], sample=request.form['amostra'], operator=request.form['operador'],
                cycles=request.form['ciclos'], liquid_initial_mass=request.form['peso solido'], solid_initial_mass=request.form['peso solido'])

    session.add(r1)
    session.commit()
    return render_template('index.html', project=request.form['projeto'], client=request.form['cliente'], sample=request.form['amostra'])


@app.route('/export_pdf')
async def generate_pdf():
    pdf = PDF(orientation='P', unit='mm', format='A4')
    pdf.add_page()
    pdf.header()
    a = [10, 10]
    side = True
    for i in header:
        pdf.element(i[0] + ': ' + i[1], a, side)
        a[1] += 5 if (not side) else 0
        side = not side
    pdf.generate(datetime.now().strftime("%d-%m-%Y-%H%M%S"))
    return render_template("index.html")

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=3000, debug=True)
