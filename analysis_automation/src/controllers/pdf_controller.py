from fpdf import FPDF

PDF_WIDTH = 210
PDF_HEIGHT = 297


class PDF(FPDF):
    def header(self):
        self.rect(x=4, y=4, w=200, h=31, style='')
    def element(self, content, position, iter):
        self.set_xy(position[0], position[1])
        self.set_font('Arial', 'B', 11)
        self.set_text_color(0, 0, 0)
        self.cell(w=100.0, h=10, align='L' if (iter) else 'R', txt=content, border=0)
    
    def generate(self, date):
        self.output('Relatorio-' + date + '.pdf', 'F')