from libs import publisher


class Cycle:
    def __init__(self):
        self.publisher = publisher.Publisher("publisher", "response", "localhost", 1883)
        self.publisher.connect_mqtt()

    def run(self):
        self.publisher.enable_eletromagnet()

    def stop(self):
        self.publisher.disable_eletromagnet()