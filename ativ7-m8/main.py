import numpy as np

class Perceptron:
    def __init__(self, learning_rate=0.1, n_iterations=100, threshold=0.5):
        self.learning_rate = learning_rate
        self.n_iterations = n_iterations
        self.threshold = threshold
        self.weights = np.zeros(2)
        self.bias = 0

    def activation_function(self, x):
        return 1 if x >= self.threshold else 0

    def predict(self, inputs):
        linear_output = np.dot(inputs, self.weights) + self.bias
        y_predicted = self.activation_function(linear_output)
        return y_predicted

    def train(self, X, y):
        for _ in range(self.n_iterations):
            for x, y_true in zip(X, y):
                y_pred = self.predict(x)
                error = y_true - y_pred
                self.weights = self.weights + error * self.learning_rate * x 
                self.bias = self.bias + error * self.learning_rate

# Exemplo de uso
if __name__ == "__main__":

    X = np.array([[0, 0], [0, 1], [1, 0], [1, 1]])

    # Resultados para AND
    y = np.array([0, 0, 0, 1]) 

    # Resultados para OR
    # y = np.array([0, 1, 1, 1])

    # Resultado para NAND
    # y = np.array([1, 1, 1, 0])

    # Resultados para XOR
    # y = np.array([0, 1, 1, 0]) 

    perceptron = Perceptron()
    perceptron.train(X, y)

    print(f"Em (0, 0), saída: {perceptron.predict([0, 0])}")
    print(f"Em (0, 1), saída: {perceptron.predict([0, 1])}")
    print(f"Em (1, 0), saída: {perceptron.predict([1, 0])}")
    print(f"Em (1, 1), saída: {perceptron.predict([1, 1])}")