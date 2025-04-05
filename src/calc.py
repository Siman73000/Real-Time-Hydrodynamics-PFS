import math


class Calculator:
    def __init__(self):
        self.result = 0
    
    def add(self, a, b):
        self.result = a + b
        return self.result
    
    def subtract(self, a, b):
        self.result = a - b
        return self.result
    
    def multiply(self, a, b):
        self.result = a * b
        return self.result
    
    def divide(self, a, b):
        self.result = a / b
        return self.result

"""
print(Calculator().add(1, 2))
print(Calculator().subtract(1, 2))
print(Calculator().multiply(2, 2))
print(Calculator().divide(6, 3))"
"""