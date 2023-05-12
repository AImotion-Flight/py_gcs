import threading

class Terminal:
    def __init__(self):
        self.lock = threading.Lock()

    def log(self, message):
        with self.lock:
            print(f'- {message}')

    def input(self):
        with self.lock:
            return input('> ')

terminal = Terminal()