import threading

class StoppableThread(threading.Thread):
    def __init__(self, target=None, args=(), kwargs=None):
        super().__init__(target=target, args=args, kwargs=kwargs)

        self.stop_event = threading.Event()

    def run(self):
        while not self.stop_event.is_set():
            self._target()

    def stop(self):
        self.stop_event.set()