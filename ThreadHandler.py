from threading import Thread, Event

class StoppableThread(Thread):
    def __init__(self,  target=None, name=None, args=()):
        super(StoppableThread, self).__init__(target=target, args=(self, args[0], args[1]))
        self._stop_event = Event()

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()
