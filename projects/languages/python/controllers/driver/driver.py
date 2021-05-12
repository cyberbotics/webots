import time
from threading import Thread

from controller import Supervisor


class T(Thread):
    def run(self):
        while not self.ev_stop:
            time.sleep(0.01)
            print(time.time())


s = Supervisor()

while s.step(5000) != -1:
    pass

