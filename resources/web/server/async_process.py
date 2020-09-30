import queue
import subprocess
import threading


class AsyncProcess:
    """Start a command line process and returns its stdout, stderr and termination asynchronously."""
    def __init__(self, command):
        self.process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.queue = queue.Queue()
        self.out = threading.Thread(target=self.__enqueue_stream, args=[1])
        self.err = threading.Thread(target=self.__enqueue_stream, args=[2])
        self.proc = threading.Thread(target=self.__enqueue_process)
        self.out.start()
        self.err.start()
        self.proc.start()

    def run(self):
        """Return data as soon as it is available on stdout or stderr or when the process exits."""
        """Data from stdout is prefixed with '1', data on stderr is prefixed with '2'."""
        """When the process exists, this function returns 'x'."""
        line = self.queue.get()
        if line == 'x':
            self.out.join()
            self.err.join()
            self.proc.join()
        return line

    def __enqueue_stream(self, type):
        stream = self.process.stdout if type == 1 else self.process.stderr
        for line in iter(stream.readline, b''):
            self.queue.put(str(type) + line.decode('utf-8'))
        stream.close()

    def __enqueue_process(self):
        self.process.wait()
        self.queue.put('x')
