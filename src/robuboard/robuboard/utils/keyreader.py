import os, sys, termios, tty, select, fcntl, time

class KeyReader:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old_term = termios.tcgetattr(self.fd)
        self.old_flags = fcntl.fcntl(self.fd, fcntl.F_GETFL)

    def __enter__(self):
        tty.setraw(self.fd)

        new = termios.tcgetattr(self.fd)
        new[1] |= termios.OPOST | termios.ONLCR   # oflag: \n -> \r\n
        termios.tcsetattr(self.fd, termios.TCSADRAIN, new)

        fcntl.fcntl(self.fd, fcntl.F_SETFL, self.old_flags | os.O_NONBLOCK)
        return self

    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_term)
        fcntl.fcntl(self.fd, fcntl.F_SETFL, self.old_flags)

    def read_available(self) -> str:
        buf = []
        while True:
            r, _, _ = select.select([sys.stdin], [], [], 0)
            if not r:
                break
            try:
                data = os.read(self.fd, 1024)
                if not data:
                    break
                buf.append(data.decode("utf-8", errors="ignore"))
            except BlockingIOError:
                break
        return "".join(buf)