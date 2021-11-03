import socket


class Connection:
    def __init__(self, port: int = 5555):
        self.HOST = ''
        self.port = port
        self.m_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.connect()

    def connect(self):
        self.m_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.m_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.m_socket.bind((self.HOST, self.port))

    def reset_port(self, port):
        self.port = port