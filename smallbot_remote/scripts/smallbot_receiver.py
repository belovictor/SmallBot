import socket


class SmallBotReceiver:
    def __init__(self, ip="0.0.0.0", port=12345):
        self._ip = ip
        self._port = port
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind((self._ip, self._port))

    def receive(self):
        data, address = self._socket.recvfrom(1024)
        return data.decode("utf-8").rstrip('\n')

    def close(self):
        self._socket.close()
