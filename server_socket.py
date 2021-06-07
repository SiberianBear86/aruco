from bluetooth import *

global connection


def initialise_server_socket():
    global connection
    server_socket = BluetoothSocket(RFCOMM)

    server_socket.bind(('', 1024))

    server_socket.listen(1)
    connection, address = server_socket.accept()