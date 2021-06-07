from bluetooth import *

global connection

port = 1024



def initialise_server_socket():
    global connection
    server_socket = BluetoothSocket(RFCOMM)

    server_socket.bind(('', port))

    server_socket.listen(1)
    connection, address = server_socket.accept()