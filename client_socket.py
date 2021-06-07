from bluetooth import *

port = 1024


def initialise_client_socket():
    nearby_devices = discover_devices(lookup_names=True)
    print("Found {} devices.".format(len(nearby_devices)))
    for addr, name in nearby_devices:
        print("  {} - {}".format(addr, name))

    service = find_service(address='B8:27:EB:8C:0A:1F')
    client_socket = BluetoothSocket(RFCOMM)

    client_socket.connect((service["host"], port))

    while True:
        try:
            coordinates = client_socket.recv(1024).decode()
            if coordinates:
                print("TODO something")
                pass
        except KeyboardInterrupt:
            client_socket.close()