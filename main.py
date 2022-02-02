from DroneClient import DroneClient
from time import sleep
import airsim.utils

if __name__ == "__main__":
    client = DroneClient()
    sleep(0.5)
    client.connect()

    print(client.isConnected())

    client.setAtPosition(-500, -800, -100)

    sleep(5)
    client.goTo(-300, -800, -100)
    
