from DroneClient import DroneClient
from time import sleep
import airsim.utils

if __name__ == "__main__":
    client = DroneClient()
    sleep(0.5)
    client.connect()

    print(client.isConnected())

    client.setAtPosition(0, -900, -100)

    sleep(5)
    client.goTo(-1000, -500, -100)
    
