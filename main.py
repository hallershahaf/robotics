from hashlib import algorithms_available
from turtle import speed
from DroneClient import DroneClient
from time import sleep
import airsim.utils

if __name__ == "__main__":
    client = DroneClient()
    client.connect()
    print(client.isConnected())
    sleep(0.5)
    client.setAtPosition(-500, -1200, -65)
    sleep(2)
    client.goTo(-100, -400, -65)
    #client.setAtPosition(-1200, -1000, -65)
    #sleep(2)
    #client.goTo(-100, -400, -65)