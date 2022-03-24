from hashlib import algorithms_available
from turtle import speed
from DroneClient import DroneClient
from time import sleep
import airsim.utils

if __name__ == "__main__":
    client = DroneClient()
    sleep(0.5)
    client.connect()

    print(client.isConnected())
    sleep(0.5)

    client.setAtPosition(-280, -1200, -100)
    sleep(2)
    client.goTo(-400, -1100, -100)

    #client.setAtPosition(-1200, -1000, -50)
    #sleep(2)
    #client.goTo(-1129, -973, -50)
    #client.goTo(-907, -877, -50)
    #client.goTo(-789, -809, -50)
    #client.goTo(-545, -657, -50)
    #client.goTo(-469, -605, -50)
    #client.goTo(-359, -563, -50)
    #client.goTo(-100, -400, -50)