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

    #client.setAtPosition(-360, -1200, -100)
    #sleep(2)
    #client.goTo(-360, -1080, -100)
    height = -65
    client.setAtPosition(-1200, -1000, height)
    sleep(2)
    client.goTo(-1129, -973, height)
    client.goTo(-907, -877, height)
    client.goTo(-789, -809, height)
    client.goTo(-545, -657, height)
    client.goTo(-469, -605, height)
    client.goTo(-359, -563, height)
    client.goTo(-100, -400, height)