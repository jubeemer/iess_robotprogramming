import pygame, serial, math, sys
from pygame.locals import *

pygame.init()
screen = pygame.display.set_mode((200, 200))

class station_bt:
    def __init__(self, port_in):
        self.start_byte = 0xFF
        self.ser = serial.Serial(port_in, 115200)
        self.data = 0x00 # sends a number 1 - 6

    def decode_input(self):
        for event in pygame.event.get():
            self.data = 0x00
            if not hasattr(event, 'key'): continue
            if event.type == KEYDOWN:
                key_press = event.key - ord('0')
                if key_press >= 1 and key_press <= 6:
                    self.data += key_press
                    self.send_input()

    def send_input(self):
        self.ser.write(bytearray([self.data]))
        print(self.data)

keyboard = station_bt("COM7")

while True:
    keyboard.decode_input()
