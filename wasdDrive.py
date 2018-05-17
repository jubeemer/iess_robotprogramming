import pygame, serial, math, sys
from pygame.locals import *

pygame.init()
screen = pygame.display.set_mode((200, 200))

class WASDcontroller:
    def __init__(self, port_in):
        self.start1 = 0xDA
        self.start2 = 0xAD
        self.ser = serial.Serial(port_in, 115200)
        self.data = 0x00 # _ _ _ _ D A S W

    def decode_input(self):
        for event in pygame.event.get():
            if not hasattr(event, 'key'): continue
            if event.type == KEYDOWN:
                if event.key == K_w: self.data |= 0x01
                elif event.key == K_s: self.data |= 0x02
                elif event.key == K_a: self.data |= 0x04
                elif event.key == K_d: self.data |= 0x08
            elif event.type == KEYUP:
                if event.key == K_w: self.data &= 0xfe
                elif event.key == K_s: self.data &= 0xfd
                elif event.key == K_a: self.data &= 0xfb
                elif event.key == K_d: self.data &= 0xf7
            self.send_data(self.data)

    def send_data(self, data):
        #self.ser.write(bytearray([self.start1, self.start2, data]))
        self.ser.write(bytearray([self.data]))

keyboard = WASDcontroller("COM8")
while True:
    keyboard.decode_input()
