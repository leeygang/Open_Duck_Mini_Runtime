import time
import os
import pygame
from glob import glob

class Sounds:
    def __init__(self, volume, sounds_path):
        self.volume = volume
        self.sounds_path = sounds_path
        self.sounds = {}

        files = glob(sounds_path + "/*.wav")


        freq = 44100    # audio CD quality
        bitsize = -16   # unsigned 16 bit
        channels = 1    # 1 is mono, 2 is stereo
        buffer = 2048   # number of samples (experiment to get right sound)
        pygame.mixer.init(freq, bitsize, channels, buffer)

        pygame.mixer.music.set_volume(self.volume)

    def play(self, sound_name):
