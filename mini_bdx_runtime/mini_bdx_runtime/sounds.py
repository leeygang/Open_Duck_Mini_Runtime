import pygame
import time
import os
import random

class Sounds:
    def __init__(self, volume=1.0, sound_directory="./"):
        pygame.mixer.init()
        pygame.mixer.music.set_volume(volume)
        self.sounds = {}

        for file in os.listdir(sound_directory):
            if file.endswith(".wav"):
                sound_path = os.path.join(sound_directory, file)
                try:
                    self.sounds[file] = pygame.mixer.Sound(sound_path)
                    print(f"Loaded: {file}")
                except pygame.error as e:
                    print(f"Failed to load {file}: {e}")

    def play(self, sound_name):
        if sound_name in self.sounds:
            self.sounds[sound_name].play()
            print(f"Playing: {sound_name}")
        else:
            print(f"Sound '{sound_name}' not found!")

    def play_random_sound(self):
        sound_name = random.choice(list(self.sounds.keys()))
        self.sounds[sound_name].play()


# Example usage
if __name__ == "__main__":
    sound_player = Sounds(1.0, "../../scripts")
    time.sleep(1)
    while True:
        sound_player.play_random_sound()
        time.sleep(3)
