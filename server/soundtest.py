import pygame.mixer
from time import sleep

pygame.mixer.init(48000, -16,1, 1024)
sound = pygame.mixer.Sound("/home/pi/Adeept_PiCar-B/server/WilhelmScream.wav")
ChannelA = pygame.mixer.Channel(1)
ChannelA.play(sound)
sleep(2.0)
