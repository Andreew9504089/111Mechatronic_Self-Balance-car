import pygame
import numpy as np
import serial
from pySerialTransfer import pySerialTransfer as txfer
from time import sleep, time

pygame.init()
pygame.joystick.init()

LOCKED = (200,100,100)
UNLOCKED = (100,200,100)
WIDTH,HEIGHT = 500,500
Done = False
Lock = True
print("Wait for Arduino BT to initialize...")
arduino = txfer.SerialTransfer('COM7') 
arduino.close()
arduino.open()
sleep(2)

class Data(object):
    UD = 0
    LR = 0

class Control():
    def __init__(self,joystick, screen, socket, txData):
        self.joystick = joystick
        self.screen = screen
        self.lock = True
        self.txData = txData
        self.LRoffset = 0
        self.UDoffset = 0
        self.socket = socket
    
    def update(self):
        if self.lock:
            print("Press Y to unlock")
        else:
            self.txData.UD = float(np.round(((self.joystick.get_axis(5)+1) - (self.joystick.get_axis(4)+1))/2,2)*100)
            self.txData.LR = float(np.round(self.joystick.get_axis(0) + self.LRoffset,2)*100)
            self.draw_joystick((self.txData.LR/100,self.txData.UD/100))

    def Lock(self,lock):
        self.lock = lock

    def sendCommand(self):
        sendSize = 0
        sendSize = self.socket.tx_obj(self.txData.UD, start_pos = sendSize)
        sendSize = self.socket.tx_obj(self.txData.LR, start_pos = sendSize)
        self.socket.send(sendSize)

    def draw_joystick(self,joy=(0,0)):
        #draw virtual joystick
        font = pygame.font.Font('freesansbold.ttf', 32)
        text = font.render('UNLOCKED', True, (0,175,0), (255,255,255))
        textRect = text.get_rect()
        textRect.center = (100, 50)
        screen.fill(UNLOCKED)
        screen.blit(text,textRect)

        font = pygame.font.Font('freesansbold.ttf', 16)
        text = font.render('Turn:'+str(np.round(joy[0],1)), True, (0,175,0), (255,255,255))
        textRect = text.get_rect()
        textRect.center = (400, 60)
        screen.blit(text, textRect)

        font = pygame.font.Font('freesansbold.ttf', 16)
        text = font.render('Acc:'+str(np.round(joy[1],1)), True, (0,175,0), (255,255,255))
        textRect = text.get_rect()
        textRect.center = (400, 40)
        screen.blit(text, textRect)

        radius = WIDTH // 3
        center = (WIDTH // 2, HEIGHT // 2)
        angle = np.arctan2(joy[1],joy[0])
        end = (center[0] + abs(joy[0])*radius*np.cos(angle), center[1] - abs(joy[1])*radius*np.sin(angle))
        pygame.draw.line(screen, (100, 100, 100), center, end, 5)
        pygame.draw.circle(screen, (0, 0, 0), center, radius, 2)
        pygame.draw.circle(screen, (100, 100, 100), end, radius/20, 0)
        pygame.display.flip()

if __name__=='__main__':
    time_last = time()
    screen = pygame.display.set_mode((WIDTH,HEIGHT))
    clock = pygame.time.Clock()
    joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
    font = pygame.font.Font('freesansbold.ttf', 32)

    text = font.render('LOCKED', True, (255,0,0), (255,255,255))
    textRect = text.get_rect()
    textRect.center = (80, 50)
    screen.fill(LOCKED)
    screen.blit(text,textRect)
    pygame.display.flip()

    txData = Data
    control = Control(joysticks[0],screen, arduino, txData)

    while not Done:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                Done = True
            
            elif  event.type == pygame.JOYBUTTONDOWN: #A:0 B:1 X:2 Y:3
                if  Lock == False and event.button == 0: #press A button
                    #print("Car locked, ,maintaining balance mode")
                    Lock = True

                    text = font.render('LOCKED', True, (255,0,0), (255,255,255))
                    textRect = text.get_rect()
                    textRect.center = (80, 50)
                    screen.fill(LOCKED)
                    screen.blit(text,textRect)
                    pygame.display.flip()

                    control.Lock(Lock)

                if  Lock == True and event.button == 3: #press Y button
                    #print("Car unlocked, ,enter remote control mode")
                    text = font.render('UNLOCKED', True, (0,175,0), (255,255,255))
                    textRect = text.get_rect()
                    textRect.center = (100, 50)
                    screen.fill(UNLOCKED)
                    screen.blit(text,textRect)
                    Lock = False

                    control.draw_joystick()
                    control.Lock(Lock)

                if event.button == 6: # Press split screen to quit
                    Done = True
            
            elif event.type == pygame.JOYAXISMOTION:
                control.update()

        if((time() - time_last) > 0.01):
            control.sendCommand()
            time_last = time()

    arduino.close()
    pygame.quit()      
        

 