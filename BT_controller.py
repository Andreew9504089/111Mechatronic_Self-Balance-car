import pygame
import numpy as np
import bluetooth 

pygame.init()
pygame.joystick.init()

LOCKED = (200,100,100)
UNLOCKED = (100,200,100)
WIDTH,HEIGHT = 500,500
Done = False
Lock = True
        
class Control():
    def __init__(self,joystick, screen, socket):
        self.joystick = joystick
        self.screen = screen
        self.lock = True
        self.LR = 0
        self.LRoffset = 0
        self.UD = 0
        self.UDoffset = 0
        self.socket = socket
    
    def update(self):
        if self.lock:
            print("Press Y to unlock")
        else:
            self.UD = (self.joystick.get_axis(5)+1)/2 - (self.joystick.get_axis(4)+1)/2
            self.LR = self.joystick.get_axis(0) + self.LRoffset
            self.draw_joystick((self.LR,self.UD))

    def Lock(self,lock):
        self.lock = lock

    def sendCommand(self):
        data = [self.UD, self.LR]
        self.socket.send(data)

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

def findBTDevice():
    global socket
    print("Searching for Bluetooth Device...")
    nearby_devices = bluetooth.discover_devices()
    num = 0
    for i in nearby_devices:
        num += 1
        print(num,": ", bluetooth.lookup_name(i))

    selection = input("> ") - 1
    print("You have selected ", bluetooth.lookup_devices[selection])
    bd_addr = nearby_devices[selection]
    port = 1

    socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    socket.connect((bd_addr, port))


if __name__=='__main__':
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

    findBTDevice()
    control = Control(joysticks[0],screen,socket)

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

    pygame.quit()      
        

 