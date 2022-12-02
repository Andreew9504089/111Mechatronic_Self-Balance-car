import serial
from pySerialTransfer import pySerialTransfer as txfer
from time import sleep

arduino = txfer.SerialTransfer('COM7')
arduino.close()
arduino.open()
print("Wait 2sec for Arduino BT to initialize...")
sleep(2)

class Data(object):
    UD = 1.
    LR = 2.


def sendCommand(data, arduino):
    sendSize = 0
    sendSize = arduino.tx_obj(data.UD, start_pos = sendSize)
    sendSize = arduino.tx_obj(data.LR, start_pos = sendSize)
    arduino.send(sendSize)


if __name__=='__main__':
    num = 0
    while(num < 10):
        txData = Data
        sendCommand(txData, arduino)
        print("sent")
        num += 1
        sleep(1)
        

    arduino.close()
        

 