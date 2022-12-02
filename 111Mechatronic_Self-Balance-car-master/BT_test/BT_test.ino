#include "SoftwareSerial.h"
#include "SerialTransfer.h"
#include "AltSoftSerial.h"
#define rx 8
#define tx 9

AltSoftSerial pySerial(rx,tx);
SerialTransfer pyTransfer;

struct __attribute__((packed))DATA{
  double UD;
  double LR;
} RxData;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(19200);
  pySerial.begin(19200);
  //pySerial.setTimeout(0.1);
  pyTransfer.begin(pySerial);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(pyTransfer.available()){
    pyTransfer.rxObj(RxData);
    Serial.print("Recevied"); Serial.print(" =>"); Serial.print(RxData.UD);Serial.print(" =>"); Serial.println(RxData.LR);
  }else{
  }
}
