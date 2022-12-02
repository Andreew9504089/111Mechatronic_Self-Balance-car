#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h" //https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
#include "SoftwareSerial.h"
#include "SerialTransfer.h"
#include "AltSoftSerial.h"
#define rx 8
#define tx 9

MPU6050 mpu;
//AltSoftSerial pySerial(rx,tx);
SerialTransfer pyTransfer;

struct __attribute__((packed))DATA{
  float UD;
  float LR;
} RxData;

float ctrl_UD = 0;
float ctrl_LR = 0;

// MPU control/status vars
bool dmpReady = false;  // 如果DMP初始化成功，则设置为1
uint8_t mpuIntStatus;   // 保存mpu中断状态
uint8_t devStatus;      // 运行后返回状态0表示成功，!0有错误
uint16_t packetSize;    // DMP数据包的大小
uint16_t fifoCount;     // 统计当前在FIFO的字节数
uint8_t fifoBuffer[64]; // FIFO储存缓冲区

// orientation/motion vars
Quaternion q;           // [w, x, y, z] // (方位,运动)变量
VectorFloat gravity;    // [x, y, z] //重力矢量
int16_t gyro[3];        
float ypr[3];           //yaw/pitch/roll（偏航/俯仰/滚动）数组
float pitch, kalpitch, input;
 
volatile bool mpuInterrupt = false;     // MPU中断
void dmpDataReady()
{
    mpuInterrupt = true;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000);
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400,true);
  #endif
  Serial.begin(19200);
  //pySerial.begin(19200);
  //pySerial.setTimeout(0.1);
  pyTransfer.begin(Serial);

  // initialize device
    //Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

     // verify connection
    //Serial.println(F("Testing device connections..."));
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    devStatus = mpu.dmpInitialize();

    
    // supply your own gyro offsets here, scaled for min sensitivity
//    mpu.setXGyroOffset(57);
//    mpu.setYGyroOffset(-29);
//    mpu.setZGyroOffset(3);
//    mpu.setZAccelOffset(967); 

    if (devStatus == 0)
    {
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));                                                                                                    
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        //Serial.print(F("DMP Initialization failed (code "));
        //Serial.print(devStatus);
        //Serial.println(F(")"));
    }

}

void loop() {
 
   if(pyTransfer.available()){
     pyTransfer.rxObj(RxData);
     //Serial.print("Recevied"); Serial.print(" =>"); Serial.print(ctrl_UD);Serial.print(" =>"); Serial.println(ctrl_LR);
      ctrl_LR = RxData.LR / 100;
      ctrl_UD = RxData.UD / 100;
      // if(ctrl_LR > 0.2){
      //   digitalWrite(LED_BUILTIN, HIGH);
      // }else{
      //   digitalWrite(LED_BUILTIN, LOW);
      // }
    }
  //Serial.println(mpuInterrupt);
  if (!dmpReady){
    //Serial.print("Return");
    return;

  }else if(mpuInterrupt){    
    fifoCount = mpu.getFIFOCount();
    if(fifoCount == packetSize){
      mpuInterrupt = false;
      mpuIntStatus = mpu.getIntStatus(); 

      if ((mpuIntStatus & 0x10) || fifoCount == 1024){
          mpu.resetFIFO();
          //Serial.println(F("FIFO overflow!"));

      }else if (mpuIntStatus & 0x02){
          mpu.getFIFOBytes(fifoBuffer, packetSize);
          fifoCount -= packetSize;
        mpu.dmpGetQuaternion(&q, fifoBuffer);        
        mpu.dmpGetGravity(&gravity, &q);             
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);    

        pitch = ypr[1] * 180/M_PI;
        input = pitch + 180;

        //Serial.println(input);
        if(input > 180){
          digitalWrite(LED_BUILTIN, HIGH);
        }else{
          digitalWrite(LED_BUILTIN, LOW);
        }

        /*
          Do control
        */
      }  
    }
  }
}