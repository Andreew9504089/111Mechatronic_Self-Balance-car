#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h" //https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
#include "KalmanFilter.h"

MPU6050 mpu;
KalmanFilter kalman(0.001, 0.003, 0.03);

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
  Serial.begin(115200);

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
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

}

void loop() {
 
    // 如果程序失败直接return。停止 
    if (!dmpReady) return;

    // 等待知道MPU中断返回值可用、数据包正常
    while (!mpuInterrupt && fifoCount < packetSize)
    {

        Serial.print(pitch); Serial.print(" =>"); Serial.print(gyro[1]);Serial.print(" =>"); Serial.print(kalpitch); Serial.print(" =>"); Serial.println(input);

    }

    // 重置中断标志,并获取INT_STATUS数据
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // 获取当前FIFO计数
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
       // 重置
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));

    // 否则，检查DMP数据准备中断
    }
    else if (mpuIntStatus & 0x02)
    {
        // 等待正确的可用的数据
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // 读取先进先出的包
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetGyro(  gyro, fifoBuffer);            // Obtain Gyro Data
        mpu.dmpGetQuaternion(&q, fifoBuffer);         //获取q值
        mpu.dmpGetGravity(&gravity, &q);              //获取重力值
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);    //获取ypr值

        pitch = ypr[1] * 180/M_PI;
        kalpitch = kalman.update(pitch, gyro[1]);
        input = kalpitch + 180;
   }
}
