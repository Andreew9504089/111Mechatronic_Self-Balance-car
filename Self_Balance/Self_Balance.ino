/*Arduino Self Balancing Robot
 * Code by: B.Aswinth Raj
 * Build on top of Lib: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
 * Website: circuitdigest.com 
 */

#include "I2Cdev.h"
#include <PID_v1.h> //From https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.h
#include "MPU6050_6Axis_MotionApps20.h" //https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050

MPU6050 mpu;

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
float ypr[3];           //yaw/pitch/roll（偏航/俯仰/滚动）数组

const int L298N_IN1 = 7;         
const int L298N_IN2 = 3;
const int L298N_IN3 = 6;
const int L298N_IN4 = 5;
const int L298N_ENA = 11;
const int L298N_ENB = 10;

/*********Tune these 4 values for your BOT*********/
double setpoint= 175 ; //平衡车垂直于地面时的值（目标值）,從序列監控取得小車在直立平衡狀況下的值
//(依照P->D->I順序調參)
double Kp = 100; //1.设置偏差比例系数(調節施予外力的直立,給的過大會震盪)
double Ki = 0.05; //2.调积分(消抖,給的過大會震盪)
double Kd = 1.2; //3.调微分(調節快速平衡)

/******End of values setting*********/

double input, output;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

 
volatile bool mpuInterrupt = false;     // MPU中断
void dmpDataReady()
{
    mpuInterrupt = true;
}

void Forward() //電機前進
{
  digitalWrite(L298N_IN1, LOW);
  digitalWrite(L298N_IN2, HIGH);
  digitalWrite(L298N_IN3, HIGH);
  digitalWrite(L298N_IN4, LOW);
  analogWrite(L298N_ENA, output);
  analogWrite(L298N_ENB, output); 
  Serial.print("F"); //Debugging information 
}

void Reverse() //電機後退
{
  digitalWrite(L298N_IN1, HIGH);
  digitalWrite(L298N_IN2, LOW);
  digitalWrite(L298N_IN3, LOW);
  digitalWrite(L298N_IN4, HIGH);
  analogWrite(L298N_ENA, -output);
  analogWrite(L298N_ENB, -output); 
  //Serial.print("R");
}

void Stop() //電機停止
{
  digitalWrite(L298N_IN1, LOW);
  digitalWrite(L298N_IN2, LOW);
  digitalWrite(L298N_IN3, LOW);
  digitalWrite(L298N_IN4, LOW);
  analogWrite(L298N_ENA, LOW);
  analogWrite(L298N_ENB, LOW);  
  //Serial.print("S");
}

void setup() {
  Serial.begin(115200);

  // initialize device
    //Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

     // verify connection
    //Serial.println(F("Testing device connections..."));
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    devStatus = mpu.dmpInitialize();

    
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(57);
    mpu.setYGyroOffset(-29);
    mpu.setZGyroOffset(3);
    mpu.setZAccelOffset(967); 

      // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
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
        
        //setup PID
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255);  
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

//初始化電機輸出引腳
    pinMode (L298N_IN1, OUTPUT);
    pinMode (L298N_IN2, OUTPUT);
    pinMode (L298N_IN3, OUTPUT);
    pinMode (L298N_IN4, OUTPUT);
    pinMode (L298N_ENA, OUTPUT);
    pinMode (L298N_ENB, OUTPUT);

//默認情況下關閉電機
    digitalWrite(L298N_IN1,LOW);
    digitalWrite(L298N_IN2,LOW);
    digitalWrite(L298N_IN3,LOW);
    digitalWrite(L298N_IN4,LOW);
}

void loop() {
 
    // 如果程序失败直接return。停止 
    if (!dmpReady) return;

    // 等待知道MPU中断返回值可用、数据包正常
    while (!mpuInterrupt && fifoCount < packetSize)
    {
        //无mpu数据,运行PID并输出到电机 
        pid.Compute();   
        
        //Print the value of Input and Output on serial monitor to check how it is working.
        //Serial.print(input); Serial.print(" =>"); Serial.println(output);
               
        if (input>140 && input<220){//If the Bot is falling 
          
        if (output>0) //Falling towards front 
        Forward(); //Rotate the wheels forward 
        else if (output<0) //Falling towards back
        Reverse(); //Rotate the wheels backward 
        }
        else //If Bot not falling
        Stop(); //Hold the wheels still
        
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

        mpu.dmpGetQuaternion(&q, fifoBuffer); //获取q值
        mpu.dmpGetGravity(&gravity, &q); //获取重力值
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //获取ypr值

        input = ypr[1] * 180/M_PI + 180;
   }
}
