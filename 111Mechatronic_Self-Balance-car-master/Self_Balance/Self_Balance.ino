/*Arduino Self Balancing Robot
 * Code by: B.Aswinth Raj
 * Build on top of Lib: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
 * Website: circuitdigest.com 
 */

/* 
Hyperparameters:

  PID, output limit for motor to track setpoint:
    Kp, Ki, Kd, upper, lower, manual_point
  PID for dynamical setpoint to avoid running forward or backward by set_dir:
    sKp, sKi, sKd, sUpper, sLower, set_dir
*/
#include "I2Cdev.h"
#include "Wire.h"
#include <PID_v1.h> //From https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.h
#include "MPU6050_6Axis_MotionApps20.h" //https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
#include "SerialTransfer.h"

MPU6050 mpu;
SerialTransfer pyTransfer;

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
float pitch, kalpitch;


const int L298N_IN1 = 7;         
const int L298N_IN2 = 3;
const int L298N_IN3 = 13;
const int L298N_IN4 = 12;
const int L298N_ENA = 6;
const int L298N_ENB = 5;

/*********Tune these 4 values for your BOT*********/
double manualpoint= 180; //平衡车垂直于地面时的值（目标值）,從序列監控取得小車在直立平衡狀況下的值
double setpoint = manualpoint;
//(依照P->D->I順序調參)
double Kp = 2; //1.设置偏差比例系数(調節施予外力的直立,給的過大會震盪)
double Ki = 0.05; //2.调积分(消抖,給的過大會震盪)
double Kd = 1.2; //3.调微分(調節快速平衡)
double upper = 255;
double lower = -255;
double sample_time = 5;

double set_dir = 0;
double sKp = 12;
double sKd = 0;
double sKi = 1;
double sUpper = 3;
double sLower = -3;
double s_sample_time = 50;

double LR_strength = 0.5; // LR from -1 to 1
double UD_strength = 0.5; // UD from -1 to 1
/******End of values setting*********/

double speedFactor = 1;
double control;
double input, output;
double s_in, s_out;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);    // motor pid
PID spid(&s_in, &s_out, &set_dir, sKp, sKi, sKd, DIRECT);   // setpoint pid
 
volatile bool mpuInterrupt = false;     // MPU中断

struct __attribute__((packed))DATA{
  float UD;
  float LR;
} RxData;

float ctrl_UD = 0;
float ctrl_LR = 0;

void dmpDataReady()
{
  mpuInterrupt = true;
}

void Forward(int turn) //電機前進
{
  if(turn == 0){
    digitalWrite(L298N_IN1, LOW);
    digitalWrite(L298N_IN2, HIGH);
    digitalWrite(L298N_IN3, HIGH);
    digitalWrite(L298N_IN4, LOW);
    analogWrite(L298N_ENA, control);
    analogWrite(L298N_ENB, control); 
    //Serial.print("F");                                //Debugging information 
    if(s_in > -1){
      s_in -= 0.01;
    }
  }else if(turn == 1){                                // turn right
    digitalWrite(L298N_IN1, LOW);
    digitalWrite(L298N_IN2, HIGH);
    digitalWrite(L298N_IN3, HIGH);
    digitalWrite(L298N_IN4, LOW);
    analogWrite(L298N_ENA, control*speedFactor);
    analogWrite(L298N_ENB, control); 
    //Serial.print("FR");                               //Debugging information 
  }else if(turn == 2){
    digitalWrite(L298N_IN1, LOW);
    digitalWrite(L298N_IN2, HIGH);
    digitalWrite(L298N_IN3, HIGH);
    digitalWrite(L298N_IN4, LOW);
    analogWrite(L298N_ENA, control);
    analogWrite(L298N_ENB, control*speedFactor); 
    //Serial.print("FL"); 
  }

}

void Reverse(int turn) //電機後退
{
  if(turn == 0){
    digitalWrite(L298N_IN1, HIGH);
    digitalWrite(L298N_IN2, LOW);
    digitalWrite(L298N_IN3, LOW);
    digitalWrite(L298N_IN4, HIGH);
    analogWrite(L298N_ENA, -control);
    analogWrite(L298N_ENB, -control); 
    //Serial.print("R");
    if(s_in < 1){
      s_in += 0.01;
    }
  }else if(turn == 1){
    digitalWrite(L298N_IN1, HIGH);
    digitalWrite(L298N_IN2, LOW);
    digitalWrite(L298N_IN3, LOW);
    digitalWrite(L298N_IN4, HIGH);
    analogWrite(L298N_ENA, -control*speedFactor);
    analogWrite(L298N_ENB, -control); 
    //Serial.print("RL");
  }else if(turn == 2){
    digitalWrite(L298N_IN1, HIGH);
    digitalWrite(L298N_IN2, LOW);
    digitalWrite(L298N_IN3, LOW);
    digitalWrite(L298N_IN4, HIGH*speedFactor);
    analogWrite(L298N_ENA, -control);
    analogWrite(L298N_ENB, -control); 
    //Serial.print("RR");
  }

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
  s_in = 0;
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
  Serial.flush();
  //pySerial.begin(9600);
  //pySerial.setTimeout(0.1);
  pyTransfer.begin(Serial);


  //Serial.println("Initializing I2C devices...");
  mpu.initialize();
  //Serial.println("Initializing DMP...");
  devStatus = mpu.dmpInitialize();

  
  mpu.setXGyroOffset(106);
  mpu.setYGyroOffset(-30);
  mpu.setZGyroOffset(-58);
  mpu.setZAccelOffset(919); 

  if (devStatus == 0)
  {
      mpu.setDMPEnabled(true);

      attachInterrupt(0, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
      dmpReady = true;
      packetSize = mpu.dmpGetFIFOPacketSize();
      
      pid.SetMode(AUTOMATIC);
      pid.SetSampleTime(sample_time);
      pid.SetOutputLimits(lower, upper);  
      spid.SetMode(AUTOMATIC);
      spid.SetSampleTime(s_sample_time);
      spid.SetOutputLimits(sLower, sUpper);  
  }
  else
  {
      //Serial.print(F("DMP Initialization failed (code "));
      //Serial.print(devStatus);
      //Serial.println(F(")"));
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
  //Serial.println(mpuInterrupt);
  if(pyTransfer.available()){
    pyTransfer.rxObj(RxData);
    ctrl_LR = RxData.LR / 100;
    ctrl_UD = RxData.UD / 100;
  }

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
          digitalWrite(LED_BUILTIN, HIGH);
      }else if (mpuIntStatus & 0x02){
        while (fifoCount >= packetSize){
          mpu.getFIFOBytes(fifoBuffer, packetSize);
          fifoCount -= packetSize;
        }
        mpu.dmpGetQuaternion(&q, fifoBuffer);        
        mpu.dmpGetGravity(&gravity, &q);             
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);    

        pitch = ypr[1] * 180/M_PI;
        input = pitch + 180;

        //Serial.println(setpoint); //Serial.print("=>"); Serial.print(s_out); Serial.print("=>"); Serial.println(setpoint);

        
        pid.Compute();                              //Print the value of Input and Output on serial monitor to check how it is working.
        spid.Compute();                             //setpoint PID

        // no data from python -> perform standby balancing
        setpoint = s_out + manualpoint + ctrl_UD*UD_strength;
        control = output;
        if (input>120 && input<240 ){      //             
          if (control>0){
            if(ctrl_LR > 0.1 && input > 160 && input < 200){
              speedFactor = 1 - ctrl_LR*LR_strength;
              Forward(1);
            }else if(ctrl_LR < -0.1 && input > 160 && input < 200){
              speedFactor = 1 + ctrl_LR*LR_strength;
              Forward(2);
            }else if(ctrl_UD != 0 && input > 160 && input < 200){
              Forward(0);
              s_in = 0;
              s_out = 0;
            }else{
              Forward(0);
            }
          }else if (control<0){
            if(ctrl_LR > 0.1  && input > 160 && input < 200){
              speedFactor = 1 - ctrl_LR*LR_strength;
              Reverse(1);
            }else if(ctrl_LR < -0.1 && input > 160 && input < 200){
              speedFactor = 1 + ctrl_LR*LR_strength;
              Reverse(2);
            }else if(ctrl_UD != 0 && input > 160 && input < 200){
              Reverse(0);
              s_in = 0;
              s_out = 0;
            }else{
              Reverse(0);
            }                              
          }                       
        }else{
          Stop();  
        }                    
      }  
    }
  }
}
