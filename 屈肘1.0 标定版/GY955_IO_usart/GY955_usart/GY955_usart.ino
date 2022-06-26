/////////////////////
/*
GY955----MINI
VCC----VCC
GND----GND
1:GY955_TX---10,send AA 38 E2 to GY-955
2:GY955_RX---9
*/
//////////////////
#include <SoftwareSerial.h>
#define ratio_gyro 0.061035 //2000/2^15  陀螺仪采样单位化比例因子
#define ratio_acc  0.000244  
#define INTERVAL_CONST 0.005

SoftwareSerial mySerial(10, 9); // RX, TX

unsigned char Re_buf[30],counter=0;
unsigned char sign=0;
int16_t DATA[7];
float ROLL,PITCH,YAW;

unsigned char qz,state;

typedef struct 
{
  int Acc_Out_X;      
  int Acc_Out_Y;    
  int Acc_Out_Z;  
  int Gyro_Out_X;   
  int Gyro_Out_Y; 
  int Gyro_Out_Z;
  
  int Gyro_X_Error;    
  float Gyro_Y_Error;
  int Gyro_Z_Error;

  float Car_Angle_Pitch;   

  float Acc_Angle_Pitch;   

  short int Gyro_Speed_Y;       
  short int Gyro_Speed_Z;       
  

}ATTITUDE ;
 
ATTITUDE  Attitude={0,0,0,0,0,0,.Gyro_X_Error=0,.Gyro_Y_Error=0,.Gyro_Z_Error=0,0,0,0};    

static void Angle_Calculate(ATTITUDE *Attitude)
{                             
   Attitude->Acc_Angle_Pitch = -(float)atan2f(Attitude->Acc_Out_X ,Attitude->Acc_Out_Z) * 180 / PI;
   Attitude->Gyro_Speed_Y = Attitude->Gyro_Out_Y - Attitude->Gyro_Y_Error;   
   Attitude->Gyro_Speed_Z = Attitude->Gyro_Out_Z - Attitude->Gyro_Z_Error;   
}

static void Kalman_Filter(ATTITUDE *Attitude)
{
    static float Q_Angle=0.001,Q_Gyro=0.0001,R_Acc=500;
    static float Q_bias=0;
    static float P[2][2]={{1.0,0.0},{0.0,1.0}};
    
    float K0,K1;
    float denominator;
    float Error;
    
    Attitude->Car_Angle_Pitch += (Attitude->Gyro_Speed_Y - Q_bias) * INTERVAL_CONST;
    
    P[0][0] += -P[0][1] * INTERVAL_CONST - P[1][0] * INTERVAL_CONST + Q_Angle;    
    P[0][1] += -P[1][1] * INTERVAL_CONST;
    P[1][0] += -P[1][1] * INTERVAL_CONST;
    P[1][1] += Q_Gyro;

    denominator = P[0][0] + R_Acc;
    K0 = P[0][0] / denominator;
    K1 = P[1][0] / denominator;
    
    Error = Attitude->Acc_Angle_Pitch - Attitude->Car_Angle_Pitch;
    Attitude->Car_Angle_Pitch += K0 * Error;
    Q_bias += K1 * Error;
    
    P[0][0] *= 1 - K0;
    P[0][1] *= 1 - K0;
    P[1][0] -= P[0][0] * K1;
    P[1][1] -= P[0][1] * K1;
   
   Attitude->Gyro_Speed_Y -= Q_bias;
}

void Get_Angle()
{
  Angle_Calculate(&Attitude);
  //Kalman_Filter(&Attitude);
}

void SCI_receive()
{
 while (mySerial.available())
    {   
      Re_buf[counter]=(unsigned char)mySerial.read();
      if(counter==0&&Re_buf[0]!=0x5A) return;      // 检查帧头         
      counter++;       
      if(counter==28)                //接收到数据
      {    
         counter=0;                 //重新赋值，准备下一帧数据的接收 
         sign=1;
      }      
     } 
}

void PRINT()
{
  unsigned char i=0,sum=0;
  if(sign)
  {   
  
     for(i=0;i<27;i++)
      sum+=Re_buf[i]; 
     if(sum==Re_buf[i] )        //检查帧头，帧尾
     {    
         DATA[0]=(Re_buf[4]<<8)|Re_buf[5];
         DATA[1]=(Re_buf[6]<<8)|Re_buf[7];
         DATA[2]=(Re_buf[8]<<8)|Re_buf[9];
         DATA[3]=(Re_buf[10]<<8)|Re_buf[11];
         DATA[4]=(Re_buf[12]<<8)|Re_buf[13];
         DATA[5]=(Re_buf[14]<<8)|Re_buf[15];
         DATA[6]=(Re_buf[16]<<8)|Re_buf[17];
         DATA[7]=(Re_buf[18]<<8)|Re_buf[19];
         DATA[8]=(Re_buf[20]<<8)|Re_buf[21];
         Attitude.Acc_Out_X = (float)((uint16_t)DATA[0])/100;
         Attitude.Acc_Out_Z =  (float)DATA[2]/100;
         Attitude.Gyro_Out_Y = (float)DATA[4]/16;
         Attitude.Gyro_Out_Z = (float)DATA[5]/16;
         YAW=(float)DATA[6]/100;
         ROLL=(float)DATA[7]/100;
         PITCH=(float)DATA[8]/100;
         
         Get_Angle();
        
        delay(5);
        Serial.print("屈肘次数：");
        Serial.print(qz);
        Serial.print(","); 
        Serial.print( "Speed_z:");  
        Serial.print(Attitude.Gyro_Speed_Z);
        Serial.print(","); 
        Serial.print("RPY: ");
        Serial.print(ROLL);
        Serial.print(",");  
        Serial.print(PITCH);
        Serial.print(",");
        Serial.print(YAW);
        Serial.println(",");            
        delay(20);
        sign=0;
   }
 } 
}

void qz_function()
{
  if(ROLL<=80&&ROLL>=50&&Attitude.Gyro_Speed_Z<5)
   state=1;
   if(ROLL<=-50&&Attitude.Gyro_Speed_Z<5&&(state==1))
   {
    state=0;
    qz++;
   }
}
void setup()
{
   Serial.begin(115200);  
   mySerial.begin(9600);
   mySerial.listen();
   delay(10);
   mySerial.write(0XAA); 
   mySerial.write(0X2D);    //初始化,连续输出模式
   mySerial.write(0XD7);    //初始化,连续输出模式
   delay(1);     
}

void loop()
{
   SCI_receive();
   PRINT();
   qz_function();
}
