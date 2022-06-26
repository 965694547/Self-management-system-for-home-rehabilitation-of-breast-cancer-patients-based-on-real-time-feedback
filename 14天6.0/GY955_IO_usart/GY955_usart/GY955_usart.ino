#include <U8glib.h>




#include <Mp3Module.h>

//#include <Mp3Module.h>
//#include <MsTimer2.h> 

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE|U8G_I2C_OPT_DEV_0);
/*
GY955----MINI
VCC----VCC
GND----GND
1:GY955_TX---10,send AA 38 E2 to GY-955
2:GY955_RX---9
注意下载程序时候拔出tx rx
*/
//////////////////
//#include <SoftwareSerial.h>

#define INTERVAL_CONST 0.005

//SoftwareSerial mySerial(10, 9); // RX, TX

unsigned char Re_buf[30],counter=0,counter1=0;//state1=0;
unsigned char sign=0;
int16_t DATA[7];
float ROLL,PITCH,YAW;
float Original_YAW,Original_ROLL,Original_PITCH;
float Last_YAW,Last_ROLL,Last_PITCH;

unsigned char counter3=0;//counter2=0,
float geomean[5],geomean_dsnl;

float geomean_ROLL[5];
int ROLL_gll;

float geomean_PITCH[5];
int PITCH_gll;





const int Key1= 7;  
const int Key2= 6;
const int Key3= 3;
const int Key4= 2;
// 连接按键的引脚
int Key1_State = 1;
int Key2_State = 1;
int Key3_State = 1;
int Key4_State = 1;// 存储按键状态的变量

int select=1;//模式选择位 
int select_1=1;//大模式选择位 

  
unsigned char qz,state1;
unsigned char xw,state2;
unsigned char wq=0;
unsigned char tb=0,state3;
unsigned char wz=0,state4;
unsigned char hbs=0,state5;
unsigned char key_state=0;//是否第一次进入按键
unsigned char key_state_1=0;//是否第一次进入按键


typedef struct 
{
  float Acc_Out_X;      
  float Acc_Out_Y;    
  float Acc_Out_Z;  
  float Gyro_Out_X;   
  float Gyro_Out_Y; 
  float Gyro_Out_Z;
  
  float Gyro_X_Error;    
  float Gyro_Y_Error;
  float Gyro_Z_Error;

  float Car_Angle_Pitch;   

  float Acc_Angle_Pitch;   

  float Gyro_Speed_X;   
  float Gyro_Speed_Y;       
  float Gyro_Speed_Z;       
  

}ATTITUDE ;
 
ATTITUDE  Attitude={0,0,0,0,0,0,.Gyro_X_Error=0,.Gyro_Y_Error=0,.Gyro_Z_Error=0,0,0,0,0};    

void Key3_interrupt()
{   
    
   
   delay(10);
   Key3_State = digitalRead(Key3);

  // 检查按键是否被按下
  if ( Key3_State == LOW) 
  {   
    key_state=1;
    if(select_1==1)
    {
          if(select==3)
          {
            select=6;
          }
          else if(select==6)
          {
            select=1;
          }
          else
          {
            select=select+1; 
          }
    }
    if(select_1==2)
    {
          if(select==5)
          {
            select=7;
          }
          else if(select==7)
          {
            select=1;
          }
          else
          {
            select=select+1; 
          }
    }
    while(digitalRead(Key3)==LOW)
    delay(1);
    Key3_State = digitalRead(Key3);
  }
   
 
}

void Key4_interrupt()
{
   delay(10);
   Key4_State = digitalRead(Key4);

  // 检查按键是否被按下
  if (Key4_State == LOW) 
  {     
    key_state_1=1;
    key_state=1;
    select=1;
   qz=0;
   xw=0;
   tb=0;
   wq=0;
   wz=0;
   if(select_1==2)
   {
      select_1=1;
   }
    else
    {
      select_1=select_1+1; 
    }
    while(digitalRead(Key4)==LOW)
    delay(5);
    Key4_State = digitalRead(Key4);
  } 
}

void setup()
{



//pinMode(Key1, INPUT_PULLUP);    
//pinMode(Key2, INPUT_PULLUP);    
  pinMode(Key3, INPUT_PULLUP);    
  pinMode(Key4, INPUT_PULLUP);    
   
   Serial.begin(9600);  
   delay(1);
   Serial.write(0XAA); 
   Serial.write(0X2D);    //初始化,连续输出模式
   Serial.write(0XD7);    //初始化,连续输出模式
   delay(10); 
 attachInterrupt(digitalPinToInterrupt(Key3),Key3_interrupt, FALLING);
 attachInterrupt(digitalPinToInterrupt(Key4),Key4_interrupt, FALLING);
// MsTimer2::set(5,serialEvent); // 500ms period
// MsTimer2::start();
 if ( u8g.getMode() == U8G_MODE_R3G3B2 ) {
    u8g.setColorIndex(255);     // white
  }
  else if ( u8g.getMode() == U8G_MODE_GRAY2BIT ) {
    u8g.setColorIndex(3);         // max intensity
  }
  else if ( u8g.getMode() == U8G_MODE_BW ) {
    u8g.setColorIndex(1);         // pixel on
  }
  else if ( u8g.getMode() == U8G_MODE_HICOLOR ) {
    u8g.setHiColorByRGB(255,255,255);
  }
}



static void Angle_Calculate(ATTITUDE *Attitude)
{                             
   Attitude->Acc_Angle_Pitch = -(float)atan2f(Attitude->Acc_Out_X ,Attitude->Acc_Out_Z) * 180 / PI;
   Attitude->Gyro_Speed_Y = Attitude->Gyro_Out_Y - Attitude->Gyro_Y_Error;   
   Attitude->Gyro_Speed_Z = Attitude->Gyro_Out_Z - Attitude->Gyro_Z_Error;
   Attitude->Gyro_Speed_X = Attitude->Gyro_Out_X - Attitude->Gyro_Z_Error;      
}

void dsnl(float input[],float*dsnl,int n)//基于短时能量与过零率的端点检测
{
  float temp1;
  for(int i=0;i<n;i++ )
  {
    temp1=input[i];
    *dsnl=*dsnl+abs(temp1);
  }
  *dsnl/=n;
//  Serial.print("过零率：");
// Serial.print(gll);
//  Serial.print(',');
//  Serial.print("短时能量：");
//Serial.print(dsnl);
//  Serial.print(',');
}

void gll(float input[],int*gll,int n)
{
  float temp1,temp2;
  for(int i=0;i<n-1;i++ )
  {
    temp1=input[i];
    temp2=input[i+1];
 
    if( temp2-temp1<-1)(*gll)--;
    if( temp2-temp1>1)(*gll)++;
  }
}

static void Kalman_Filter(ATTITUDE *Attitude)
{
    static float Q_Angle=0.001,Q_Gyro=0.0001,R_Acc=500;
    static float Q_bias=0;
    static float P[2][2]={{1.0,0.0},{0.0,1.0}};
    
    float K0,K1;
    float denominator;
    float Error;
    
    Attitude->Car_Angle_Pitch += (Attitude->Gyro_Speed_X - Q_bias) * INTERVAL_CONST;
    
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

void final_list_1()
{
  int energy=(1*wq+3*xw+3*qz)/60*4;
  u8g.firstPage();  
   do {
         // graphic commands to redraw the complete screen should be placed here  
     u8g.setFont(u8g_font_6x10);
   u8g.setPrintPos(25,8); u8g.print("Exercise list");
  // call procedure from base class, http://arduino.cc/en/Serial/Print
   u8g.setPrintPos(0,18);u8g.print("Clench fist:");
   u8g.setPrintPos(72,18);u8g.print(wq);
      u8g.setPrintPos(0,28);u8g.print("Bend elbows:");
   u8g.setPrintPos(72,28);u8g.print(qz);
     u8g.setPrintPos(0,38);u8g.print("Wrist rotation:");
   u8g.setPrintPos(90,38);u8g.print(xw);
   u8g.setPrintPos(0,48);u8g.print("Calorie:");
   u8g.setPrintPos(48,48);u8g.print(energy);delay(10);
       } while( u8g.nextPage() );
}

void final_list_2()
{
  int energy=(1*wq+3*xw+3*qz+tb*5+wz*6)/60*4;
  u8g.firstPage();  
   do {
         // graphic commands to redraw the complete screen should be placed here  
     u8g.setFont(u8g_font_6x10);
   u8g.setPrintPos(25,8); u8g.print("Exercise list");
  // call procedure from base class, http://arduino.cc/en/Serial/Print
     u8g.setPrintPos(0,16);u8g.print("Clench fist:");
   u8g.setPrintPos(72,16);u8g.print(wq);
     u8g.setPrintPos(0,24);u8g.print("Wrist rotation:");
   u8g.setPrintPos(90,24);u8g.print(xw);
   u8g.setPrintPos(0,32);u8g.print("Bend elbows:");
   u8g.setPrintPos(72,32);u8g.print(qz);
   u8g.setPrintPos(0,40);u8g.print("Lifting arm:");
   u8g.setPrintPos(72,40);u8g.print(tb);
   u8g.setPrintPos(0,48);u8g.print("Outreach:");
   u8g.setPrintPos(55,48);u8g.print(wz);
      u8g.setPrintPos(0,56);u8g.print("Calorie:");
   u8g.setPrintPos(48,56);u8g.print(energy);delay(10);
       } while( u8g.nextPage() );
}

void Get_Angle()
{
  Angle_Calculate(&Attitude);
  //Kalman_Filter(&Attitude);
}

void serialEvent()
{
 while (Serial.available())
    {   
      Re_buf[counter]=(unsigned char)Serial.read();
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
         Attitude.Acc_Out_Y =  (float)DATA[1]/100;
         Attitude.Acc_Out_Z =  (float)DATA[2]/100;
         
         Attitude.Gyro_Out_X = (float)DATA[3]/16;
         Attitude.Gyro_Out_Y = (float)DATA[4]/16;
         Attitude.Gyro_Out_Z = (float)DATA[5]/16;
         YAW=(float)DATA[6]/100;
         ROLL=(float)DATA[7]/100;
         PITCH=(float)DATA[8]/100;

        
         if(counter1==1)
         {
             Mp3Module Mymp3(10,12,13);
            Mymp3.VolumeSet(28);
            Mymp3.Play(1,Wait);
            //delay(50);
            Mymp3.VolumeSet(28);
            Mymp3.Play(2,Wait);
            //delay(50);
         }


         if(counter1<=10&&counter1!=1)
         {
             u8g.firstPage();  
          do {
            u8g.setFont(u8g_font_unifont);
            //u8g.setFont(u8g_font_osb21);
            u8g.drawStr( 0, 22, "Please waiting!");
              } while( u8g.nextPage() );
            // rebuild the picture after some delay 
             //Serial.println("WAITING");
           delay(5);
           Original_YAW+=YAW;
           Original_ROLL+=ROLL;
           Original_PITCH+=PITCH;
         }

         if(counter1==10)
         {
            Mp3Module Mymp3(10,12,13);
            Original_YAW/=9;
            Original_ROLL/=9;
            Original_PITCH/=9; 
            //Serial.println(Original_ROLL);
            delay(5);
            Mymp3.VolumeSet(28);
            Mymp3.Play(3,Wait);
            delay(10000);
            key_state=1;
            key_state_1=1;
         }
         
        
         counter1++;
         
        if(counter1>10)
        {
        counter1=11;
        Get_Angle();

//           
//           if( (((float)((uint16_t)DATA[0])/100)<100)&&(((float)DATA[1]/100)<100)&&(((float)DATA[2]/100)<100))
//           {
//           geomean_dsnl=0;
//           geomean[counter3]=sqrt((Attitude.Gyro_Speed_X*Attitude.Gyro_Speed_X+Attitude.Gyro_Speed_Y*Attitude.Gyro_Speed_Y+Attitude.Gyro_Speed_Z*Attitude.Gyro_Speed_Z)/3);//sqrt((Attitude.Acc_Out_X*Attitude.Acc_Out_X+Attitude.Acc_Out_Y*Attitude.Acc_Out_Y+Attitude.Acc_Out_Z*Attitude.Acc_Out_Z)/3);//Attitude.Gyro_Speed_X;//sqrt((Attitude.Gyro_Speed_X*Attitude.Gyro_Speed_X+Attitude.Gyro_Speed_Y*Attitude.Gyro_Speed_Y+Attitude.Gyro_Speed_Z*Attitude.Gyro_Speed_Z)/3);
//           counter3++;
//           dsnl(geomean,&geomean_dsnl,5);
//           }
//           if(counter3==5)counter3=0;
//           
//         float temp;
//         for(int i=0;i<4;i++)
//         {
//          geomean_ROLL[i]=geomean_ROLL[i+1];
//          geomean_PITCH[i]=geomean_PITCH[i+1];
//         }
//         
//         ROLL_gll=0;
//         geomean_ROLL[4]=ROLL;
//         gll(geomean_ROLL,&ROLL_gll,5);
//         
//         PITCH_gll=0;
//         geomean_PITCH[4]=PITCH;
//         gll(geomean_PITCH,&PITCH_gll,5);

      
       


        switch(select)
       {
            case 2:xw_right_function();break;
            case 3:qz_function();break;
            case 4:tb_function();break;
            case 5:wz_function();break;
            case 6:final_list_1();break;
            case 7:final_list_2();break;
       }
        }
                   
        delay(10);
        sign=0;
        
   }
 } 
}


//void Original_PRINT()
//{ 
//  unsigned char i=0,sum=0;
//  if(sign)
//  {   
//  
//     for(i=0;i<27;i++)
//      sum+=Re_buf[i]; 
//     if(sum==Re_buf[i] )        //检查帧头，帧尾
//     {    
//         DATA[6]=(Re_buf[16]<<8)|Re_buf[17];
//         DATA[7]=(Re_buf[18]<<8)|Re_buf[19];
//         DATA[8]=(Re_buf[20]<<8)|Re_buf[21];
//     
//         Original_YAW=(float)DATA[6]/100+Original_YAW;
//         Original_ROLL+=((float)DATA[7]/100);
//         Original_PITCH+=((float)DATA[8]/100);
//         sign=0;
//         Serial.println(Original_ROLL);
//         delay(100);
//     }
//  }  
//}

void qz_function()//现在的问题是停滞状态下数据不稳定，且放下手的时候要停滞一段时间
{
  Mp3Module Mymp3(10,12,13);
  if(abs(Original_ROLL-ROLL)<=50&&abs(Attitude.Gyro_Speed_Z)<=40)
   state1=1;
   if(abs(Original_ROLL-ROLL)>=126-40&&(state1==1))
   {
    state1=0;
    qz++; 
    Mymp3.VolumeSet(28);Mymp3.Play(7,NoWait);
   }

    if(qz==32&&select_1==1){select=6;key_state=1;}
    if(qz==32&&select_1==2){select++;key_state=1;}

   
   u8g.firstPage();  
   do {
         // graphic commands to redraw the complete screen should be placed here  
   u8g.setFont(u8g_font_6x10);
   u8g.setPrintPos(0,8); u8g.print("Pattern:bend elbows");
  // call procedure from base class, http://arduino.cc/en/Serial/Print
   u8g.setPrintPos(0,18);u8g.print("Times:");
   u8g.setPrintPos(35,18);u8g.print(qz);
   u8g.setPrintPos(0,28);u8g.print("State:");
   u8g.setPrintPos(35,28);u8g.print(state1);
   u8g.setPrintPos(0,38);u8g.print("RPY:");
   u8g.setPrintPos(23,38);u8g.print(ROLL);
   u8g.setPrintPos(83,38);u8g.print(PITCH);
   //u8g.setPrintPos(98,38);u8g.print(YAW);
   u8g.setPrintPos(0,48);u8g.print("Speed:");
   u8g.setPrintPos(35,48);u8g.print(Attitude.Gyro_Speed_Z);
       } while( u8g.nextPage() );
  delay(10);     
       // rebuild the picture after some delay 
//   Serial.print("屈肘次数：");
//   Serial.print(qz);
//   Serial.print(","); 
//   Serial.print(Attitude.Gyro_Speed_Z);
//   Serial.print(","); 
//   Serial.print("RPY: ");
//   Serial.print(ROLL);
//   Serial.print(",");  
//   Serial.print(PITCH);
//   Serial.print(",");
//   Serial.print(YAW);
//   Serial.println(";"); 
}

void xw_right_function()
{
  Mp3Module Mymp3(10,12,13);
  if(abs(Original_ROLL-ROLL)<=25&&abs(Attitude.Gyro_Speed_Z)<40)
  {
    state2=1;
    Last_YAW=YAW;
    Last_ROLL=ROLL;
    Last_PITCH=PITCH;    
  }
  
  if(ROLL>=22-50&&ROLL<=41+50&&PITCH>-61-50&&PITCH<-49+50&&((state2==1)||(state2==4)))//
  {
    if(state2==4){xw++;Mymp3.VolumeSet(28);Mymp3.Play(7,NoWait);}
    state2=2;
    Last_YAW=YAW;
    Last_ROLL=ROLL;
    Last_PITCH=PITCH;    
  }
//  if((Last_ROLL-ROLL)<-5.0&&(Last_PITCH-PITCH)>5.0&&(state2==2))
//  {
//    state2=3;
//    Last_YAW=YAW;
//    Last_ROLL=ROLL;
//    Last_PITCH=PITCH;    
//  }
  if((Last_PITCH-PITCH)<-100.0&&(state2==2))
  {
    state2=4;
    Last_YAW=YAW;
    Last_ROLL=ROLL;
    Last_PITCH=PITCH;    
  }
//  if((Last_ROLL-ROLL)>5.0&&(Last_PITCH-PITCH)>5.0&&(state2==4))
//  {
//    state2=5;
//    Last_YAW=YAW;
//    Last_ROLL=ROLL;
//    Last_PITCH=PITCH;    
//  }

  if(xw==32){select++;key_state=1;}
  
     u8g.firstPage();  
   do {
         // graphic commands to redraw the complete screen should be placed here  
   u8g.setFont(u8g_font_6x10);
   u8g.setPrintPos(0,8); u8g.print("Pattern:wrist rotation");
  // call procedure from base class, http://arduino.cc/en/Serial/Print
   u8g.setPrintPos(0,18);u8g.print("Times:");
   u8g.setPrintPos(35,18);u8g.print(xw);
   u8g.setPrintPos(0,28);u8g.print("State:");
   u8g.setPrintPos(35,28);u8g.print(state2);
   u8g.setPrintPos(0,38);u8g.print("RPY:");
   u8g.setPrintPos(23,38);u8g.print(ROLL);
   u8g.setPrintPos(83,38);u8g.print(PITCH);
   //u8g.setPrintPos(98,38);u8g.print(YAW);
   u8g.setPrintPos(0,48);u8g.print("Speed:");
   u8g.setPrintPos(35,48);u8g.print(Attitude.Gyro_Speed_Z);
       } while( u8g.nextPage() );
       delay(10);
//  Serial.print("旋腕次数：");
//  Serial.print(xw);
//  Serial.print(","); 
//  Serial.print(state2); 
//  Serial.print(","); 
//  Serial.print( "Speed_z:");  
//  Serial.print(Attitude.Gyro_Speed_Z);
//  Serial.print(","); 
//  Serial.print("RPY: ");
//  Serial.print(ROLL);
//  Serial.print(",");  
//  Serial.print(PITCH);
//  Serial.print(",");
//  Serial.print(YAW);
//  Serial.println(";"); 
}

void tb_function()
{
  Mp3Module Mymp3(10,12,13);
  if(abs(Original_ROLL-ROLL)<=50&&abs(Attitude.Gyro_Speed_Z)<=40)
   state3=1;
  if(abs(Original_ROLL-ROLL)>=60&&(state3==1)&&abs(Attitude.Gyro_Speed_Z)>=10)
   {
    state3=2;
   }
  if(abs(Original_ROLL-ROLL)>=(134-40)&&(state3==2)&&abs(Attitude.Gyro_Speed_Z)<=60)
   {
    state3=0;
    tb++;
    Mymp3.VolumeSet(28);
    Mymp3.Play(7,NoWait);
   }

    if(tb==32&&select_1==2){select++;key_state=1;}
    
     u8g.firstPage();  
   do {
         // graphic commands to redraw the complete screen should be placed here  
   u8g.setFont(u8g_font_6x10);
   u8g.setPrintPos(0,8); u8g.print("Pattern:lifting arm");
  // call procedure from base class, http://arduino.cc/en/Serial/Print
   u8g.setPrintPos(0,18);u8g.print("Times:");
   u8g.setPrintPos(35,18);u8g.print(tb);
   u8g.setPrintPos(0,28);u8g.print("State:");
   u8g.setPrintPos(35,28);u8g.print(state3);
   u8g.setPrintPos(0,38);u8g.print("RPY:");
   u8g.setPrintPos(23,38);u8g.print(ROLL);
   u8g.setPrintPos(83,38);u8g.print(PITCH);
   //u8g.setPrintPos(98,38);u8g.print(YAW);
   u8g.setPrintPos(0,48);u8g.print("Speed:");
   u8g.setPrintPos(35,48);u8g.print(Attitude.Gyro_Speed_Z);
       } while( u8g.nextPage() );
       delay(10);
}

void wz_function()
{
  Mp3Module Mymp3(10,12,13);
  if(abs(ROLL-0)<=20||abs(YAW+318.55)<=10)
   state4=1;
  if(abs(ROLL-0)>=30&&(state4==1)&&abs(Attitude.Gyro_Speed_Z)>=5)
{
state4=2;
}
  if(abs(ROLL+61.30)<=20&&(state4==2)&&abs(Attitude.Gyro_Speed_Z)<=60)
   {
    state4=0;
    wz++;
    Mymp3.VolumeSet(28);
    Mymp3.Play(7,NoWait);
   }

    if(wz==32&&select_1==2){select=7;key_state=1;}
     u8g.firstPage();  
   do {
         // graphic commands to redraw the complete screen should be placed here  
   u8g.setFont(u8g_font_6x10);
   u8g.setPrintPos(0,8); u8g.print("Pattern:outreach");
  // call procedure from base class, http://arduino.cc/en/Serial/Print
   u8g.setPrintPos(0,18);u8g.print("Times:");
   u8g.setPrintPos(35,18);u8g.print(wz);
   u8g.setPrintPos(0,28);u8g.print("State:");
   u8g.setPrintPos(35,28);u8g.print(state4);
   u8g.setPrintPos(0,38);u8g.print("RPY:");
   u8g.setPrintPos(23,38);u8g.print(ROLL);
   u8g.setPrintPos(83,38);u8g.print(YAW);
   //u8g.setPrintPos(98,38);u8g.print(PITCH);
   u8g.setPrintPos(0,48);u8g.print("Speed:");
   u8g.setPrintPos(35,48);u8g.print(Attitude.Gyro_Speed_Z);
       } while( u8g.nextPage() );
       delay(10);
}


void wq_function()
{
  Mp3Module Mymp3(10,12,13);
  int sensorValue1 = analogRead(A0);
  int sensorValue2 = analogRead(A3);
  if(sensorValue1>=50)
  {
    wq=wq+1;  
    Mymp3.VolumeSet(28);
    Mymp3.Play(7,NoWait);
    while(1)
    {
     sensorValue1 = analogRead(A0);
    if(sensorValue1<100)
    break;  
    }
  }

  if(wq==32){select++;key_state=1;}
   
     u8g.firstPage();  
   do {
         // graphic commands to redraw the complete screen should be placed here  
   u8g.setFont(u8g_font_6x10);
   u8g.setPrintPos(0,8); u8g.print("Pattern:clench fist");
  // call procedure from base class, http://arduino.cc/en/Serial/Print
   u8g.setPrintPos(0,18);u8g.print("Times:");
   u8g.setPrintPos(35,18);u8g.print(wq);
   u8g.setPrintPos(0,28);u8g.print("SensorValue1:");
   u8g.setPrintPos(80,28);u8g.print(sensorValue1);
   u8g.setPrintPos(0,38);u8g.print("SensorValue2:");
   u8g.setPrintPos(80,38);u8g.print(sensorValue1);
   
       } while( u8g.nextPage() );
       delay(10);
//  Serial.print(wq);
//  Serial.print(',');
//  Serial.print(sensorValue1);
//  Serial.print(',');
//  Serial.print(sensorValue2);
//  Serial.println(';');
//  delay(500);  
}




void loop()
{
   Mp3Module Mymp3(10,12,13);
//   SCI_receive();
//   if(counter1<10)
//   {
//    Serial.println("WAITING");
//    Original_PRINT();
//    delay(2);
//   }
//   else if(counter1==10&&(state1==0))
//   {
//     Original_YAW/=10;
//     Original_ROLL/=10;
//     Original_PITCH/=10;
//     Serial.println(Original_ROLL);
//     state1=1;
//   }
//   else 
//   {
//     PRINT();
//     qz_function();
//   }
if(select==1&&counter1>10)
  {
  wq_function();
  }
else 
  {
  serialEvent(); 
  PRINT();
  }
  
  if(key_state_1==1)
 {
        key_state_1=0;
        switch(select_1)
        {
        case 1:Mymp3.VolumeSet(28);Mymp3.Play(11,Wait);break;
          case 2:Mymp3.VolumeSet(28);Mymp3.Play(12,Wait);break;
        }
 }
 
  if(key_state==1){
    key_state=0;
  switch(select){
      case 1 :Mymp3.VolumeSet(28);Mymp3.Play(4,Wait);break;
       case 2 :Mymp3.VolumeSet(28);Mymp3.Play(5,Wait);break;
        case 3 : Mymp3.VolumeSet(28);Mymp3.Play(6,Wait);break;
         case 4 :Mymp3.VolumeSet(28);Mymp3.Play(8,Wait);break;
          case 5 : Mymp3.VolumeSet(28);Mymp3.Play(9,Wait);break;
//           case 6 : Mymp3.VolumeSet(28);Mymp3.Play(10,Wait);break;
//            case 7 : Mymp3.VolumeSet(28);Mymp3.Play(10,Wait);break;
    }
   if(select==6||select==7)
   {
    Mymp3.VolumeSet(28);Mymp3.Play(10,Wait);

          if(select==6)
          {
              if(wq==32&&xw==32&&qz==32)
              {
                Mymp3.VolumeSet(28);
                Mymp3.Play(13,Wait);
              }
              else
              {
                Mymp3.VolumeSet(28);
                Mymp3.Play(14,Wait);
              }
          }
        if(select==7)
              {
                  if(wq==32&&xw==32&&qz==32&&tb==32&&wz==32)
                  {
                    Mymp3.VolumeSet(28);
                    Mymp3.Play(13,Wait);
                  }
                  else
                  {
                    Mymp3.VolumeSet(28);
                    Mymp3.Play(14,Wait);
                  }
              }
   }
}
   
//  u8g.firstPage();  
//  do {
//    u8g.setFont(u8g_font_unifont);
//  //u8g.setFont(u8g_font_osb21);
//    u8g.drawStr( 0, 22, "Please waiting!");
//  } while( u8g.nextPage() );
//  // rebuild the picture after some delay 
}
