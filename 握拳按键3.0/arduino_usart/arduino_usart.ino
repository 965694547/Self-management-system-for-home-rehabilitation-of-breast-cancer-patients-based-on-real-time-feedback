/////////////////////
/*
GY955----MINI
VCC----VCC
GND----GND
1:RX---TX,send AA 38 E2 to GY-955
2:TX---RX
3:MINI_TX---FT232_RX
*/
//////////////////
unsigned char Re_buf[30],counter=0;
unsigned char sign=0;

  float ROLL,PITCH,YAW;
  float Q4[4];
    



void setup() {
 
   Serial.begin(115200);  
  delay(1);    
  Serial.write(0XAA); 
  Serial.write(0X38);    //初始化,连续输出模式
  Serial.write(0XE2);    //初始化,连续输出模式
}

void loop() {
  unsigned char i=0,sum=0;
   int16_t DATA[7];
  if(sign)
  {   
  
     for(i=0;i<19;i++)
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
         YAW= (float)((uint16_t)DATA[0])/100;
         ROLL=(float)DATA[1]/100;
         PITCH=  (float)DATA[2]/100;
         Q4[0]= (float)DATA[3]/10000;
         Q4[1]= (float)DATA[4]/10000;
         Q4[2]= (float)DATA[5]/10000;
         Q4[3]= (float)DATA[6]/10000;
       Serial.print("RPY: ");
       Serial.print( ROLL);
       Serial.print(",");
       Serial.print( PITCH);
       Serial.print(",");
       Serial.println( YAW);
       Serial.print("Q4: ");
       Serial.print( Q4[0]);
       Serial.print(",");
       Serial.print( Q4[1]);
       Serial.print(",");
       Serial.print( Q4[2]);
       Serial.print(",");
       Serial.print( Q4[3]);
       Serial.print(";");
       sign=0;        
   }
  } 

}
void serialEvent() {

 
      while (Serial.available()) {   
      Re_buf[counter]=(unsigned char)Serial.read();
      if(counter==0&&Re_buf[0]!=0x5A) return;      // 检查帧头         
      counter++;       
      if(counter==20)                //接收到数据
      {    
         counter=0;                 //重新赋值，准备下一帧数据的接收 
         sign=1;
      }      
      }

}
