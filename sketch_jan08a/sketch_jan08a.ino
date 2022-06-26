int wq=0;

void setup()
{
  // 初始化串口
  Serial.begin(9600);
}
void loop() 
{
  int sensorValue1 = analogRead(A0);
  int sensorValue2 = analogRead(A3);
  if(sensorValue1>=100)
  {
    wq=wq+1;  
    while(1)
    {
     sensorValue1 = analogRead(A3);
    if(sensorValue1<100)
    break;  
    }
  }
  Serial.print(wq);
  Serial.print(',');
  Serial.print(sensorValue1);
  Serial.print(',');
  Serial.print(sensorValue2);
  Serial.println(';');
  delay(500);
}
