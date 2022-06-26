#include "Mp3Module.h"

Mp3Module Mymp3(11,12,13); //对应引脚RX(11) TX(12) BUSY(13)

void setup() 
{
  Mymp3.VolumeSet(30);     //设置音量，最大值为30，默认音量为30
  Mymp3.Play(3,NoWait);    //播放003文件，不等待
}

void loop() {
  // put your main code here, to run repeatedly:

}
