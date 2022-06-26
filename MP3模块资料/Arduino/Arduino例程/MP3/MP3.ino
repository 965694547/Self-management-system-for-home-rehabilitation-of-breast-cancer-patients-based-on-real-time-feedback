#include <U8glib.h>

//#include <Mp3Module.h>

//#include "Mp3Module.h"

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE);
Mp3Module Mymp3(10,12,13);
void setup() 
{
  Mymp3.VolumeSet(28);
  Mymp3.Play(1,NoWait);
  
}

void loop() 
{
}
