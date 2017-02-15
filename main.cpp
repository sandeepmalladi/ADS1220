#include "mbed.h"
#include "ADS1220.h"
#include "math.h"


#define PGA 128                                     // Programmable Gain = 1
#define VREF 5.0                                    // Internal reference of 2.048V
#define VFSR VREF/PGA             
#define FSR (((long int)1<<23)) 
#define LSB_Size (VFSR/FSR) 

ADS1220 ads1220_com(SPI_MOSI, SPI_MISO, SPI_SCK,SPI_CS);
Serial data_serial(USBTX, USBRX);
InterruptIn DRDY(PC_5);





bool New_data_avialable;
void ext_int_DRDY(void);
float code2volt(float c);

signed long tData;
float volt;

int main() 
{
   data_serial.baud(115200);
   wait(0.1);
   DRDY.rise(&ext_int_DRDY);
   ads1220_com.Config(); 
   ads1220_com.SendStartCommand();
   while(1) 
   {
         if(New_data_avialable)
          {
            New_data_avialable = 0;
            tData = ads1220_com.ReadData();
            volt = code2volt(tData);
            data_serial.printf("\rVoltage : %f\n",volt);
          }
   }
}

float code2volt(float c)
{
     float Vout = 0;
     Vout = (float)(c*LSB_Size*1000);    //In  mV
   return Vout;
}

void ext_int_DRDY(void)
{
  New_data_avialable = 1;
}
