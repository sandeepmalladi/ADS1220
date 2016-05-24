#include "mbed.h"
#include "ADS1220.h"
#include "DigoleSerialDisp.h"
#include "DataAnalysis.h"
#include "MedianFilt.h"
#include "math.h"


#define NWIDTH 20  /* Size of the data buffer; length of the sequence. */
#define STOPPER 0  /* Smaller than any datum */
#define AVERGARE_COUNT 1000 /*Count can be any number from 2,4,8,16 */

#define PGA 128                 // Programmable Gain = 1
#define VREF 5.0            // Internal reference of 2.048V
#define VFSR VREF/PGA             
#define FSR (((long int)1<<23)) 
#define LSB_Size (VFSR/FSR) 

ADS1220 ads1220_com(SPI_MOSI, SPI_MISO, SPI_SCK);
Serial _serial_comm(SERIAL_TX,SERIAL_RX);
Serial data_serial(USBTX, USBRX);
SerialDisp Lcd_display(LCD_MOSI, LCD_MISO, LCD_SCK);
InterruptIn DRDY(PC_5);
InterruptIn Button_press(USER_BUTTON);

unsigned char str[]                          = "PCE Force Gauge "; 
unsigned char removeweight_str[] = "Please Remove Weights....... ";
unsigned char placeweight_str[]  = "Calculating the offset..... ";





char New_data_avialable,button_pressed = 0,Analysis;
void ext_int_DRDY(void);
void showvolt(float volts);
void menu(void);
signed long avg_filter(signed long avg_val, int n);
void Caliberation(void);
void pressed(void);
float code2enob(float c);
void weight_cal(void);
signed long Median(signed long median_val, int n, int median_cnt);
float g2scale(float g, int scale);
float medfilter(signed long datum);
float median(signed long data[]);
float Mode_Filter(signed long avg_val, int n);
float code2volt(float c);

signed long tData,avg_tdata;
unsigned config_Buff[4];
float offset_Val = 0, actual_weight = 0,disp_wt,volt;
char get_char;

int main() 
{
   char buf[10],time_buf[14];
     Button_press.mode(PullUp);
     // Delay for initial pullup to take effect
   wait(0.1);
   DRDY.rise(&ext_int_DRDY);
     Button_press.fall(&pressed);
   ads1220_com.Config(); 
   ads1220_com.SendStartCommand();
     Lcd_display.LCD12864_Initialise();
     data_serial.baud(115200);
     // Setup a serial interrupt function to receive data
   _serial_comm.attach(&SerialData::Rx_interrupt, _serial_comm.RxIrq);
     // Setup a serial interrupt function to transmit data
   _serial_comm.attach(&SerialData::Tx_interrupt, _serial_comm.TxIrq);
     Lcd_display.LCD12864_DrawLogo();
     set_time(1887180123); // Set RTC time to 16 December 2013 10:05:23 UTC
   while(1) 
   {
         clock_t seconds = time(NULL);
                 weight_cal();
         strftime(time_buf,sizeof(time_buf), "%I:%M:%S (%p)", localtime(&seconds));
               Lcd_display.LCD12864_DisplayString(0,0,(uint8_t *)&str[0],COUNTOF(str));
             Lcd_display.LCD12864_DisplayString(1,0,(uint8_t *)&time_buf[0],COUNTOF(time_buf));
             sprintf(buf,"%.4f KG", (float)disp_wt);
                 wait_ms(2);
             Lcd_display.LCD12864_DisplayString(2,2,(uint8_t *)&buf[0],COUNTOF(buf));
             wait(0.5);
         if(Button_press)
         {
             Analysis = 1;
         }
   }
}



void weight_cal(void)
{
         if(button_pressed)
         {
                                button_pressed = 0;
                if(New_data_avialable)
                {
                        New_data_avialable = 0;
                        Caliberation();
                        wait(1);
                }
                    }
                    else
                    {
                            if(New_data_avialable)
                            {
                                    //led = !led;
                                    New_data_avialable = 0;
                                    tData = ads1220_com.ReadData();
                                  volt = code2volt(tData);
                                  data_serial.printf("\rVoltage : %f\n",volt);
                                    avg_tdata = Mode_Filter(tData, AVERGARE_COUNT);
                                    //data_serial.printf("\r\nHex code  %x",avg_tdata);
                                    actual_weight = avg_tdata - offset_Val;
                                    disp_wt = actual_weight/weight_gram ;
                                    disp_wt = disp_wt * gain_error_correction;
                                    disp_wt = g2scale(disp_wt,3);
                                    //data_serial.printf("\r%.4f\n",disp_wt);
                                    //data_serial.printf("\t KG\n");    
                            }
                    }
}


float code2volt(float c)
{
     float Vout = 0;
     Vout = (float)(c*LSB_Size*1000);    //In  mV
   return Vout;
}


float g2scale(float g, int scale)
{
    switch(scale) 
    {
    case 0:
        return g*1e6;
    case 1:
        return g*1e3;
    case 2:
        return g;
    case 3:
        return g/1e3;
    case 4:
        return g*(16.0/453.59237);
    case 5:
        return g/453.59237;
    case 6:
        return g/(14.0*453.59237);
    default: // something wrong
        return 0;
    }
}

float code2enob(float c)
{
    if (c==0) return 24.0;
    return 24.0- logf(c);

}

void pressed(void)
{
    button_pressed = 1;
}
signed long avg_filter(signed long avg_val, int n)
{
    int i;
    signed long mean;
    mean=0;
    for (i=0;i<n;++i) 
        mean+= avg_val;
    return mean/n;
}
void Caliberation(void)
{
    signed long cal_offset = 0;
    Lcd_display.LCD12864_CLEAR();
        wait_ms(2);
    Lcd_display.LCD12864_DisplayString(1,0,(unsigned char *)&removeweight_str[0],COUNTOF(removeweight_str));
    data_serial.printf("\rPlease Remove Weights.......\n");
    wait(2);
        Lcd_display.LCD12864_CLEAR();
        wait_ms(2);
    Lcd_display.LCD12864_DisplayString(1,0,(unsigned char *)&placeweight_str[0],COUNTOF(placeweight_str));
    data_serial.printf("\rCalculating the offset......\n");
    wait(2);
    tData = ads1220_com.ReadData();
      offset_Val = Mode_Filter(tData, AVERGARE_COUNT);
    cal_offset = (offset_Val / (weight_gram));
      cal_offset = cal_offset * gain_error_correction;
        cal_offset = g2scale(cal_offset,3);
    data_serial.printf("\r%.4f",(float *)cal_offset);
      Lcd_display.LCD12864_CLEAR();
    wait_ms(1);
}


float Mode_Filter(signed long avg_val, int n)
{
        float returnval = 0;
   // read multiple values and sort them to take the mode
   signed long sortedValues[n];
   for(int i=0;i<n;i++)
     {
     signed long value = avg_val;
     int j;
     if(value < sortedValues[0] || i==0)
         {
        j=0; //insert at first position
     }
     else
         {
       for(j=1;j<i;j++){
          if(sortedValues[j-1]<=value && sortedValues[j]>=value)
                    {
            // j is insert position
            break;
          }
       }
     }
     for(int k=i;k>j;k--)
         {
       // move all values higher than current reading up one position
       sortedValues[k]=sortedValues[k-1];
     }
     sortedValues[j]=value; //insert current reading
   }
   //return scaled mode of 700 values
   for(int i=200;i<800;i++)
     {
     returnval +=sortedValues[i];
   } 
   returnval = returnval/600;
   return returnval;
}

void Set_Disp_Mux_config()
{
            char Mux_data;

        
                Mux_data = data_serial.getc();
                                data_serial.printf("\n");

              switch(Mux_data) 
                {
                        
                     case '0':
                                        ads1220_com.set_MUX(0);
                                        data_serial.printf("Input Multiplexer Configuration Set to 0: AINP = AIN0, AINN = AIN1\n");
                                        ads1220_com.ReadRegister(ADS1220_0_REGISTER, 0x01, &config_Buff[0]);
                                        data_serial.printf("\r REG0 : %x\n",config_Buff[0]);
                     break;
                     case '1':
                                        ads1220_com.set_MUX(1);
                                        data_serial.printf("Input Multiplexer Configuration Set to 1: AINP = AIN0, AINN = AIN2\n");
                                        ads1220_com.ReadRegister(ADS1220_0_REGISTER, 0x01, &config_Buff[0]);
                                        data_serial.printf("\r REG0 : %x\n",config_Buff[0]);                                          
                     break;
                     case '2':
                                        ads1220_com.set_MUX(2);
                                        data_serial.printf("Input Multiplexer Configuration Set to 2: AINP = AIN0, AINN = AIN3\n");
                                        ads1220_com.ReadRegister(ADS1220_0_REGISTER, 0x01, &config_Buff[0]);
                                        data_serial.printf("\r REG0 : %x\n",config_Buff[0]);                                          
                     break;
                     case '3':
                                        ads1220_com.set_MUX(3);
                                        data_serial.printf("Input Multiplexer Configuration Set to 3: AINP = AIN1, AINN = AIN2\n");
                                        ads1220_com.ReadRegister(ADS1220_0_REGISTER, 0x01, &config_Buff[0]);
                                        data_serial.printf("\r REG0 : %x\n",config_Buff[0]);                                          
                     break;
                     case '4':
                                        ads1220_com.set_MUX(4);
                                        data_serial.printf("Input Multiplexer Configuration Set to 4: AINP = AIN1, AINN = AIN3\n");
                                        ads1220_com.ReadRegister(ADS1220_0_REGISTER, 0x01, &config_Buff[0]);
                                        data_serial.printf("\r REG0 : %x\n",config_Buff[0]);                                          
                     break;
                     case '5':
                                        ads1220_com.set_MUX(5);
                                        data_serial.printf("Input Multiplexer Configuration Set to 5: AINP = AIN2, AINN = AIN3\n");
                                        ads1220_com.ReadRegister(ADS1220_0_REGISTER, 0x01, &config_Buff[0]);
                                        data_serial.printf("\r REG0 : %x\n",config_Buff[0]);                                          
                     break;
                     case '6':
                                        ads1220_com.set_MUX(6);
                                        data_serial.printf("Input Multiplexer Configuration Set to 6: AINP = AIN1, AINN = AIN0\n");
                                        ads1220_com.ReadRegister(ADS1220_0_REGISTER, 0x01, &config_Buff[0]);
                                        data_serial.printf("\r REG0 : %x\n",config_Buff[0]);                                          
                     break;
                     case '7':
                                        ads1220_com.set_MUX(7);
                                        data_serial.printf("Input Multiplexer Configuration Set to 7: AINP = AIN3, AINN = AIN2\n");
                                        ads1220_com.ReadRegister(ADS1220_0_REGISTER, 0x01, &config_Buff[0]);
                                        data_serial.printf("\r REG0 : %x\n",config_Buff[0]);                                          
                     break;
                        case '8':
                                        ads1220_com.set_MUX(8);
                                        data_serial.printf("Input Multiplexer Configuration Set to 8: AINP = AIN0, AINN = AVSS\n");
                                        ads1220_com.ReadRegister(ADS1220_0_REGISTER, 0x01, &config_Buff[0]);
                                        data_serial.printf("\r REG0 : %x\n",config_Buff[0]);                                          
                     break;
                         case '9':
                                        ads1220_com.set_MUX(9);
                                        data_serial.printf("Input Multiplexer Configuration Set to 9: AINP = AIN1, AINN = AVSS\n");
                                        ads1220_com.ReadRegister(ADS1220_0_REGISTER, 0x01, &config_Buff[0]);
                                        data_serial.printf("\r REG0 : %x\n",config_Buff[0]);                                          
                     break;
                     default:               
                        break;
                                                             

            }
}

void Set_Disp_Gain_config(void)
{
                char Mux_data;

        
                Mux_data = data_serial.getc();
                                data_serial.printf("\n");

              switch(Mux_data) 
                {
                        
                     case '0':
                                        ads1220_com.set_GAIN(0);
                                        data_serial.printf("Set Gain Configuration to 0 for Gain = 1 \n");
                                        ads1220_com.ReadRegister(ADS1220_0_REGISTER, 0x01, &config_Buff[0]);
                                        data_serial.printf("\r REG0 : %x\n",config_Buff[0]);
                     break;
                     case '1':
                                        ads1220_com.set_GAIN(1);
                                        data_serial.printf("Set Gain Configuration to 1 for Gain = 2 \n");
                                        ads1220_com.ReadRegister(ADS1220_0_REGISTER, 0x01, &config_Buff[0]);
                                        data_serial.printf("\r REG0 : %x\n",config_Buff[0]);                                          
                     break;
                     case '2':
                                        ads1220_com.set_GAIN(2);
                                        data_serial.printf("Set Gain Configuration to 2 for Gain = 4 \n");
                                        ads1220_com.ReadRegister(ADS1220_0_REGISTER, 0x01, &config_Buff[0]);
                                        data_serial.printf("\r REG0 : %x\n",config_Buff[0]);                                          
                     break;
                     case '3':
                                        ads1220_com.set_GAIN(3);
                                        data_serial.printf("Set Gain Configuration to 3 for Gain = 8 \n");
                                        ads1220_com.ReadRegister(ADS1220_0_REGISTER, 0x01, &config_Buff[0]);
                                        data_serial.printf("\r REG0 : %x\n",config_Buff[0]);                                          
                     break;
                     case '4':
                                        ads1220_com.set_GAIN(4);
                                        data_serial.printf("Set Gain Configuration to 4 for Gain = 16 \n");
                                        ads1220_com.ReadRegister(ADS1220_0_REGISTER, 0x01, &config_Buff[0]);
                                        data_serial.printf("\r REG0 : %x\n",config_Buff[0]);                                          
                     break;
                     case '5':
                                        ads1220_com.set_GAIN(5);
                                        data_serial.printf("Set Gain Configuration to 5 for Gain = 32 \n");
                                        ads1220_com.ReadRegister(ADS1220_0_REGISTER, 0x01, &config_Buff[0]);
                                        data_serial.printf("\r REG0 : %x\n",config_Buff[0]);                                          
                     break;
                     case '6':
                                        ads1220_com.set_GAIN(6);
                                        data_serial.printf("Set Gain Configuration to 6 for Gain = 64 \n");
                                        ads1220_com.ReadRegister(ADS1220_0_REGISTER, 0x01, &config_Buff[0]);
                                        data_serial.printf("\r REG0 : %x\n",config_Buff[0]);                                          
                     break;
                     case '7':
                                        ads1220_com.set_GAIN(7);
                                        data_serial.printf("Set Gain Configuration to 7 for Gain = 128 \n");
                                        ads1220_com.ReadRegister(ADS1220_0_REGISTER, 0x01, &config_Buff[0]);
                                        data_serial.printf("\r REG0 : %x\n",config_Buff[0]);                                          
                     break;
                     default:               
                        break;
                 }
}

void Set_Disp_Bypass_config(void)
{
    char Bypass_data;

        
                Bypass_data = data_serial.getc();
        data_serial.printf("\n");

              switch(Bypass_data) 
                {
                        
                     case '0':
                                        ads1220_com.set_PGA_BYPASS(0);
                                        data_serial.printf("Set PGA Bypass Configuration to 0 for PGA Enabling \n");
                                        ads1220_com.ReadRegister(ADS1220_0_REGISTER, 0x01, &config_Buff[0]);
                                        data_serial.printf("\r REG0 : %x\n",config_Buff[0]);
                     break;
                     case '1':
                                        ads1220_com.set_PGA_BYPASS(1);
                                        data_serial.printf("Set PGA Bypass Configuration to 1 for PGA Disabled and bypassed\n");
                                        ads1220_com.ReadRegister(ADS1220_0_REGISTER, 0x01, &config_Buff[0]);
                                        data_serial.printf("\r REG0 : %x\n",config_Buff[0]);                                          
                     break;
                     
                     default:               
                        break;
            }
}

void menu(void)
{
    uint8_t data;
    
        data_serial.printf("Menu :\n");
        data_serial.printf("\r0.Set Input Multiplexer Configuration\n");
        data_serial.printf("\r1.Set Gain\n");
        data_serial.printf("\r2.Set Data Rate\n");
        data_serial.printf("\r3.Set Operation Mode\n");
        data_serial.printf("\r4.Set Conversion Mode\n");
        data_serial.printf("\r5.Set Temprature Sensor Mode\n");
        data_serial.printf("\r6.Set Burn out Current Sources\n");
        data_serial.printf("\r7.Set Voltage Refernce Selection\n");
        data_serial.printf("\r8.Set Power Down Configuration\n");
    
            data = data_serial.getc();
      data_serial.printf("\n");
      switch(data) 
            {
         case '0':
                                    data_serial.printf("\rEnter the value to set Input Multiplexer Configuration\n");
                                    data_serial.printf("\r0: AINP = AIN0, AINN = AIN1 \n");
                                    data_serial.printf("\r1: AINP = AIN0, AINN = AIN2 \n");
                                    data_serial.printf("\r2: AINP = AIN0, AINN = AIN3 \n");
                                    data_serial.printf("\r3: AINP = AIN1, AINN = AIN2 \n");
                                    data_serial.printf("\r4: AINP = AIN1, AINN = AIN3 \n");
                                    data_serial.printf("\r5: AINP = AIN2, AINN = AIN3 \n");
                                    data_serial.printf("\r6: AINP = AIN1, AINN = AIN0 \n");
                                    data_serial.printf("\r7: AINP = AIN3, AINN = AIN2 \n");
                                    data_serial.printf("\r8: AINP = AIN0, AINN = AVSS \n");
                                    data_serial.printf("\r9: AINP = AIN1, AINN = AVSS \n");
                                    data_serial.printf("\rs.Exit\n");
                                    
                  Set_Disp_Mux_config(); 
                    break;  
                    case '1':
                                    data_serial.printf("\rEnter the value to set Gain\n");
                                    data_serial.printf("\r0: Gain = 1 \n");
                                    data_serial.printf("\r1: Gain = 2 \n");
                                    data_serial.printf("\r2: Gain = 4 \n");
                                    data_serial.printf("\r3: Gain = 8 \n");
                                    data_serial.printf("\r4: Gain = 16 \n");
                                    data_serial.printf("\r5: Gain = 32 \n");
                                    data_serial.printf("\r6: Gain = 64 \n");
                                    data_serial.printf("\r7: Gain = 128 \n");
                                    data_serial.printf("\rs.Exit\n");
                                    
                  Set_Disp_Gain_config(); 
                    break;
                    case '2':
                                    data_serial.printf("\rEnter the value to set Bypass\n");
                                    data_serial.printf("\r0: PGA Enabled\n");
                                    data_serial.printf("\r1: PGA Disabled\n");
                                    data_serial.printf("\rs.Exit\n");
                                    
                  Set_Disp_Bypass_config(); 
                    break;

                    default:                
                    break;
         }
                    
}




void ext_int_DRDY(void)
{
  New_data_avialable = 1;
}
