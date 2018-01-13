#include "Copter.h"



#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    c_buff = 0;     //0: temp-buf
    c_state = 0;    //0: wait-start-buff, 1: data-buffer, 2: end-buffer-->0
    ips_bytes = 0;
    s16_US_HEIGHT = 156;
    optflow.init();
    
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here

    // uartF: serial5, baud 115200
//================================IPS_POSITION====================================//
    // Get available bytes
    ips_bytes = hal.uartF->available();
    while (ips_bytes-- > 0) {
        // Get data string here
        ips_char[0] = hal.uartF->read();
        if(ips_char[0] == 's'){
            c_buff = 1;
            c_state = 1;
        }
        else if(ips_char[0] == 'e'){
            // end-of-frame: get ips_pos & time_stamp
            if(ips_char[5] ==','){
                // valid frame
                ips_data[0] = ips_char[1] - 0x30;
                ips_data[1] = (ips_char[2]-0x30)*100 + (ips_char[3]-0x30)*10 + (ips_char[4]-0x30); //pos_x
                ips_data[2] = (ips_char[6]-0x30)*100 + (ips_char[7]-0x30)*10 + (ips_char[8]-0x30); //pos_y
            }
            c_buff = 0;
            c_state = 0;
        }
        else{
            if(c_state == 1){
                ips_char[c_buff] = ips_char[0];
                // hal.uartF->printf("%c",ips_char[c_buff]);
                c_buff++;
            }
        }
    }

    //
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 20Hz code here
//==============================TEMPERATURE======================================//
    air_temperature = barometer.get_temperature();
    // hal.uartF->printf("temp:%f",air_temperature);

//==============================IPS_TRANSMIT======================================//
    hal.uartE->printf("{PARAM,TRIGGER_US}\n");
    ips_delay_ms = AP_HAL::millis();    // trigger IPS_transmission on Tiva C
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif
