/*
 * File:   main.c - BATLAB COMMS PIC
 * Author: Daniel Cambron
 *         Lexcelon, LLC
 * Created on June 5, 2016, 5:25 PM
 * 
 *  Lexcelon Batlab V1.0 Communications Processor Firmware
 *  Copyright (C) 2017 Daniel Cambron <daniel.cambron@lexcelon.com>
 *  Copyright (C) 2017 Lexcelon, LLC
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.

 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 *  
 */
#include <xc.h>
#include "usb.h"
#include <string.h>
#include "usb_config.h"
#include "usb_ch9.h"
#include "usb_cdc.h"
#include "main.h"
#include "hal.h"
#include "i2c.h"

__CONFIG(FOSC_INTOSC & WDTE_SWDTEN & PWRTE_ON & MCLRE_ON & CP_ON & BOREN_ON & CLKOUTEN_OFF & IESO_OFF & FCMEN_OFF);
__CONFIG(WRT_HALF & CPUDIV_NOCLKDIV & USBLSCLK_48MHz & PLLMULT_3x & PLLEN_ENABLED & STVREN_ON & BORV_LO & LPBOR_OFF & LVP_OFF);

void main()
{
    packet p,q,r; //temporary packets to handle message passing
    uint32_t i;
    uint32_t led_ctr = 0;
    hardware_init();
    
    
    LED4_PIN = LOW;
    for(i=0;i<100000;i++);
    LED0_PIN = LOW;
    for(i=0;i<100000;i++);
    LED3_PIN = LOW;
    for(i=0;i<100000;i++);
    LED2_PIN = LOW;
    for(i=0;i<100000;i++);
    LED1_PIN = LOW;
    for(i=0;i<100000;i++);
    LED4_PIN = HIGH;
    LED0_PIN = HIGH;
    LED3_PIN = HIGH;
    LED2_PIN = HIGH;
    LED0_PIN = HIGH; 
    LED1_PIN = HIGH;
    
    EXT_PSU_PIN  = HIGH;
  
    while(1)
    {
        if(led_ctr > 10) //no activity in a while, turn the COMMs LED off
        {
          led_ctr = 10;
          LED1_PIN    = HIGH; // turn the LED off  
        }
        if(get_usb_packet(&q)) // Handle incoming USB packet
        {
            LED1_PIN    = LOW;
            led_ctr = 0;
            if(q.payload[1] == NAMESPACE_COMMS) // If incoming packet is meant for this pic, send the USB response
            {
                process_packet(&r,&q); //actually do what the message says
                send_usb_packet(&r);
            }
            else
            {
                send_uart_packet(&q);
            } 
        } 
        if(get_uart_packet(&p)) // Queue outgoing USB packets from incoming UART packets
        {
            LED1_PIN    = LOW; //Signal COMMS activity
            led_ctr = 0;
            if(p.payload[1] == NAMESPACE_COMMS) //if the incoming packet is meant for this pic, send a uart response
            {
                process_packet(&q,&p);
                send_uart_packet(&q);
            }
            else
            {
                send_usb_packet(&p); //forward to PC
            }
        }
        service_psu();
        service_leds();
        service();  //this actually tx and rx all data that has been queued up in the buffers
        //for(i=0;i<1000;i++);
        led_ctr++;
    } // END WHILE(1))
} // END MAIN