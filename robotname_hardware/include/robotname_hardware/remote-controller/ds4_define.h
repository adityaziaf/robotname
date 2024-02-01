#ifndef __RC_DS4_DEFINE_H__
#define __RC_DS4_DEFINE_H__

#include <string>

#define JS_EX           0       //0
#define JS_CR           1       //1
#define JS_TR           2       //2
#define JS_SQ           3       //3
#define JS_L1           4      //4
#define JS_R1           5      //5
#define JS_L2           6      //6
#define JS_R2           7     //7
#define JS_SHARE        8     //8
#define JS_SELECT       8    
#define JS_OPT          9     //9
#define JS_START        9
#define JS_PS           10    //10
#define JS_L3           11    //11
#define JS_R3           12    //12
#define JS_UP           13
#define JS_DW           14
#define JS_LF           15
#define JS_RG           16

#define JS_AXIS_LX      0x00
#define JS_AXIS_LY      0x01
#define JS_AXIS_L2      0x02
#define JS_AXIS_RX      0x03
#define JS_AXIS_RY      0x04
#define JS_AXIS_R2      0x05
#define JS_AXIS_HATX    0x06
#define JS_AXIS_HATY    0x07

#define JS_LED_RED      0xFF0000
#define JS_LED_GREEN    0x00FF00
#define JS_LED_BLUE     0x0000FF
#define JS_LED_WHITE    0xFFFFFF
#define JS_LED_PURPLE   0xFF00FF
#define JS_LED_YELLOW   0xFFFF00
#define JS_LED_CYAN     0x00FFFF

#define JS_EVENT_BUTTON 0x01 // button pressed/released
#define JS_EVENT_AXIS   0x02 // joystick moved
#define JS_EVENT_INIT   0x80 // initial state of device

/*Set Bit*/
#define setbit(var, bit) {var |= (1<<bit);}
/*Clear Bit*/
#define clrbit(var, bit) {var &=~(1<<bit);}
/*Replace Bit*/
#define repbit(var, bit, l) {clrbit(var, bit); var |= l<<bit;}
/*Check Bit*/
#define cekbit(var, bit) (((var>>bit) & 0x01) == 1)
/*Check All Bit*/
#define cekallbit(var, bit) ((1<<bit) == var)

std::string const button_str[] = {"EX", "CR", "TR", "SQ", "L1", "R1", "L2", "R2", "SH", "OP", "PS", "L3", "R3", "UP", "DW", "LF", "RG"};

#define getButtonString(button) button_str[button]

#endif //__RC_DS4_DEFINE_H__