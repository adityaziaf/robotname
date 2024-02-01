#ifndef __RC_DS4_H__
#define __RC_DS4_H__

#include "robotname_hardware/remote-controller/ds4_define.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
#include <unistd.h>
#include <thread>

#include "stdlib.h"
#include <stdio.h>
#include <sys/ioctl.h>
#include <mtd/mtd-user.h>
#include <errno.h>
#include <dirent.h> 
#include <string.h>

typedef struct {
    uint32_t button;
    int8_t lx;
    int8_t ly;
    int8_t rx;
    int8_t ry;
    uint8_t l2;
    uint8_t r2;
} dataJoy;

struct JoystickEvent {
    // static const short MIN_AXES_VALUE = -32768;
    // static const short MAX_AXES_VALUE = 32767;
    unsigned int time;
    short value;
    unsigned char type;
    unsigned char number;

    // bool isButton() { return (type & JS_EVENT_BUTTON) != 0; }
    // bool isAxis() { return (type & JS_EVENT_AXIS) != 0; }
    // bool isInitialState() { return (type & JS_EVENT_INIT) != 0; }
    // friend std::ostream& operator<<(std::ostream& os, const JoystickEvent& e);
};

class Joystick 
{
private:    
    const char* default_file = "/dev/input/js0";
    const char* led_file_loc = "/sys/class/leds/";
    const char* battery_file_loc = "/sys/class/power_supply/sony_controller_battery_";

    std::string led_class_id;
    std::string battery_class_id;
    uint8_t is_class_added = 0;
    int id;
public:
    dataJoy data;
    Joystick(char* file_fd) {
        // joy.buttons.push_back(0);
        std::cout << "joy object creeeated!" << std::endl;
        data = dataJoy();
        id = open(file_fd, O_RDONLY|O_NONBLOCK);
    };

    Joystick(char* file_fd, char* led_id, char* bat_id) {
        is_class_added = 1;

        std::cout << "joy object creeeated!" << std::endl;
        data = dataJoy();
        id = open(file_fd, O_RDONLY|O_NONBLOCK);

        led_class_id = std::string(led_id);
        battery_class_id = std::string(bat_id);
    }

    Joystick(char* led_id, char* bat_id) {
        is_class_added = 1;
        
        std::cout << "joy object creeeated!" << std::endl;
        data = dataJoy();
        id = open(default_file, O_RDONLY|O_NONBLOCK);

        led_class_id = std::string(led_id);
        battery_class_id = std::string(bat_id);
    }

    Joystick() {
        std::cout << "joy object creeeated!" << std::endl;
        data = dataJoy();
        id = open(default_file, O_RDONLY|O_NONBLOCK);
    };

    void printJoy() {
        std::string const jbut = "SQEXCRTRR1L1R2L2LFDWRGUPSTR3L3SL";
        //print ljoy
        std::cout << "L(" << (int16_t) data.lx << ", " << (int16_t) data.ly << ", " << (int16_t) data.l2 << ") ";
        //print rjoy
        std::cout << "R(" << (int16_t) data.rx << ", " << (int16_t) data.ry << ", " << (int16_t) data.r2 << ") ";
        //print button
        for(int i=0; i<17; i++) {
            int state = (data.button >> i) & 1;
            // if(state) std::cout << jbut.substr((15-i)*2, 2) << ' ';
            if(state) std::cout << button_str[i] << ' ';
        }
        std::cout << std::endl;

    }

    void event() {
        JoystickEvent events[5];
        ssize_t r1 = read(id, events, sizeof events[5]);
        if (r1 != -1) {
            int new_event_count = r1/sizeof(JoystickEvent);
            for (int evi=0; evi<new_event_count; evi++) {
                auto& ev = events[evi];
                // if(ev.type == EV_SYN) continue;
                switch (ev.type) {
                    case JS_EVENT_AXIS: {
                        switch (ev.number) {
                            case JS_AXIS_LX:   { data.lx = ev.value >> 8; if(data.lx == -128) data.lx = -127;} break;
                            case JS_AXIS_LY:   { data.ly = ev.value >> 8; if(data.ly == -128) data.ly = -127;} break;
                            case JS_AXIS_RX:   { data.rx = ev.value >> 8; if(data.rx == -128) data.rx = -127;} break;
                            case JS_AXIS_RY:   { data.ry = ev.value >> 8; if(data.ry == -128) data.ry = -127;} break;
                            case JS_AXIS_L2:   { data.l2 = ev.value; uint16_t hys = data.button; repbit(data.button, JS_L2, (ev.value>(127+(cekbit(hys, JS_L2)?-27:27))));} break;
                            case JS_AXIS_R2:   { data.r2 = ev.value; uint16_t hys = data.button; repbit(data.button, JS_R2, (ev.value>(127+(cekbit(hys, JS_R2)?-27:27))));} break;
                            case JS_AXIS_HATX: { repbit(data.button, JS_RG, (ev.value>0)); repbit(data.button, JS_LF, (ev.value<0)); } break;
                            case JS_AXIS_HATY: { repbit(data.button, JS_DW, (ev.value>0)); repbit(data.button, JS_UP, (ev.value<0)); } break;
                        }
                    } break;
                    case JS_EVENT_BUTTON: {
                        switch (ev.number) {
                            case JS_SHARE   : { repbit(data.button, JS_SHARE, ev.value); } break;
                            case JS_L3      : { repbit(data.button, JS_L3, ev.value); } break;
                            case JS_R3      : { repbit(data.button, JS_R3, ev.value); } break;
                            case JS_OPT     : { repbit(data.button, JS_OPT, ev.value); } break;
                            case JS_L1      : { repbit(data.button, JS_L1, ev.value); } break;
                            case JS_R1      : { repbit(data.button, JS_R1, ev.value); } break;
                            case JS_TR      : { repbit(data.button, JS_TR, ev.value); } break;
                            case JS_CR      : { repbit(data.button, JS_CR, ev.value); } break;
                            case JS_EX      : { repbit(data.button, JS_EX, ev.value); } break;
                            case JS_SQ      : { repbit(data.button, JS_SQ, ev.value); } break;
                            case JS_PS      : { repbit(data.button, JS_PS, ev.value); } break;
                        }
                    } break;
                }

                // printf("%d %x %d \n", ev.type, ev.number, ev.value);
                printJoy();
            }
        }
    };

    void change_led(int hex_color){
        char dir_name[3][100];
        char buf[50];
        
        DIR *d;
        struct dirent *dir;
        d = opendir("/sys/class/leds");
        if (d) {
            while ((dir = readdir(d)) != NULL) {
                // sprintf(dir_name[0], "%s", dir->d_name);
                memcpy(buf, dir->d_name, strlen(dir->d_name));
                buf[strlen(dir->d_name)] =  0;

                // printf("%s\n", &buf[20]);
                // printf("%s\n", &buf[0]);

                if(memcmp("red", &buf[20], 3) == 0){
                    memcpy(dir_name[0], "/sys/class/leds/", 16);
                    memcpy(&dir_name[0][16], buf, strlen(buf));
                }
                if(memcmp("green", &buf[20], 5) == 0){
                    memcpy(dir_name[1], "/sys/class/leds/", 16);
                    memcpy(&dir_name[1][16], buf, strlen(buf));
                }
                if(memcmp("blue", &buf[20], 4) == 0){
                    memcpy(dir_name[2], "/sys/class/leds/", 16);
                    memcpy(&dir_name[2][16], buf, strlen(buf));
                }
                memset(buf, 0, 50);
        
            }
            closedir(d);
        }
        else return;
        FILE *f1;
        FILE *f2;
        FILE *f3;

        memcpy(&dir_name[0][23+16], "/brightness\0", 12);
        memcpy(&dir_name[1][25+16], "/brightness\0", 12);
        memcpy(&dir_name[2][24+16], "/brightness\0", 12);

        // printf("%s\n", dir_name[0]);
        // printf("%s\n", dir_name[1]);
        // printf("%s\n", dir_name[2]);

        f1 = fopen(dir_name[0], "w+");
        f2 = fopen(dir_name[1], "w+");
        f3 = fopen(dir_name[2], "w+");

        if(f1 == NULL) {
            printf("Error opening file: red - %i : %s\n", errno, strerror(errno));   
            // exit(1);             
        }
        else{
            fprintf(f1,"%d",(hex_color >> 16) & 0xff);
            fclose(f1);
        }
        
        if(f2 == NULL) {
            printf("Error opening file: green - %i : %s\n", errno, strerror(errno));   
            // exit(1);             
        }
        else{
            fprintf(f2,"%d",(hex_color >> 8) & 0xff);
            fclose(f2);
        }

        if(f3 == NULL) {
            printf("Error opening file: blue - %i : %s\n", errno, strerror(errno));   
            // exit(1);             
        }
        else{
            fprintf(f3,"%d",(hex_color) & 0xff);
            fclose(f3);
        }
        
        return;
    };
};

#endif