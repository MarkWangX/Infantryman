
#ifndef DBUS_H
#define DBUS_H

#include "main.h"

/* ------------------------------ Macro Definition ------------------------------ */
#define SW_UP       0x01
#define SW_MID      0x03
#define SW_DOWN     0x02

/* ------------------------------ Type Definition ------------------------------ */
typedef struct {
    float LY; //left, vertical
    float LX; //left, horizontal
    float RY; //right, vertical
    float RX; //right, horizontal

    uint8_t sw1; //left switch
    uint8_t sw2; //right switch

    struct {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t l;
        uint8_t r;
    } mouse;

    union {  // 共用体，key_code和bit共享一段内存
        uint16_t key_code;
        struct {
            uint16_t W: 1;
            uint16_t S: 1;
            uint16_t A: 1;
            uint16_t D: 1;
            uint16_t SHIFT: 1;
            uint16_t CTRL: 1;
            uint16_t Q: 1;
            uint16_t E: 1;
            uint16_t R: 1;
            uint16_t F: 1;
            uint16_t G: 1;
            uint16_t Z: 1;
            uint16_t X: 1;
            uint16_t C: 1;
            uint16_t V: 1;
            uint16_t B: 1;
        } bit;
    } kb;

    float wheel;  //左上角拨轮
} DBUS_Type;

/* ------------------------------ Extern Global Variable ------------------------------ */
extern DBUS_Type dbus;

/* ------------------------------ Function Declaration (used in other .c files) ------------------------------ */
void Dbus_Init(void); //放在main
void Dbus_UART_IRQHandler(void); //放在中断处理函数中

#endif //DBUS_H

