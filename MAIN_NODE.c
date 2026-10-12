#include <lpc21xx.h>
#include "types.h"
#include "delays.h"
#include "LCD.h"
#include "DS18B20.h"
#include "CAN.h"
#include "dashboard.h"
#include "EINT.h"
#include "project_functions.h"

int main(void)
{
    f32 engine_temp = 32.5f;
    f32 read_val;
    u8 window_current_level = 0; /* 0 (Fully Open) to 8 (Fully Closed) */
    u8 ecu_fault_flag = 0;       /* 0 = All ECUs Healthy, 1 = ECU Offline */
    
    /* 1. Initialize LCD Display (P0.0 - P0.10) */
    LCD_init();
    
    /* 2. Show Modern 20x4 Splash Screen */
    display_splash_screen();
    
    /* 3. Initialize Switch Inputs (P0.14, P0.15, P0.16) */
    Switches_Init();
    
    /* 4. Initialize CAN1 Peripheral (250 kbps on P0.24, P0.25) */
    CAN1_Init();
    
    /* 5. Main Execution Loop */
    while(1)
    {
        /* Check Window UP Switch (P0.16) */
        if(flag_win_up)
        {
            flag_win_up = 0; /* Clear flag */
            delay_ms(20); /* Debounce */
            if(Is_Switch_Win_Up_Pressed())
            {
                window_current_level = Process_Window_Control(WINDOW_CMD_UP, window_current_level, &ecu_fault_flag);
            }
        }
        
        /* Check Window DOWN Switch (P0.14) */
        if(flag_win_down)
        {
            flag_win_down = 0; /* Clear flag */
            delay_ms(20); /* Debounce */
            if(Is_Switch_Win_Down_Pressed())
            {
                window_current_level = Process_Window_Control(WINDOW_CMD_DOWN, window_current_level, &ecu_fault_flag);
            }
        }
        
        /* Check Reverse Gear Switch (P0.15) */
        if(flag_reverse)
        {
            flag_reverse = 0; /* Clear flag */
            if(Read_Switch_Reverse())
            {
                Process_Reverse_Mode(&ecu_fault_flag);
            }
        }
        
        /* Sample Engine Temperature from DS18B20 Sensor (P0.22) */
        read_val = DS18B20_ReadTemperature();
        if(read_val > -50.0f && read_val < 150.0f)
        {
            engine_temp = read_val;
        }
        else
        {
            engine_temp = -999.0f; /* Sensor disconnected indicator */
        }
        
        /* Render Real-Time 20x4 LCD Telemetry Dashboard */
        display_dashboard(engine_temp, window_current_level, REVERSE_DISABLE, ecu_fault_flag);
        
        delay_ms(150);
    }
    
    return 0;
}
