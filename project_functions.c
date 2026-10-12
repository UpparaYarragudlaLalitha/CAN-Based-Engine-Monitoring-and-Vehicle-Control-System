#include "project_functions.h"
#include "dashboard.h"
#include "delays.h"
#include "EINT.h"

u8 Process_Window_Control(u8 action, u8 current_level, u8 *fault_flag)
{
    CAN_MSG tx_msg, rx_msg;
    u16 timeout;
    u8 response_received;
    
    tx_msg.id = CAN_ID_WINDOW_CTRL;
    tx_msg.dlc = 1;
    tx_msg.rtr = 0;
    tx_msg.data[0] = action;
    
    /* 1. Boundary Checks */
    if(action == WINDOW_CMD_UP && current_level >= 8)
    {
        display_window_screen(WINDOW_CMD_UP, 8);
        delay_ms(400);
        LCD_clear();
        return 8;
    }
    if(action == WINDOW_CMD_DOWN && current_level == 0)
    {
        display_window_screen(WINDOW_CMD_DOWN, 0);
        delay_ms(400);
        LCD_clear();
        return 0;
    }
    
    /* 2. Transmit Window Command Frame (0x101) */
    CAN1_Tx(&tx_msg);
    
    /* 3. Wait for Node Response (0x201) with Timeout (500 ms max) */
    response_received = 0;
    for(timeout = 0; timeout < 50; timeout++)
    {
        if(CAN1_Rx(&rx_msg))
        {
            if(rx_msg.id == CAN_ID_WINDOW_STATUS)
            {
                current_level = rx_msg.data[0];
                response_received = 1;
                *fault_flag = 0; /* Clear ECU fault */
                break;
            }
        }
        delay_ms(10);
    }
    
    /* 4. Handle Node Timeout / Disconnection Error */
    if(!response_received)
    {
        *fault_flag = 1; /* Set ECU fault */
        display_node_error_screen(NODE_TYPE_WINDOW);
        return current_level;
    }
    
    /* Display updated position */
    display_window_screen(action, current_level);
    delay_ms(200);
    
    /* 5. Continuous Press (Holding the button) */
    while((action == WINDOW_CMD_UP && Is_Switch_Win_Up_Pressed()) ||
          (action == WINDOW_CMD_DOWN && Is_Switch_Win_Down_Pressed()))
    {
        if((action == WINDOW_CMD_UP && current_level < 8) ||
           (action == WINDOW_CMD_DOWN && current_level > 0))
        {
            CAN1_Tx(&tx_msg);
            
            response_received = 0;
            for(timeout = 0; timeout < 30; timeout++)
            {
                if(CAN1_Rx(&rx_msg))
                {
                    if(rx_msg.id == CAN_ID_WINDOW_STATUS)
                    {
                        current_level = rx_msg.data[0];
                        response_received = 1;
                        break;
                    }
                }
                delay_ms(10);
            }
            
            if(!response_received)
            {
                *fault_flag = 1;
                display_node_error_screen(NODE_TYPE_WINDOW);
                return current_level;
            }
            
            display_window_screen(action, current_level);
            delay_ms(200);
        }
        else
        {
            display_window_screen(action, current_level);
            delay_ms(100);
            break;
        }
    }
    
    delay_ms(300);
    LCD_clear();
    return current_level;
}

void Process_Reverse_Mode(u8 *fault_flag)
{
    CAN_MSG tx_msg, rx_msg;
    u8 reverse_alert = 0;
    u8 reverse_distance = 0;
    u16 watchdog_counter = 0;

    /* Enable Reverse Alert Node. */
    tx_msg.id = CAN_ID_REVERSE_ENABLE;
    tx_msg.dlc = 1;
    tx_msg.rtr = 0;
    tx_msg.data[0] = REVERSE_ENABLE;
    if(!CAN1_Tx(&tx_msg))
    {
        *fault_flag = 1;
        display_node_error_screen(NODE_TYPE_REVERSE);
        return;
    }

    LCD_clear();

    while(1)
    {
        /* Leave reverse mode when the reverse switch is released. */
        if(!Read_Switch_Reverse())
            break;

        if(CAN1_Rx(&rx_msg))
        {
            if(rx_msg.id == CAN_ID_REVERSE_ALERT && rx_msg.dlc >= 2)
            {
                reverse_alert = (rx_msg.data[0] != 0) ? 1 : 0;
                reverse_distance = rx_msg.data[1];
                watchdog_counter = 0;
                *fault_flag = 0;

                LCD_set_cursor(1, 0);
                LCD_string("<< REVERSE RADAR >> ");

                LCD_set_cursor(2, 0);
                LCD_string("DIST : ");
                if(reverse_distance < 100) LCD_string(" ");
                if(reverse_distance < 10)  LCD_string(" ");
                LCD_integer(reverse_distance);
                LCD_string(" cm       ");

                LCD_set_cursor(3, 0);
                if(reverse_alert)
                {
                    LCD_string("STATUS: OBSTACLE!   ");
                }
                else
                {
                    LCD_string("STATUS: PATH CLEAR  ");
                }

                LCD_set_cursor(4, 0);
                if(reverse_alert)
                {
                    LCD_string("STOP VEHICLE        ");
                }
                else
                {
                    LCD_string("SAFE TO MOVE        ");
                }
            }
            /* Ignore unrelated CAN frames without declaring a fault. */
        }
        else
        {
            watchdog_counter++;
            /* 16 x 50 ms = 800 ms without a valid reverse-alert frame. */
            if(watchdog_counter >= 16)
            {
                *fault_flag = 1;
                display_node_error_screen(NODE_TYPE_REVERSE);
                break;
            }
        }

        delay_ms(50);
    }

    /* Disable Reverse Alert Node on exit. */
    tx_msg.id = CAN_ID_REVERSE_ENABLE;
    tx_msg.dlc = 1;
    tx_msg.rtr = 0;
    tx_msg.data[0] = REVERSE_DISABLE;
    CAN1_Tx(&tx_msg);

    LCD_clear();
}
