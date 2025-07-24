// USER/mode_choose.c
#include "mode_choose.h"

static ControlMode current_mode;

/**
 * @brief  Initialize the control mode module.
 */
void mode_init(void)
{
    // Default to HMI mode on startup
    current_mode = MODE_HMI;
}

/**
 * @brief  Set the active control mode.
 * @param  mode: The new mode to set (MODE_HMI or MODE_ROS).
 */
void set_control_mode(ControlMode mode)
{
    if (current_mode != mode)
    {
        current_mode = mode;
        motor_stop(); // Safety feature: stop motors when switching modes
        if (mode == MODE_HMI)
        {
            printf("> Switched to HMI control mode.\r\n");
        }
        else
        {
            printf("> Switched to ROS control mode.\r\n");
        }
    }
}

/**
 * @brief  Get the current active control mode.
 * @retval The current control mode.
 */
ControlMode get_control_mode(void)     
{
    return current_mode;
} 
