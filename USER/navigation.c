// USER/navigation.c
#include "navigation.h"

// Define map dimensions (example values, in mm)
#define MAP_WIDTH 3000
#define MAP_HEIGHT 2000

// Define target points (example values, in mm)
#define POINT_A_X 500
#define POINT_A_Y 500
#define POINT_B_X 2500
#define POINT_B_Y 500
// ... and so on for C, D, E, F

/**
 * @brief  Navigate to predefined point A.
 * @note   This is a placeholder. Logic to be filled in later.
 *         Estimated time and path will be calculated here.
 */
void navigate_to_point_A(void)
{
    printf("> CMD: Navigating to Point A (%d, %d)...\r\n", POINT_A_X, POINT_A_Y);
    // motor_turn_left(100);
    // delay_ms(500); // Placeholder for turning time
    // motor_forward(150);
    // delay_ms(2000); // Placeholder for travel time
    // motor_stop();
    printf("> INFO: Reached Point A (simulated).\r\n");
}

/**
 * @brief  Navigate to predefined point B.
 */
void navigate_to_point_B(void)
{
    printf("> CMD: Navigating to Point B...\r\n");
    // Placeholder for actual navigation logic
    printf("> INFO: Reached Point B (simulated).\r\n");
}

/**
 * @brief  Navigate to predefined point C.
 */
void navigate_to_point_C(void)
{
    printf("> CMD: Navigating to Point C...\r\n");
    // Placeholder for actual navigation logic
    printf("> INFO: Reached Point C (simulated).\r\n");
}

/**
 * @brief  Navigate to predefined point D.
 */
void navigate_to_point_D(void)
{
    printf("> CMD: Navigating to Point D...\r\n");
    // Placeholder for actual navigation logic
    printf("> INFO: Reached Point D (simulated).\r\n");
}

/**
 * @brief  Navigate to predefined point E.
 */
void navigate_to_point_E(void)
{
    printf("> CMD: Navigating to Point E...\r\n");
    // Placeholder for actual navigation logic
    printf("> INFO: Reached Point E (simulated).\r\n");
}

/**
 * @brief  Navigate to predefined point F.
 */
void navigate_to_point_F(void)
{
    printf("> CMD: Navigating to Point F...\r\n");
    // Placeholder for actual navigation logic
    printf("> INFO: Reached Point F (simulated).\r\n");
} 
