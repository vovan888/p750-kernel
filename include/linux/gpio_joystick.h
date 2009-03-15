#ifndef _GPIO_JOYSTICK_H
#define _GPIO_JOYSTICK_H

struct gpio_joystick_platform_data {
	int gpios[5];	/* gpios for PUSH (irq), NW, NE, SE, SW directions */
	int keycodes[5];/* keycodes for PUSH, UP, LEFT, DOWN, RIGHT directions */
};

#endif
