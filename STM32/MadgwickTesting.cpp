#include <stm32f7xx_hal.h>
#include <stm32_hal_legacy.h>

/* Custom interface to all the STM32 peripherals. Can be found here:
 * https://bitbucket.org/codex653/thor_stm32f767zit 
 * 
 * Note that this library is NOT required for the actual filter to work. 
 * It is simply used here for testing on my actual embedded system. Feel
 * free to use it or not. */
#include "thor.h"

int main(void)
{
	HAL_Init();
	ThorSystemClockConfig();

	for (;;)
	{
		
	}
}
