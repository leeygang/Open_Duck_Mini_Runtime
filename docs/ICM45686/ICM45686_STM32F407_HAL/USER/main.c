#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "IMU.h"


// VCC--------5V或者3.3V都可以
// SPI 模式接线
// PA4 ------------------------CS
// PA5 ------------------------SCLK
// PA6------------------------MISO
// PA7------------------------MOSI

// IIC 接线
// PC1 ------------------------SCL
// PC2 ------------------------SDA
/* Interface reference is given as a parameter
     * For I2C : BMI2_I2C_INTF
     * For SPI : BMI2_SPI_INTF
     */
float motion6[7];
float ypr[3];          // 上传yaw pitch roll的值
int math_pl=0;

int main(void)
{
	
    HAL_Init();                   	//初始化HAL库    
    Stm32_Clock_Init(336,8,2,7);  	//设置时钟,168Mhz
	delay_init(168);               	//初始化延时函数
	uart_init(115200);             	//初始化USART
	//bsp_Icm42688Init();
		
	IMU_init();
	
	delay_ms(100);
	while(1)
	{	
		IMU_getYawPitchRoll(ypr);
		IMU_TT_getgyro(motion6);
		printf("acc(mg):%.2f\t%.2f\t%.2f\tgyro(dps):%.2f\t%.2f\t%.2f", motion6[0], motion6[1], motion6[2], motion6[3], motion6[4], motion6[5]);
		printf("\tangle:%.2f\t%.2f\t%.2f\r\n", ypr[0], ypr[1], ypr[2]);
		math_pl++;
		delay_ms(20);
	}
}

