#include "delay.h"
#include "sys.h"
#include "usart.h" 
#include "myiic.h"
#include "IMU.h"
#include "spi.h"
// 模式选择需要在basic_read_reg.c里选择宏定义ICM_USE_HARD_SPI/ICM_USE_I2C

// VCC--------5V或者3.3V都可以
//SPI 模式接线
// PA2------------------------CS
// PB13------------------------SCLK
// PB14------------------------MISO
// PB15------------------------MOSI

//IIC 模式接线
// PB6------------------------SCL
// PB7------------------------SDA
// AD0默认上拉可以不接
float motion6[7];
float ypr[3];          // 上传yaw pitch roll的值
int math_pl=0;

int main(void)
{
	SystemInit();
	delay_init();	    	 //延时函数初始化	  
	uart_init(115200);	 	//串口初始化为115200
	printf("OK\r\n");
	IIC_Init();
	SPI2_Init();
	//SPI2_SetSpeed(SPI_BaudRatePrescaler_16);
	//delay_ms(100);
	
	//load_config();
	//delay_ms(50);
	IMU_init();
	delay_ms(100);
	while(1)
	{	
		IMU_getYawPitchRoll(ypr);
		IMU_TT_getgyro(motion6);
		printf("%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\r\n", motion6[0], motion6[1], motion6[2], motion6[3], motion6[4], motion6[5]);
		math_pl++;
		delay_ms(10);
	}
}



