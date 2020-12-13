
#include "QMC5883L.h"
#include "compute.h"

#define  QMC5883_Buf_Size 5

float x_mag,y_mag,z_mag;
	
int8_t  QMC5883_Buf_index = 0;	  
int16_t  QMC5883_FIFO[3][6];      //磁力计滤波

/**************************实现函数********************************************
*函数原型:	   void HMC58X3_FIFO_init(void)
*功　　能:	   连续读取100次数据，以初始化FIFO数组
输入参数：  无
输出参数：  无
*******************************************************************************/
void QMC58X3_FIFO_init(void)
{
   int16_t temp[3];
   unsigned char i;
   for(i=0; i<QMC5883_Buf_Size; i++)
   {
 	   QMC58X3_getRaw(&temp[0],&temp[1],&temp[2]);
  	   delay_us(50);  //延时再读取数据
   }
}

/**************************实现函数********************************************
*函数原型:	   void  HMC58X3_newValues(int16_t x,int16_t y,int16_t z)
*功　　能:	   更新一组数据到FIFO数组
输入参数：  磁力计三个轴对应的ADC值
输出参数：  无
*******************************************************************************/
void  QMC58X3_newValues(int16_t x,int16_t y,int16_t z)
{
	unsigned char i ;
	int32_t sum=0;

	QMC5883_FIFO[0][QMC5883_Buf_index] = x;
	QMC5883_FIFO[1][QMC5883_Buf_index] = y;
	QMC5883_FIFO[2][QMC5883_Buf_index] = z;
	QMC5883_Buf_index = (QMC5883_Buf_index + 1) % QMC5883_Buf_Size;
					  
	sum=0;
	for(i=0; i<QMC5883_Buf_Size; i++)
	{	//取数组内的值进行求和再取平均
   	sum += QMC5883_FIFO[0][i];
	}
	QMC5883_FIFO[0][QMC5883_Buf_Size] = sum/QMC5883_Buf_Size;	//将平均值更新

	sum=0;
	for(i=0; i<QMC5883_Buf_Size; i++)
	{
   	sum += QMC5883_FIFO[1][i];
	}
	QMC5883_FIFO[1][QMC5883_Buf_Size] = sum/QMC5883_Buf_Size;

	sum=0;
	for(i=0; i<QMC5883_Buf_Size; i++)
	{
   	sum += QMC5883_FIFO[2][i];
	}
	QMC5883_FIFO[2][QMC5883_Buf_Size] = sum/QMC5883_Buf_Size;
} //HMC58X3_newValues


/**************************实现函数********************************************
*函数原型:	  void HMC58X3_getRaw(int16_t *x,int16_t *y,int16_t *z)
*功　　能:	   写HMC5883L的寄存器
输入参数：    reg  寄存器地址
			  val   要写入的值	
输出参数：  无
*******************************************************************************/
void QMC58X3_getRaw(int16_t *x, int16_t *y, int16_t *z) 
{
   unsigned char vbuff[6];
   vbuff[0]=vbuff[1]=vbuff[2]=vbuff[3]=vbuff[4]=vbuff[5]=0;
   IICreadBytes(QMC58X3_ADDR,X_OUTPUT_LSB,6,vbuff);
   QMC58X3_newValues(((int16_t)vbuff[1] << 8) | vbuff[0],((int16_t)vbuff[3] << 8) | vbuff[2],((int16_t)vbuff[5] << 8) | vbuff[4]);
   *x = QMC5883_FIFO[0][QMC5883_Buf_Size];
   *y = QMC5883_FIFO[1][QMC5883_Buf_Size];
   *z = QMC5883_FIFO[2][QMC5883_Buf_Size];
}


/**************************实现函数********************************************
*函数原型:	  void HMC5883L_SetUp(void)
*功　　能:	   初始化 HMC5883L 使之进入可用状态
输入参数：     	
输出参数：  无
*******************************************************************************/
void QMC5883L_SetUp(void)
{      
   unsigned char qmc_config = 0x00;  
   IICwriteBytes(QMC58X3_ADDR, QMC5883_RESET_ADDR, 1, &qmc_config);
   
   qmc_config = 0x45;
   IICwriteBytes(QMC58X3_ADDR, QMC5883_CONTROL_ADDR, 1, &qmc_config);
   delay_ms(10);
   IICwriteBytes(QMC58X3_ADDR, QMC5883_CONTROL_ADDR, 1, &qmc_config);
   delay_ms(10);

   QMC58X3_FIFO_init();
}

/**************************实现函数********************************************
*函数原型:	  void Read_5883(void)
*功　　能:	  磁数据采集
*******************************************************************************/
void Read_5883(void)
{  	
   int16_t xr,yr,zr;
   QMC58X3_getRaw(&xr, &yr, &zr);
   
	x_mag = -(float)yr / 3.0f;
	y_mag = -(float)xr / 3.0f;
	z_mag = (float)zr / 3.0f;
} 


