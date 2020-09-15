/**  
注意：涉及到串口通信的请将开发板上12M晶振换成11.0592MHZ。
*/

#include "public.h"  
#include "uart.h"
#include "tftlcd.h"
#include "touch.h"
#include "gui.h"
#include "stdlib.h"

sbit LED1=P2^6;

#define MULTIEDIT_START_X		10	  		//起始X坐标
#define MULTIEDIT_START_Y		20			//起始Y坐标
#define MULTIEDIT_WIDTH			150			//长
#define MULTIEDIT_HEIGHT		20			//高

#define MULTIEDIT1_START_X		MULTIEDIT_START_X	  		//起始X坐标
#define MULTIEDIT1_START_Y		MULTIEDIT_START_Y+MULTIEDIT_HEIGHT+1			//起始Y坐标

#define MULTIEDIT_RIM_LTOC 		0XA535	    //左上外线颜色
#define MULTIEDIT_RIM_LTIC 		0X8431		//左上内线颜色
#define MULTIEDIT_RIM_RBOC 		0XFFFF		//右下外线颜色
#define MULTIEDIT_RIM_RBIC 		0XDEFC		//右下内线颜色

#define MULTIEDIT_BACKCOLOR		WHITE  		//背景色
#define MULTIEDIT_FRONTCOLOR	BLUE   		//前景色
#define MULTIEDIT_FONTSIZE		16   		//文本框内字体大小

//多行文本框创建
//x0,y0:矩形的左上角坐标
//width,height:文本框的尺寸
//backcolor:背景颜色
void MultiEdit_Create(u16 x0,u16 y0,u16 width,u16 height,u16 backcolor)
{
	gui_draw_vline(x0,y0,height,MULTIEDIT_RIM_LTOC);			//左外线
	gui_draw_hline(x0,y0,width,MULTIEDIT_RIM_LTOC);			//上外线
	gui_draw_vline(x0+1,y0+1,height-2,MULTIEDIT_RIM_LTIC);	//左内线
	gui_draw_hline(x0+1,y0+1,width-2,MULTIEDIT_RIM_LTIC);	//上内线
	gui_draw_vline(x0+width-1,y0,height,MULTIEDIT_RIM_RBOC);		//右外线
	gui_draw_hline(x0,y0+height-1,width,MULTIEDIT_RIM_RBOC);		//下外线
	gui_draw_vline(x0+width-2,y0+1,height-2,MULTIEDIT_RIM_RBIC);	//右内线
	gui_draw_hline(x0+1,y0+height-2,width-2,MULTIEDIT_RIM_RBIC);	//下内线
	gui_fill_rectangle(x0+2,y0+2,width-3,height-3,backcolor);//填充内部	
}

//多行文本框显示字符串
//x0,y0:矩形的左上角坐标
//width,height:文本框的尺寸
//offset:开始显示的偏移
//backcolor:背景颜色
//frontcolor:前景颜色
//size:文字大小
//str:字符串
void MultiEdit_ShowString(u16 x0,u16 y0,u16 width,u16 height,u16 xoffset,u16 yoffset,u16 backcolor,u16 frontcolor,u8 size,u8 *str)
{
	static u16 color_temp=0;

	color_temp=BACK_COLOR;
	BACK_COLOR=backcolor;
	gui_show_stringex(x0+xoffset,y0+yoffset,width,height,frontcolor,size,str,0);
	BACK_COLOR=color_temp;	
}






#define BUTTON_START_X		MULTIEDIT_START_X	  		//起始X坐标
#define BUTTON_START_Y		MULTIEDIT1_START_Y+MULTIEDIT_HEIGHT+5			//起始Y坐标
#define BUTTON_WIDTH		30			//长
#define BUTTON_HEIGHT		30			//高
#define BUTTON_ARCSIZE		4  			//默认圆角的半径

#define BUTTON_SPACE_X		10  			//X间隔
#define BUTTON_SPACE_Y		10  			//Y间隔

//按钮颜色定义
#define BTN_DFU_BCFUC		0X0000		//默认松开的颜色
#define BTN_DFU_BCFDC		0X0000		//默认按下的颜色

#define ARC_BTN_RIMC		0X0000		//圆角按钮边框颜色
#define ARC_BTN_TP1C		0XD6BB		//第一行的颜色
#define ARC_BTN_UPHC		0X8452		//上半部分颜色
#define ARC_BTN_DNHC		0X52CD		//下半部分颜色

#define BUTTON_BACKCOLOR	0XEF5D  	//背景色
#define BUTTON_FRONTCOLOR	BLACK   	//前景色
#define BUTTON_FONTSIZE		16   		//字体大小


//按键触摸比较值
#define CONTROL_START_X		BUTTON_START_X
#define CONTROL_START_Y		BUTTON_START_Y
#define CONTROL_END_X		BUTTON_START_X+BUTTON_WIDTH
#define CONTROL_END_Y		BUTTON_START_Y+BUTTON_HEIGHT


//按钮创建
//x0,y0:矩形的左上角坐标
//width,height:文本框的尺寸
//backcolor:背景颜色
//void Button_Create(u16 x0,u16 y0,u16 width,u16 height,u8 r,u16 backcolor,u16 frontcolor,u8 size,u8*str)
void Button_Create(u16 x0,u16 y0,u16 width,u16 height,u8 r,u16 frontcolor,u8 size,u8*str)
{
	gui_draw_arcrectangle(x0,y0,width,height,r,1,ARC_BTN_UPHC,ARC_BTN_DNHC);//填充圆角按钮
	gui_draw_arcrectangle(x0,y0,width,height,r,0,ARC_BTN_RIMC,ARC_BTN_RIMC);//画圆角边框
	gui_draw_hline (x0+r,y0+1,width-2*r,ARC_BTN_TP1C);//第一行
	gui_show_strmid(x0+1,y0+1,width,height,frontcolor,size,str);
}

//按钮按下显示
//x0,y0:矩形的左上角坐标
//width,height:文本框的尺寸
//backcolor:背景颜色
//void Button_PressCreate(u16 x0,u16 y0,u16 width,u16 height,u8 r,u16 backcolor,u16 frontcolor,u8 size,u8*str)
void Button_PressCreate(u16 x0,u16 y0,u16 width,u16 height,u8 r,u16 frontcolor,u8 size,u8*str)
{	
	gui_draw_arcrectangle(x0,y0,width,height,r,1,WHITE,WHITE);//填充圆角按钮
	gui_draw_arcrectangle(x0,y0,width,height,r,0,GREEN,GREEN);//画圆角边框
	gui_draw_hline (x0+r,y0+1,width-2*r,ARC_BTN_TP1C);//第一行
	gui_show_strmid(x0+1,y0+1,width,height,frontcolor,size,str);
}




const u8 Button_text[][2]={"1","2","3","4","5"}; //按钮显示文本

u8 T_BUF;
//简易计算器测试
void Calculator_Test(void)
{
	u8 i=0;
	u8 j=0;
	u8 Button_PressFlag=0;
	u8 total_cnt=0;
	u8 dat1_cnt=0;
	u8 dat2_cnt=0;
	u8 fuhao_value=0;
	u8 fuhao_flag=0;
	int dat1_num=0;
	int dat2_num=0;
	int result=0;
	u8 pos;
	u8 row[]={0,2,1,0,2};
	u8 col[]={0,0,1,2,2};
	
	FRONT_COLOR=RED;
	LCD_ShowString(10,0,tftlcd_data.width,tftlcd_data.height,16,"Calculator Test");
  MultiEdit_Create(MULTIEDIT_START_X,MULTIEDIT_START_Y,MULTIEDIT_WIDTH,MULTIEDIT_HEIGHT,MULTIEDIT_BACKCOLOR); 
	MultiEdit_ShowString(MULTIEDIT_START_X,MULTIEDIT_START_Y,MULTIEDIT_WIDTH,MULTIEDIT_HEIGHT,5,2,MULTIEDIT_BACKCOLOR,MULTIEDIT_FRONTCOLOR,MULTIEDIT_FONTSIZE," ");
	MultiEdit_Create(MULTIEDIT1_START_X,MULTIEDIT1_START_Y,MULTIEDIT_WIDTH,MULTIEDIT_HEIGHT,MULTIEDIT_BACKCOLOR); 
	MultiEdit_ShowString(MULTIEDIT1_START_X,MULTIEDIT1_START_Y,MULTIEDIT_WIDTH,MULTIEDIT_HEIGHT,5,2,MULTIEDIT_BACKCOLOR,MULTIEDIT_FRONTCOLOR,MULTIEDIT_FONTSIZE,"0");
	
	for(i=0;i<4;i++)
	{
			//Button_Create(BUTTON_START_X+(BUTTON_SPACE_X+BUTTON_WIDTH)*col[i],BUTTON_START_Y+(BUTTON_SPACE_Y+BUTTON_HEIGHT)*row[i],BUTTON_WIDTH,BUTTON_HEIGHT,BUTTON_ARCSIZE,BUTTON_BACKCOLOR,BUTTON_FRONTCOLOR,24,Button_text[i]);	
	Button_Create(BUTTON_START_X+(BUTTON_SPACE_X+BUTTON_WIDTH)*col[i],BUTTON_START_Y+(BUTTON_SPACE_Y+BUTTON_HEIGHT)*row[i],BUTTON_WIDTH,BUTTON_HEIGHT,BUTTON_ARCSIZE,BUTTON_FRONTCOLOR,24,Button_text[i]);
	}
	
	while(1)
	{
		TOUCH_Scan();
		if(xpt_xy.sta)
		{
			for(i=0;i<4;i++)   //扫描按钮，获取对应的位置信息
			{
					if((xpt_xy.lcdx>=BUTTON_START_X+(BUTTON_SPACE_X+BUTTON_WIDTH)*col[i]) && 
						(xpt_xy.lcdx<=BUTTON_START_X+BUTTON_WIDTH+(BUTTON_SPACE_X+BUTTON_WIDTH)*col[i])
						 && (xpt_xy.lcdy>=BUTTON_START_Y+(BUTTON_SPACE_Y+BUTTON_HEIGHT)*row[i]) && 
						(xpt_xy.lcdy<=BUTTON_START_Y+BUTTON_HEIGHT+(BUTTON_SPACE_Y+BUTTON_HEIGHT)*row[i]))
					{
						pos=i;
						T_BUF=i+1;
						Button_PressFlag=1;			
					}	
			}
		}
		else
		{
			if(Button_PressFlag)
			{
				Button_PressFlag=0;
				//Button_Create(BUTTON_START_X+(BUTTON_SPACE_X+BUTTON_WIDTH)*col[pos],BUTTON_START_Y+(BUTTON_SPACE_Y+BUTTON_HEIGHT)*row[pos],BUTTON_WIDTH,BUTTON_HEIGHT,BUTTON_ARCSIZE,BUTTON_BACKCOLOR,BUTTON_FRONTCOLOR,24,Button_text[pos]);
			Button_Create(BUTTON_START_X+(BUTTON_SPACE_X+BUTTON_WIDTH)*col[pos],BUTTON_START_Y+(BUTTON_SPACE_Y+BUTTON_HEIGHT)*row[pos],BUTTON_WIDTH,BUTTON_HEIGHT,BUTTON_ARCSIZE,BUTTON_FRONTCOLOR,24,Button_text[pos]);
			}
		}
		if(Button_PressFlag)//发送数据
		{
			//Button_PressCreate(BUTTON_START_X+(BUTTON_SPACE_X+BUTTON_WIDTH)*col[pos],BUTTON_START_Y+(BUTTON_SPACE_Y+BUTTON_HEIGHT)*row[pos],BUTTON_WIDTH,BUTTON_HEIGHT,BUTTON_ARCSIZE,BUTTON_BACKCOLOR,BUTTON_FRONTCOLOR,24,Button_text[pos]);			
		Button_PressCreate(BUTTON_START_X+(BUTTON_SPACE_X+BUTTON_WIDTH)*col[pos],BUTTON_START_Y+(BUTTON_SPACE_Y+BUTTON_HEIGHT)*row[pos],BUTTON_WIDTH,BUTTON_HEIGHT,BUTTON_ARCSIZE,BUTTON_FRONTCOLOR,24,Button_text[pos]);
		}

		i++;
		if(i%20==0)
		LED1=!LED1;
		delay_ms(10);	
	}	
}	



/*******************************************************************************
注意事项：
将NRF24L01插入对应插座

实验操作：
P2^1 --- J19 D1
P3^1 --- JP1 K1
P3^2 --- JP1 K2
CE   --- P1^2;
CSN  --- P1^3;
SCK  --- P1^7;
MOSI --- P1^1;
MISO --- P1^6;
IRQ  --- P1^4;
*******************************************************************************

/***************************************************/
#define TX_ADR_WIDTH   5  // 5个字节宽度的发送/接收地址	
#define TX_PLOAD_WIDTH 4  // 数据通道有效数据宽度

//记得引脚冲突检查
sbit LED =  P2^1;

u8 code TX_ADDRESS[TX_ADR_WIDTH] = {0x34,0xaa,0x00,0x00,0x03};  // 定义静态发送地址
u8 RX_BUF[TX_PLOAD_WIDTH];
u8 TX_BUF[TX_PLOAD_WIDTH];
u8 flag;
u8 DATA = 0x01;
u8 bdata sta;
sbit  RX_DR     = sta^6;
sbit  TX_DS     = sta^5;
sbit  MAX_RT = sta^4;

sbit CE =  P1^2;
sbit CSN=  P1^3;
sbit SCK=  P1^7;
sbit MOSI= P1^1;
sbit MISO= P1^6;
sbit IRQ = P1^4;

// SPI(nRF24L01) commands
#define READ_REG    0x00  // Define read command to register
#define WRITE_REG   0x20  // Define write command to register
#define RD_RX_PLOAD 0x61  // Define RX payload register address
#define WR_TX_PLOAD 0xA0  // Define TX payload register address
#define FLUSH_TX    0xE1  // Define flush TX register command
#define FLUSH_RX    0xE2  // Define flush RX register command
#define REUSE_TX_PL 0xE3  // Define reuse TX payload register command
#define NOP         0xFF  // Define No Operation, might be used to read status register

// SPI(nRF24L01) registers(addresses)
#define CONFIG      0x00  // 'Config' register address
#define EN_AA       0x01  // 'Enable Auto Acknowledgment' register address
#define EN_RXADDR   0x02  // 'Enabled RX addresses' register address
#define SETUP_AW    0x03  // 'Setup address width' register address
#define SETUP_RETR  0x04  // 'Setup Auto. Retrans' register address
#define RF_CH       0x05  // 'RF channel' register address
#define RF_SETUP    0x06  // 'RF setup' register address
#define STATUS      0x07  // 'Status' register address
#define OBSERVE_TX  0x08  // 'Observe TX' register address
#define CD          0x09  // 'Carrier Detect' register address
#define RX_ADDR_P0  0x0A  // 'RX address pipe0' register address
#define RX_ADDR_P1  0x0B  // 'RX address pipe1' register address
#define RX_ADDR_P2  0x0C  // 'RX address pipe2' register address
#define RX_ADDR_P3  0x0D  // 'RX address pipe3' register address
#define RX_ADDR_P4  0x0E  // 'RX address pipe4' register address
#define RX_ADDR_P5  0x0F  // 'RX address pipe5' register address
#define TX_ADDR     0x10  // 'TX address' register address
#define RX_PW_P0    0x11  // 'RX payload width, pipe0' register address
#define RX_PW_P1    0x12  // 'RX payload width, pipe1' register address
#define RX_PW_P2    0x13  // 'RX payload width, pipe2' register address
#define RX_PW_P3    0x14  // 'RX payload width, pipe3' register address
#define RX_PW_P4    0x15  // 'RX payload width, pipe4' register address
#define RX_PW_P5    0x16  // 'RX payload width, pipe5' register address
#define FIFO_STATUS 0x17  // 'FIFO Status Register' register address

void blink(char i);

/**************************************************
*函数名：  init_io
*函数功能：初始化IO
*输入：    无
*输出：    无
/**************************************************/
void init_io(void)
{
    CE  = 0;        // 关闭使能
    CSN = 1;        // SPI禁止
    SCK = 0;        // SPI时钟置低
    IRQ = 1;        // 中断复位
    LED = 1;        // 关闭指示灯
}
/**************************************************/

/**************************************************
*函数名：                              SPI_RW
*函数功能：                            读写一个字节
*输入：                                无
*输出：                                无
/**************************************************/
u8 SPI_RW(u8 byte)
{
	u8 i;
	for(i=0; i<8; i++)          // 循环8次
	{
	   MOSI = (byte & 0x80);   // byte最高位输出到MOSI
	   byte <<= 1;             // 低一位移位到最高位
	   SCK = 1;                // 拉高SCK,nRF24L01从MOSI读取1位数据,同时从MISO输出1位数据
	   byte |= MISO;           // 读MISO到byte最低位
	   SCK = 0;                // SCK置低
	}
	return(byte);               // 返回读取一个字节
}
/**************************************************/

/**************************************************
*函数名：  SPI_RW_Reg
*函数功能：写数据到reg
*输入：    无
*输出：    无
/**************************************************/
u8 SPI_RW_Reg(u8 reg, u8 value)
{
	u8 status;
	CSN = 0;                   // CSN置低，开始传输数据
	status = SPI_RW(reg);      // 选择寄存器，同时返回状态字
	SPI_RW(value);             // 写数据到寄存器
	CSN = 1;                   // CSN拉高，结束数据传输
	return(status);            // 返回状态寄存器
}
/**************************************************/

/**************************************************
*函数名：  SPI_Read
*函数功能：从reg寄存器读字节
*输入：    无
*输出：    无
/**************************************************/
u8 SPI_Read(u8 reg)
{
	u8 reg_val;
	CSN = 0;                    // CSN置低,开始传输数据
	SPI_RW(reg);                // 选择寄存器
	reg_val = SPI_RW(0);        // 然后从该寄存器读数据
	CSN = 1;                    // CSN拉高,结束数据传输
	return(reg_val);            // 返回寄存器数据
}
/**************************************************/

/**************************************************
*函数名：  SPI_Read_Buf
*函数功能：从reg寄存器读数据
*输入：    无
*输出：    无
/**************************************************/
u8 SPI_Read_Buf(u8 reg, u8 * pBuf, u8 bytes)
{
	u8 status, i;
	CSN = 0;                    // CSN置低，开始传输数据
	status = SPI_RW(reg);       // 选择寄存器，同时返回状态字
	for(i=0; i<bytes; i++)
		pBuf[i] = SPI_RW(0);    // 逐个字节从nRF24L01读出
	CSN = 1;                    // CSN拉高，结束数据传输
	return(status);             // 返回状态寄存器
}
/**************************************************/

/**************************************************
*函数名：  SPI_Write_Buf
*函数功能：把缓存的数据写入NRF
*输入：    无
*输出：    无
/**************************************************/
u8 SPI_Write_Buf(u8 reg, u8 * pBuf, u8 bytes)
{
	u8 status, i;
	CSN = 0;                    // CSN置低，开始传输数据
	status = SPI_RW(reg);       // 选择寄存器，同时返回状态字
	for(i=0; i<bytes; i++)
		SPI_RW(pBuf[i]);        // 逐个字节写入nRF24L01
	CSN = 1;                    // CSN拉高,结束数据传输
	return(status);             // 返回状态寄存器
}
/**************************************************/

/**************************************************
*函数名：   Check_ACK
*函数功能： 检查接收设备有无数据包，设定没有应答信号重发
*输入：     无
*输出：     无
/**************************************************/
u8 Check_ACK(bit clear)
{
    delay_ms(200);
    while(IRQ);
    sta = SPI_RW(NOP);                    // 返回状态寄存器
    if(TX_DS)
    {
        blink(3);
    }
    if(MAX_RT)
        if(clear)                         // 是否清除TX FIFO，没有清除在复位MAX_RT中断标志后重发
            SPI_RW(FLUSH_TX);
    SPI_RW_Reg(WRITE_REG + STATUS, sta);  // 清除TX_DS或MAX_RT中断标志
    IRQ = 1;
    if(TX_DS)
        return(0x00);
    else
        return(0xff);
}
/**************************************************/

/**************************************************
*函数名：   RX_Mode
*函数功能： 将nrf设置为接收模式
*输入：     无
*输出：     无
/**************************************************/
void RX_Mode(void)
{
	CE = 0;
	SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // 接收设备接收通道0使用和发送设备相同的发送地址
	SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);               // 使能接收通道0自动应答
	SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);           // 使能接收通道0
	SPI_RW_Reg(WRITE_REG + RF_CH, 40);                 // 选择射频通道0x40
	SPI_RW_Reg(WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH);  // 接收通道0选择和发送通道相同有效数据宽度
	SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x07);            // 数据传输率1Mbps,发射功率0dBm,低噪声放大器增益
	SPI_RW_Reg(WRITE_REG + CONFIG, 0x0f);              // CRC使能，16位CRC校验，上电，接收模式
	delay_ms(150);
	CE = 1;                                            // 拉高CE启动接收设备
}
/**************************************************/

/**************************************************
*函数名：  TX_Mode
*函数功能：将nrf设置为发送模式
*输入：    无
*输出：    无
/**************************************************/
void TX_Mode(u8 * BUF)
{
    CE = 0;
	SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);     // 写入发送地址
	SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // 应答接收设备，接收通道0地址和发送地址相同
	SPI_Write_Buf(WR_TX_PLOAD, BUF, TX_PLOAD_WIDTH);                  // 写数据包到TX FIFO
	SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);       // 使能接收通道0自动应答
	SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);   // 使能接收通道0
	SPI_RW_Reg(WRITE_REG + SETUP_RETR, 0x0a);  // 自动重发延时等待250us+86us，自动重发
	SPI_RW_Reg(WRITE_REG + RF_CH, 40);         // 选择射频通道0x40
	SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x07);    // 数据传输率1Mbps，发射功率0dBm，低噪声放大器增益
	SPI_RW_Reg(WRITE_REG + CONFIG, 0x0e);      // CRC使能，16位CRC校验，上电
    delay_ms(150);
    CE = 1;
}
/**************************************************/

/**************************************************
*函数名：                              NRFSendMessage
*函数功能：                            无线发送数据
*输入：                                无
*输出：                                无
/**************************************************/
void NRFSendMessage(u8 * BUF)
{
	TX_Mode(BUF);
	Check_ACK(0);           // 等待发送完毕，清除TX FIFO
    SPI_RW_Reg(FLUSH_TX,0x00); //清除发送缓存
	delay_ms(250);
    delay_ms(250);
}
/**************************************************/

void blink(char i)
{
    while(i--)
    {
        LED = 1;
        delay_ms(500);
        LED = 0;
        delay_ms(500);
    }

}


/**************************************************
*函数名：  CheckButtons
*函数功能：检查按键是否按下，按下发送一字节数据
*输入：    无
*输出：    无
/**************************************************/
void CheckButtons()
{
        TX_BUF[0]=T_BUF;
			  NRFSendMessage(TX_BUF);        // 将nrf设置为发送模式并发送数据           		 
}
/**************************************************
*函数名：                              main
*函数功能：                            主函数
*输入：                                无
*输出：                                无
/**************************************************/

void main()
{	

	UART_Init();
	TFTLCD_Init();
  init_io();                      // 初始化IO
	Calculator_Test();
	CheckButtons();           // 按键扫描	
	while(1)
	{
		
	}		
}
