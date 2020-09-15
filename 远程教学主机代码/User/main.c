/**  
ע�⣺�漰������ͨ�ŵ��뽫��������12M���񻻳�11.0592MHZ��
*/

#include "public.h"  
#include "uart.h"
#include "tftlcd.h"
#include "touch.h"
#include "gui.h"
#include "stdlib.h"

sbit LED1=P2^6;

#define MULTIEDIT_START_X		10	  		//��ʼX����
#define MULTIEDIT_START_Y		20			//��ʼY����
#define MULTIEDIT_WIDTH			150			//��
#define MULTIEDIT_HEIGHT		20			//��

#define MULTIEDIT1_START_X		MULTIEDIT_START_X	  		//��ʼX����
#define MULTIEDIT1_START_Y		MULTIEDIT_START_Y+MULTIEDIT_HEIGHT+1			//��ʼY����

#define MULTIEDIT_RIM_LTOC 		0XA535	    //����������ɫ
#define MULTIEDIT_RIM_LTIC 		0X8431		//����������ɫ
#define MULTIEDIT_RIM_RBOC 		0XFFFF		//����������ɫ
#define MULTIEDIT_RIM_RBIC 		0XDEFC		//����������ɫ

#define MULTIEDIT_BACKCOLOR		WHITE  		//����ɫ
#define MULTIEDIT_FRONTCOLOR	BLUE   		//ǰ��ɫ
#define MULTIEDIT_FONTSIZE		16   		//�ı����������С

//�����ı��򴴽�
//x0,y0:���ε����Ͻ�����
//width,height:�ı���ĳߴ�
//backcolor:������ɫ
void MultiEdit_Create(u16 x0,u16 y0,u16 width,u16 height,u16 backcolor)
{
	gui_draw_vline(x0,y0,height,MULTIEDIT_RIM_LTOC);			//������
	gui_draw_hline(x0,y0,width,MULTIEDIT_RIM_LTOC);			//������
	gui_draw_vline(x0+1,y0+1,height-2,MULTIEDIT_RIM_LTIC);	//������
	gui_draw_hline(x0+1,y0+1,width-2,MULTIEDIT_RIM_LTIC);	//������
	gui_draw_vline(x0+width-1,y0,height,MULTIEDIT_RIM_RBOC);		//������
	gui_draw_hline(x0,y0+height-1,width,MULTIEDIT_RIM_RBOC);		//������
	gui_draw_vline(x0+width-2,y0+1,height-2,MULTIEDIT_RIM_RBIC);	//������
	gui_draw_hline(x0+1,y0+height-2,width-2,MULTIEDIT_RIM_RBIC);	//������
	gui_fill_rectangle(x0+2,y0+2,width-3,height-3,backcolor);//����ڲ�	
}

//�����ı�����ʾ�ַ���
//x0,y0:���ε����Ͻ�����
//width,height:�ı���ĳߴ�
//offset:��ʼ��ʾ��ƫ��
//backcolor:������ɫ
//frontcolor:ǰ����ɫ
//size:���ִ�С
//str:�ַ���
void MultiEdit_ShowString(u16 x0,u16 y0,u16 width,u16 height,u16 xoffset,u16 yoffset,u16 backcolor,u16 frontcolor,u8 size,u8 *str)
{
	static u16 color_temp=0;

	color_temp=BACK_COLOR;
	BACK_COLOR=backcolor;
	gui_show_stringex(x0+xoffset,y0+yoffset,width,height,frontcolor,size,str,0);
	BACK_COLOR=color_temp;	
}






#define BUTTON_START_X		MULTIEDIT_START_X	  		//��ʼX����
#define BUTTON_START_Y		MULTIEDIT1_START_Y+MULTIEDIT_HEIGHT+5			//��ʼY����
#define BUTTON_WIDTH		30			//��
#define BUTTON_HEIGHT		30			//��
#define BUTTON_ARCSIZE		4  			//Ĭ��Բ�ǵİ뾶

#define BUTTON_SPACE_X		10  			//X���
#define BUTTON_SPACE_Y		10  			//Y���

//��ť��ɫ����
#define BTN_DFU_BCFUC		0X0000		//Ĭ���ɿ�����ɫ
#define BTN_DFU_BCFDC		0X0000		//Ĭ�ϰ��µ���ɫ

#define ARC_BTN_RIMC		0X0000		//Բ�ǰ�ť�߿���ɫ
#define ARC_BTN_TP1C		0XD6BB		//��һ�е���ɫ
#define ARC_BTN_UPHC		0X8452		//�ϰ벿����ɫ
#define ARC_BTN_DNHC		0X52CD		//�°벿����ɫ

#define BUTTON_BACKCOLOR	0XEF5D  	//����ɫ
#define BUTTON_FRONTCOLOR	BLACK   	//ǰ��ɫ
#define BUTTON_FONTSIZE		16   		//�����С


//���������Ƚ�ֵ
#define CONTROL_START_X		BUTTON_START_X
#define CONTROL_START_Y		BUTTON_START_Y
#define CONTROL_END_X		BUTTON_START_X+BUTTON_WIDTH
#define CONTROL_END_Y		BUTTON_START_Y+BUTTON_HEIGHT


//��ť����
//x0,y0:���ε����Ͻ�����
//width,height:�ı���ĳߴ�
//backcolor:������ɫ
//void Button_Create(u16 x0,u16 y0,u16 width,u16 height,u8 r,u16 backcolor,u16 frontcolor,u8 size,u8*str)
void Button_Create(u16 x0,u16 y0,u16 width,u16 height,u8 r,u16 frontcolor,u8 size,u8*str)
{
	gui_draw_arcrectangle(x0,y0,width,height,r,1,ARC_BTN_UPHC,ARC_BTN_DNHC);//���Բ�ǰ�ť
	gui_draw_arcrectangle(x0,y0,width,height,r,0,ARC_BTN_RIMC,ARC_BTN_RIMC);//��Բ�Ǳ߿�
	gui_draw_hline (x0+r,y0+1,width-2*r,ARC_BTN_TP1C);//��һ��
	gui_show_strmid(x0+1,y0+1,width,height,frontcolor,size,str);
}

//��ť������ʾ
//x0,y0:���ε����Ͻ�����
//width,height:�ı���ĳߴ�
//backcolor:������ɫ
//void Button_PressCreate(u16 x0,u16 y0,u16 width,u16 height,u8 r,u16 backcolor,u16 frontcolor,u8 size,u8*str)
void Button_PressCreate(u16 x0,u16 y0,u16 width,u16 height,u8 r,u16 frontcolor,u8 size,u8*str)
{	
	gui_draw_arcrectangle(x0,y0,width,height,r,1,WHITE,WHITE);//���Բ�ǰ�ť
	gui_draw_arcrectangle(x0,y0,width,height,r,0,GREEN,GREEN);//��Բ�Ǳ߿�
	gui_draw_hline (x0+r,y0+1,width-2*r,ARC_BTN_TP1C);//��һ��
	gui_show_strmid(x0+1,y0+1,width,height,frontcolor,size,str);
}




const u8 Button_text[][2]={"1","2","3","4","5"}; //��ť��ʾ�ı�

u8 T_BUF;
//���׼���������
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
			for(i=0;i<4;i++)   //ɨ�谴ť����ȡ��Ӧ��λ����Ϣ
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
		if(Button_PressFlag)//��������
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
ע�����
��NRF24L01�����Ӧ����

ʵ�������
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
#define TX_ADR_WIDTH   5  // 5���ֽڿ�ȵķ���/���յ�ַ	
#define TX_PLOAD_WIDTH 4  // ����ͨ����Ч���ݿ��

//�ǵ����ų�ͻ���
sbit LED =  P2^1;

u8 code TX_ADDRESS[TX_ADR_WIDTH] = {0x34,0xaa,0x00,0x00,0x03};  // ���徲̬���͵�ַ
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
*��������  init_io
*�������ܣ���ʼ��IO
*���룺    ��
*�����    ��
/**************************************************/
void init_io(void)
{
    CE  = 0;        // �ر�ʹ��
    CSN = 1;        // SPI��ֹ
    SCK = 0;        // SPIʱ���õ�
    IRQ = 1;        // �жϸ�λ
    LED = 1;        // �ر�ָʾ��
}
/**************************************************/

/**************************************************
*��������                              SPI_RW
*�������ܣ�                            ��дһ���ֽ�
*���룺                                ��
*�����                                ��
/**************************************************/
u8 SPI_RW(u8 byte)
{
	u8 i;
	for(i=0; i<8; i++)          // ѭ��8��
	{
	   MOSI = (byte & 0x80);   // byte���λ�����MOSI
	   byte <<= 1;             // ��һλ��λ�����λ
	   SCK = 1;                // ����SCK,nRF24L01��MOSI��ȡ1λ����,ͬʱ��MISO���1λ����
	   byte |= MISO;           // ��MISO��byte���λ
	   SCK = 0;                // SCK�õ�
	}
	return(byte);               // ���ض�ȡһ���ֽ�
}
/**************************************************/

/**************************************************
*��������  SPI_RW_Reg
*�������ܣ�д���ݵ�reg
*���룺    ��
*�����    ��
/**************************************************/
u8 SPI_RW_Reg(u8 reg, u8 value)
{
	u8 status;
	CSN = 0;                   // CSN�õͣ���ʼ��������
	status = SPI_RW(reg);      // ѡ��Ĵ�����ͬʱ����״̬��
	SPI_RW(value);             // д���ݵ��Ĵ���
	CSN = 1;                   // CSN���ߣ��������ݴ���
	return(status);            // ����״̬�Ĵ���
}
/**************************************************/

/**************************************************
*��������  SPI_Read
*�������ܣ���reg�Ĵ������ֽ�
*���룺    ��
*�����    ��
/**************************************************/
u8 SPI_Read(u8 reg)
{
	u8 reg_val;
	CSN = 0;                    // CSN�õ�,��ʼ��������
	SPI_RW(reg);                // ѡ��Ĵ���
	reg_val = SPI_RW(0);        // Ȼ��ӸüĴ���������
	CSN = 1;                    // CSN����,�������ݴ���
	return(reg_val);            // ���ؼĴ�������
}
/**************************************************/

/**************************************************
*��������  SPI_Read_Buf
*�������ܣ���reg�Ĵ���������
*���룺    ��
*�����    ��
/**************************************************/
u8 SPI_Read_Buf(u8 reg, u8 * pBuf, u8 bytes)
{
	u8 status, i;
	CSN = 0;                    // CSN�õͣ���ʼ��������
	status = SPI_RW(reg);       // ѡ��Ĵ�����ͬʱ����״̬��
	for(i=0; i<bytes; i++)
		pBuf[i] = SPI_RW(0);    // ����ֽڴ�nRF24L01����
	CSN = 1;                    // CSN���ߣ��������ݴ���
	return(status);             // ����״̬�Ĵ���
}
/**************************************************/

/**************************************************
*��������  SPI_Write_Buf
*�������ܣ��ѻ��������д��NRF
*���룺    ��
*�����    ��
/**************************************************/
u8 SPI_Write_Buf(u8 reg, u8 * pBuf, u8 bytes)
{
	u8 status, i;
	CSN = 0;                    // CSN�õͣ���ʼ��������
	status = SPI_RW(reg);       // ѡ��Ĵ�����ͬʱ����״̬��
	for(i=0; i<bytes; i++)
		SPI_RW(pBuf[i]);        // ����ֽ�д��nRF24L01
	CSN = 1;                    // CSN����,�������ݴ���
	return(status);             // ����״̬�Ĵ���
}
/**************************************************/

/**************************************************
*��������   Check_ACK
*�������ܣ� �������豸�������ݰ����趨û��Ӧ���ź��ط�
*���룺     ��
*�����     ��
/**************************************************/
u8 Check_ACK(bit clear)
{
    delay_ms(200);
    while(IRQ);
    sta = SPI_RW(NOP);                    // ����״̬�Ĵ���
    if(TX_DS)
    {
        blink(3);
    }
    if(MAX_RT)
        if(clear)                         // �Ƿ����TX FIFO��û������ڸ�λMAX_RT�жϱ�־���ط�
            SPI_RW(FLUSH_TX);
    SPI_RW_Reg(WRITE_REG + STATUS, sta);  // ���TX_DS��MAX_RT�жϱ�־
    IRQ = 1;
    if(TX_DS)
        return(0x00);
    else
        return(0xff);
}
/**************************************************/

/**************************************************
*��������   RX_Mode
*�������ܣ� ��nrf����Ϊ����ģʽ
*���룺     ��
*�����     ��
/**************************************************/
void RX_Mode(void)
{
	CE = 0;
	SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // �����豸����ͨ��0ʹ�úͷ����豸��ͬ�ķ��͵�ַ
	SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);               // ʹ�ܽ���ͨ��0�Զ�Ӧ��
	SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);           // ʹ�ܽ���ͨ��0
	SPI_RW_Reg(WRITE_REG + RF_CH, 40);                 // ѡ����Ƶͨ��0x40
	SPI_RW_Reg(WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH);  // ����ͨ��0ѡ��ͷ���ͨ����ͬ��Ч���ݿ��
	SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x07);            // ���ݴ�����1Mbps,���书��0dBm,�������Ŵ�������
	SPI_RW_Reg(WRITE_REG + CONFIG, 0x0f);              // CRCʹ�ܣ�16λCRCУ�飬�ϵ磬����ģʽ
	delay_ms(150);
	CE = 1;                                            // ����CE���������豸
}
/**************************************************/

/**************************************************
*��������  TX_Mode
*�������ܣ���nrf����Ϊ����ģʽ
*���룺    ��
*�����    ��
/**************************************************/
void TX_Mode(u8 * BUF)
{
    CE = 0;
	SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);     // д�뷢�͵�ַ
	SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // Ӧ������豸������ͨ��0��ַ�ͷ��͵�ַ��ͬ
	SPI_Write_Buf(WR_TX_PLOAD, BUF, TX_PLOAD_WIDTH);                  // д���ݰ���TX FIFO
	SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);       // ʹ�ܽ���ͨ��0�Զ�Ӧ��
	SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);   // ʹ�ܽ���ͨ��0
	SPI_RW_Reg(WRITE_REG + SETUP_RETR, 0x0a);  // �Զ��ط���ʱ�ȴ�250us+86us���Զ��ط�
	SPI_RW_Reg(WRITE_REG + RF_CH, 40);         // ѡ����Ƶͨ��0x40
	SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x07);    // ���ݴ�����1Mbps�����书��0dBm���������Ŵ�������
	SPI_RW_Reg(WRITE_REG + CONFIG, 0x0e);      // CRCʹ�ܣ�16λCRCУ�飬�ϵ�
    delay_ms(150);
    CE = 1;
}
/**************************************************/

/**************************************************
*��������                              NRFSendMessage
*�������ܣ�                            ���߷�������
*���룺                                ��
*�����                                ��
/**************************************************/
void NRFSendMessage(u8 * BUF)
{
	TX_Mode(BUF);
	Check_ACK(0);           // �ȴ�������ϣ����TX FIFO
    SPI_RW_Reg(FLUSH_TX,0x00); //������ͻ���
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
*��������  CheckButtons
*�������ܣ���鰴���Ƿ��£����·���һ�ֽ�����
*���룺    ��
*�����    ��
/**************************************************/
void CheckButtons()
{
        TX_BUF[0]=T_BUF;
			  NRFSendMessage(TX_BUF);        // ��nrf����Ϊ����ģʽ����������           		 
}
/**************************************************
*��������                              main
*�������ܣ�                            ������
*���룺                                ��
*�����                                ��
/**************************************************/

void main()
{	

	UART_Init();
	TFTLCD_Init();
  init_io();                      // ��ʼ��IO
	Calculator_Test();
	CheckButtons();           // ����ɨ��	
	while(1)
	{
		
	}		
}
