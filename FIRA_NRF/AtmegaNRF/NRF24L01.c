#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "NRF24L01.h"

// SPI(nRF24L01) commands
#define READ_REG_NRF24L01    	0x00				// Define read command to register
#define WRITE_REG_NRF24L01   	0x20 				// Define write command to register
#define RD_RX_PLOAD 			0x61 				// Define RX payload register address
#define WR_TX_PLOAD 			0xA0 				// Define TX payload register address
#define FLUSH_TX    			0xE1 				// Define flush TX register command
#define FLUSH_RX    			0xE2 				// Define flush RX register command
#define REUSE_TX_PL 			0xE3 				// Define reuse TX payload register command
#define NOP         			0xFF 				// Define No Operation, might be used to read status register
//***************************************************//
// SPI(nRF24L01) registers(addresses)
#define CONFIG      			0x00				// 'Config' register address
#define EN_AA       			0x01                // 'Enable Auto Acknowledgment' register address
#define EN_RXADDR   			0x02                // 'Enabled RX addresses' register address
#define SETUP_AW    			0x03                // 'Setup address width' register address
#define SETUP_RETR  			0x04                // 'Setup Auto. Retrans' register address
#define RF_CH       			0x05                // 'RF channel' register address
#define RF_SETUP    			0x06 				// 'RF setup' register address
#define STATUS      			0x07 				// 'Status' register address
#define OBSERVE_TX  			0x08 				// 'Observe TX' register address
#define CD          			0x09 				//'Carrier Detect' register address
#define RX_ADDR_P0  			0x0A				// 'RX address pipe0' register address
#define RX_ADDR_P1  			0x0B 				// 'RX address pipe1' register address
#define RX_ADDR_P2  			0x0C 				// 'RX address pipe2' register address
#define RX_ADDR_P3  			0x0D 				// 'RX address pipe3' register address
#define RX_ADDR_P4  			0x0E 				// 'RX address pipe4' register address
#define RX_ADDR_P5  			0x0F				// 'RX address pipe5' register address
#define TX_ADDR     			0x10 				// 'TX address' register address
#define RX_PW_P0    			0x11 				// 'RX payload width, pipe0' register address
#define RX_PW_P1    			0x12 				// 'RX payload width, pipe1' register address
#define RX_PW_P2    			0x13 				// 'RX payload width, pipe2' register address
#define RX_PW_P3    			0x14 				// 'RX payload width, pipe3' register address
#define RX_PW_P4    			0x15 				// 'RX payload width, pipe4' register address
#define RX_PW_P5    			0x16 				// 'RX payload width, pipe5' register address
#define FIFO_STATUS 			0x17 			    	// 'FIFO Status Register' register address
#define DYNPD					0x1C

#define TX_ADR_WIDTH   	5  // (5 byte-wide Transmit / Receive Address)
#define PLOAD_WIDTH 	11  // (Data path width of valid data)

//CE Pin & CSN Pin & IRQ Pin
//IRQ(PD2)   CSN(PC5)   CE(PC4)

#define IRQ	  (PIND & (1<<2))

#define CE_0      PORTC &= ~(1<<4);
#define CE_1      PORTC |= (1<<4);
	              
#define CSN_0     PORTC &= ~(1<<5);
#define CSN_1     PORTC |= (1<<5);

#define	RX_DR			0x40
#define	TX_DS			0x20
#define	MAX_RT			0x10

uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0x2A,0x2A,0x2A,0x2A,0x2A};  //	 (Define a static send address)
uint8_t RX_ADDRESS[TX_ADR_WIDTH] = {0xA2,0xA2,0xA2,0xA2,0xA2};

uint8_t RX_BUF[PLOAD_WIDTH];
uint8_t TX_BUF[PLOAD_WIDTH];
void flush_tx()
{
	CSN_0;
	SPDR = FLUSH_TX;
	while(!(SPSR & (1<<SPIF)));
	CSN_1;
}
void flush_rx()
{
	CSN_0;
	SPDR = FLUSH_RX;
	while(!(SPSR & (1<<SPIF)));
	CSN_1;
}
uint8_t SPI_RW(uint8_t Data)
{
	SPDR = Data;
	while(!(SPSR & (1<<SPIF)));
	return SPDR;
}
/****Reg to register a byte written while returning state bytes****/
uint8_t SPI_RW_Reg(uint8_t reg,uint8_t value)
{
	uint8_t status;
	CSN_0;
	status=SPI_RW(reg);   //select register  and write value to it
	SPI_RW(value);   
	CSN_1;
	return(status); 
}
/****To register reg read a byte, and return to state bytes*****/
uint8_t SPI_Read_Reg(uint8_t reg)
{
	uint8_t status;
	CSN_0;
	SPI_RW(reg);
    status = SPI_RW(0);
	CSN_1;
	return(status);
}
/********Read the bytes bytes of data****************/
uint8_t SPI_Read_Buf(uint8_t reg,uint8_t *pBuf,uint8_t bytes)
{
	uint8_t status,byte_ctr;

	CSN_0;
	status=SPI_RW(reg);       
	for(byte_ctr=0;byte_ctr<bytes;byte_ctr++)
		pBuf[byte_ctr]=SPI_RW(0);

	CSN_1;
	return(status);
}
/****************Write bytes bytes of data*************/
uint8_t SPI_Write_Buf(uint8_t reg,uint8_t *pBuf,uint8_t bytes)
{
	uint8_t status,byte_ctr;
	CSN_0;
	status=SPI_RW(reg); 
	_delay_us(10);
	for(byte_ctr=0;byte_ctr<bytes;byte_ctr++)
		SPI_RW(*(pBuf++));
	CSN_1;
	return(status);
}
void TX_Mode(uint8_t *tx_buf)
{
	
	CE_0;
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, ((1<<3)|(0<<2)|(1<<1)|(0<<0)|(1<<4)|(1<<5)));	
	flush_tx();
	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, PLOAD_WIDTH);
	CE_1;
	_delay_us(10);
} 
void nRF24L01_Initial(void)
{
	DDRB |= (1<<DDB5)|(1<<DDB3)|(1<<DDB2);
	DDRB &= ~(1<<DDB4);
	DDRC |= (1<<4) | (1<<5);
	SPCR|= (1<<SPE)|(1<<MSTR)|(1<<SPR1)|(1<<SPI2X);    
 	CSN_1;
	CE_0; 
}
void nRF24L01_Config(void)
{
	SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, ((1<<3)|(0<<2)|(1<<4)|(1<<5))); 
	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_AA, 0x3F);
	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_RXADDR, 0x3F); 
	SPI_RW_Reg(WRITE_REG_NRF24L01 + SETUP_AW, 0x03); 
	SPI_RW_Reg(WRITE_REG_NRF24L01 + SETUP_RETR, 0x4F); 
	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_CH, 15);
	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_SETUP,0x0E);
	SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS,0x0E);
	SPI_RW_Reg(WRITE_REG_NRF24L01 + OBSERVE_TX,0x00);
	SPI_RW_Reg(WRITE_REG_NRF24L01 + FIFO_STATUS,0x11);
	SPI_RW_Reg(WRITE_REG_NRF24L01 + CD,0x00);
	SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P0, PLOAD_WIDTH); 
	SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P1, PLOAD_WIDTH);
	SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P2, PLOAD_WIDTH);
	SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P3, PLOAD_WIDTH);
	SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P4, PLOAD_WIDTH);
	SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P5, PLOAD_WIDTH);
	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_SETUP, ((1<<3)|(3<<1)));
	SPI_RW_Reg(WRITE_REG_NRF24L01 + DYNPD, 0x00);
	SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, ((1<<3)|(0<<2)|(1<<4)|(1<<5)));
		
	flush_rx();
	flush_tx();
	
	SPI_Write_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P1, RX_ADDRESS, TX_ADR_WIDTH);
	CE_1; 
	CE_0; 
	SPI_Write_Buf(WRITE_REG_NRF24L01 + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);
	SPI_Write_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);
	CE_1;
	RX_Mode();
}
void NRF24L01_Send()
{
	TX_Mode(TX_BUF);
	RX_Mode();
}
void RX_Mode(void)
{
	CE_0;	
	flush_rx();
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, ((1<<3)|(0<<2)|(1<<1)|(1<<0)|(1<<4)|(1<<5))); 
  	CE_1;
	_delay_us(10);
}
void NRF24L01_Receive(void)
{
	uint8_t status=0x01;
	CE_0;
	_delay_us(10);
	status=SPI_Read_Reg(STATUS);					    
	if(status & (1<<6))								    
	{
		SPI_Read_Buf(RD_RX_PLOAD,RX_BUF,PLOAD_WIDTH);    // read receive payload from RX_FIFO buffer
		SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, (1<<6));      
	}
	CE_1;

}
