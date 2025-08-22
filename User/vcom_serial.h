#ifndef __VCOM_SERIAL_H__
#define __VCOM_SERIAL_H__

#include "class/cdc/cdc.h"

#define RX_BUFF_SIZE 1024 // RX buffer size
#define TX_BUFF_SIZE 1024 // RX buffer size

typedef struct
{
	uint8_t rx_buff[RX_BUFF_SIZE];
	uint16_t rx_bytes;
	uint16_t rx_wrptr;
	uint16_t rx_rdptr;
	uint8_t tx_buff[TX_BUFF_SIZE];
	uint16_t tx_bytes;
	uint16_t tx_wrptr;
	uint16_t tx_rdptr;

	uint16_t hw_flow; // BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send)

	uint8_t in_buff[512];
	uint16_t in_bytes;
	uint16_t in_ready;
	uint8_t out_buff[512];
	uint16_t out_bytes;
	uint16_t out_ready;
} VCOM;

extern volatile VCOM Vcom;

void VCOM_Init(void);
void VCOM_SetLineCoding(cdc_line_coding_t const *p_line_coding);

void VCOM_Send_DMA(uint8_t *data, uint16_t len);

#endif // __VCOM_SERIAL_H__
