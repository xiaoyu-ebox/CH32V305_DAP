#ifndef __VCOM_SERIAL_H__
#define __VCOM_SERIAL_H__

#include "class/cdc/cdc.h"

#define RX_BUFF_SZ 256
#define TX_BUFF_SZ 256


void VCOM_Init(void);
void VCOM_SetLineCoding(cdc_line_coding_t const *p_line_coding);

void VCOM_Send_DMA(uint8_t *data, uint16_t len);

#endif // __VCOM_SERIAL_H__
