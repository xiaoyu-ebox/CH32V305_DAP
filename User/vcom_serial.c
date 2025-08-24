#include <string.h>
#include "ch32v30x.h"
#include "ch32v30x_usb.h"
#include "core_riscv.h"
#include "vcom_serial.h"
#include "tusb.h"
#include "message_buffer.h"

uint8_t RxBuffer[RX_BUFF_SZ] __attribute__((aligned(4)));
uint8_t RxBuffer2[RX_BUFF_SZ] __attribute__((aligned(4)));
uint8_t TxBuffer[TX_BUFF_SZ] __attribute__((aligned(4)));
uint8_t TxBuffer2[TX_BUFF_SZ] __attribute__((aligned(4)));

DMA_Channel_TypeDef *uartTxDma = DMA1_Channel2;     // 发送dma
DMA_Channel_TypeDef *uartRxDma = DMA1_Channel3;     // 接收dma
SemaphoreHandle_t xSemaphore = NULL;

extern void vcom_uart_rx_cb(uint8_t *data, uint16_t len);

void USART3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
    {
        USART_ReceiveData(USART3); // 清除空闲中断标志（关键！）

        // 禁止dma
        uartRxDma->CFGR &= (uint16_t)(~DMA_CFGR1_EN);

        // 计算已接收数据长度
        uint16_t remain_cnt = DMA_GetCurrDataCounter(uartRxDma); // 剩余未传输的数据量
        uint16_t received_len = sizeof(RxBuffer) - remain_cnt;   // 已接收的数据长度

        uint8_t *currRxBuffer = (uint8_t *)uartRxDma->MADDR;
        uartRxDma->MADDR = (currRxBuffer == RxBuffer) ? (uint32_t)RxBuffer2 : (uint32_t)RxBuffer;

        // 重置DMA
        uartRxDma->CNTR = sizeof(RxBuffer);
        uartRxDma->CFGR |= DMA_CFGR1_EN;

        if (received_len)
        {
            // 数据处理
            vcom_uart_rx_cb(currRxBuffer, received_len);
        }        
    }
}

// 接收完成后中断处理
void DMA1_Channel3_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_IT_TC3))
    {
        // 禁止dma
        uartRxDma->CFGR &= (uint16_t)(~DMA_CFGR1_EN);

        // 处理接收完成的数据
        uint8_t *currRxBuffer = (uint8_t *)uartRxDma->MADDR;
        uartRxDma->MADDR = (currRxBuffer == RxBuffer) ? (uint32_t)RxBuffer2 : (uint32_t)RxBuffer;

        // 重置DMA
        uartRxDma->CNTR = sizeof(RxBuffer);
        uartRxDma->CFGR |= DMA_CFGR1_EN;

        DMA_ClearITPendingBit(DMA1_IT_TC3);

        // 数据处理
        vcom_uart_rx_cb(currRxBuffer, sizeof(RxBuffer));
    }
}

// 发送完成后中断处理
void DMA1_Channel2_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_IT_TC2))
    {
        DMA_ClearITPendingBit(DMA1_IT_TC2);

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken); // 如果需要，切换任务
    }
}

uint32_t VCOM_Send_DMA(uint8_t *data, uint16_t len)
{
    if (len > TX_BUFF_SZ)
        len = TX_BUFF_SZ;

    if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(5000)) != pdPASS)
    {
        return 0;
    }

    // 禁止dma
    uartTxDma->CFGR &= (uint16_t)(~DMA_CFGR1_EN);

    // 更新buf指针
    uartTxDma->MADDR = (uint32_t)data;

    // 重置DMA
    uartTxDma->CNTR = len;
    uartTxDma->CFGR |= DMA_CFGR1_EN;

    return len;
}

uint8_t *VCOM_Get_Send_Idle_Buffer(void)
{
    return ((uint8_t *)(uartTxDma->MADDR) == TxBuffer) ? TxBuffer2 : TxBuffer;
}

void VCOM_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    xSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(xSemaphore);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure); // PB10 => USART3_TX

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure); // PB11 => USART3_RX

    /*******发送 Channel *******/
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART3->DATAR;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)TxBuffer;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_BufferSize = 0;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_ITConfig(uartTxDma, DMA_IT_TC, ENABLE); // 关键代码
    DMA_Init(uartTxDma, &DMA_InitStructure);
    DMA_Cmd(uartTxDma, ENABLE);

    /*******接收 Channel *******/
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART3->DATAR;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)RxBuffer;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_BufferSize = sizeof(RxBuffer);
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_ITConfig(uartRxDma, DMA_IT_TC, ENABLE); // 关键代码
    DMA_Init(uartRxDma, &DMA_InitStructure);
    DMA_Cmd(uartRxDma, ENABLE);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART3, &USART_InitStructure);

    USART_DMACmd(USART3, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);

    // 配置NVIC中断优先级
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART3, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;    // 接收DMA1 通道2 中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   // 抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;          // 子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel3_IRQn;    // 发送DMA1 通道3 中断通道
    NVIC_Init(&NVIC_InitStructure);
}

void VCOM_SetLineCoding(cdc_line_coding_t const *p_line_coding)
{
    USART_InitTypeDef USART_InitStructure;

    switch (p_line_coding->data_bits)
    {
    case 8:
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
        break;
    default:
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
        break;
    }

    switch (p_line_coding->parity)
    {
    case 0:
        USART_InitStructure.USART_Parity = USART_Parity_No;
        break;
    case 1:
        USART_InitStructure.USART_Parity = USART_Parity_Odd;
        USART_InitStructure.USART_WordLength = USART_WordLength_9b;
        break;
    case 2:
        USART_InitStructure.USART_Parity = USART_Parity_Even;
        USART_InitStructure.USART_WordLength = USART_WordLength_9b;
        break;
    default:
        USART_InitStructure.USART_Parity = USART_Parity_No;
        break;
    }

    switch (p_line_coding->stop_bits)
    {
    case 0:
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        break;
    case 1:
        USART_InitStructure.USART_StopBits = USART_StopBits_1_5;
        break;
    case 2:
        USART_InitStructure.USART_StopBits = USART_StopBits_2;
        break;
    default:
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        break;
    }

    USART_InitStructure.USART_BaudRate = p_line_coding->bit_rate;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    __disable_irq();

    USART_Init(USART3, &USART_InitStructure);

    __enable_irq();
}
