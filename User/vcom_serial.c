#include <string.h>
#include "ch32v30x.h"
#include "ch32v30x_usb.h"
// #include "ch32v30x_usbhs_device.h"
#include "core_riscv.h"
// #include "usb_desc.h"
#include "vcom_serial.h"
#include "tusb.h"
#include "message_buffer.h"

#define RX_BUFF_SZ 256
#define TX_BUFF_SZ 256
#define RXDMA_SZ (RX_BUFF_SZ * 1)

uint8_t RxBuffer[RXDMA_SZ] __attribute__((aligned(4)));
uint8_t TxBuffer[TX_BUFF_SZ] __attribute__((aligned(4)));

DMA_Channel_TypeDef *uartTxDma = DMA1_Channel2;
DMA_Channel_TypeDef *uartRxDma = DMA1_Channel3;

extern MessageBufferHandle_t cdc_rcv_queue;
extern void usbd_defer_func(osal_task_func_t func, void *param, bool in_isr);

void USART3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
    {
        USART_ReceiveData(USART3); // 清除空闲中断标志（关键！）

        // 计算已接收数据长度
        uint16_t remain_cnt = DMA_GetCurrDataCounter(uartRxDma); // 剩余未传输的数据量
        uint16_t received_len = RXDMA_SZ - remain_cnt;           // 已接收的数据长度

        if (received_len && cdc_rcv_queue)
        {
            // 重置DMA（循环模式下自动覆盖旧数据）
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            /*size_t xBytesSent = */ xMessageBufferSendFromISR(cdc_rcv_queue,
                                                               (void *)RxBuffer, received_len,
                                                               &xHigherPriorityTaskWoken);

            // 触发usb任务立即跑cdc_task
            usbd_defer_func(NULL, NULL, true);
        }
        // 重置DMA（循环模式下自动覆盖旧数据）
        DMA_Cmd(uartRxDma, DISABLE);
        DMA_SetCurrDataCounter(uartRxDma, RXDMA_SZ);
        DMA_Cmd(uartRxDma, ENABLE);
    }
}

// 发送完成后中断处理
void DMA1_Channel2_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_IT_TC2))
    {
        DMA_ClearITPendingBit(DMA1_IT_TC2);
    }
}

void DMA1_Channel3_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_IT_TC3))
    {
        DMA_ClearITPendingBit(DMA1_IT_TC3);
        // 处理接收完成的数据
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (cdc_rcv_queue)
        {
            /*size_t xBytesSent = */ xMessageBufferSendFromISR(cdc_rcv_queue,
                                                               (void *)RxBuffer, RXDMA_SZ,
                                                               &xHigherPriorityTaskWoken);
        }
        // 重置DMA（循环模式下自动覆盖旧数据）
        DMA_Cmd(uartRxDma, DISABLE);
        DMA_SetCurrDataCounter(uartRxDma, RXDMA_SZ);
        DMA_Cmd(uartRxDma, ENABLE);
    }
}

void VCOM_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;

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
    DMA_InitStructure.DMA_BufferSize = RXDMA_SZ;
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

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;  // DMA1 通道2 中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // 抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        // 子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel3_IRQn; // DMA1 通道3 中断通道
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

void VCOM_Send_DMA(uint8_t *data, uint16_t len)
{
    DMA_Cmd(uartTxDma, DISABLE); // 关闭DMA
    if (len > TX_BUFF_SZ)
        len = TX_BUFF_SZ;
    memcpy(TxBuffer, data, len);
    DMA_SetCurrDataCounter(uartTxDma, len); // 设置传输长度
    DMA_Cmd(uartTxDma, ENABLE);             // 启动DMA
}
