#ifndef BSP_RC_H
#define BSP_RC_H
#include "struct_typedef.h"
#include "main.h"

 extern UART_HandleTypeDef huart3;
 extern DMA_HandleTypeDef hdma_usart3_rx;

void RC_restart(uint16_t dma_buf_num);
extern void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
#endif
