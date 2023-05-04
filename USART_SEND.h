#ifndef __USART_SEND_H
#define __USART_SEND_H
#include "stm32f4xx.h"

extern void Usart_send_byte(USART_TypeDef * USARTx, uint8_t Byte);
extern void Usart_send_array(USART_TypeDef * USARTx, uint8_t * Array, uint8_t Length);
extern void Usart_send_string(USART_TypeDef * USARTx, uint8_t * String);
extern void Usart_send_u16(USART_TypeDef * USARTx, uint16_t data);
extern void Usart_send_space(USART_TypeDef * USARTx);
extern void usart_send_enter(USART_TypeDef * USARTx);
#endif
