#ifndef __USART_SEND_H
#define __USART_SEND_H
#include "stm32f4xx.h"
/**
 * ??????
 * @Author   zilin-Wang
 * @DateTime 2017-02-28
 * @version  1.0
 * @param    USARTx     ???
 * @param    Byte       ????
 */
extern void Usart_send_byte(USART_TypeDef * USARTx, uint8_t Byte);
/**
 * USART????????
 * @Author   zilin-Wang
 * @DateTime 2017-02-28
 * @version  1.0
 * @param    USARTx     ???
 * @param    Array      ???
 * @param    Length     ????
 */
extern void Usart_send_array(USART_TypeDef * USARTx, uint8_t * Array, uint8_t Length);
/**
 * ???????
 * @Author   zilin-Wang
 * @DateTime 2017-02-28
 * @version  1.0	
 * @param    USARTx     ???
 * @param    String     ??
 */
extern void Usart_send_string(USART_TypeDef * USARTx, uint8_t * String);
/**
 * ??????4???
 * @Author   zilin-Wang
 * @DateTime 2017-03-01
 * @version  1.0
 * @param    USARTx     ???
 * @param    data       ???
 */
extern void Usart_send_u16(USART_TypeDef * USARTx, uint16_t data);
/**
 * ????
 * @Author   zilin-Wang
 * @DateTime 2017-03-01
 * @version  1.0
 */
extern void Usart_send_space(USART_TypeDef * USARTx);
/**
 * ????
 * @Author   zilin-Wang
 * @DateTime 2017-03-01
 * @version  1.0
 */
extern void usart_send_enter(USART_TypeDef * USARTx);
#endif