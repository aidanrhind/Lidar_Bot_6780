#include "USART_SEND.h"
#include "stm32f4xx.h"

void Usart_send_byte(USART_TypeDef * USARTx, uint8_t Byte)
{
	while((USARTx->SR & USART_SR_TC) != USART_SR_TC);		
	USARTx->DR = Byte;	
}

void Usart_send_array(USART_TypeDef * USARTx, uint8_t * Array, uint8_t Length)
{
	while(Length-- != 0)
	{
		while((USARTx->SR & USART_SR_TC) != USART_SR_TC);	
		USARTx->DR = *Array++;
	}
}


void Usart_send_string(USART_TypeDef * USARTx, uint8_t * String)
{
	while(*String != '\0')
	{
		while((USARTx->SR & USART_SR_TC) != USART_SR_TC);	
		USARTx->DR = *String++;
	}
}

void Usart_send_u16(USART_TypeDef * USARTx, uint16_t data)
{
	if (data >= 1000)
	{
		while((USARTx->SR & USART_SR_TC) != USART_SR_TC);		
		USARTx->DR = (uint8_t)(data / 1000) + 48;	
		while((USARTx->SR & USART_SR_TC) != USART_SR_TC);		
		USARTx->DR = (uint8_t)((data % 1000) / 100) + 48;			
		while((USARTx->SR & USART_SR_TC) != USART_SR_TC);		
		USARTx->DR = (uint8_t)((data % 100) / 10) + 48;		
		while((USARTx->SR & USART_SR_TC) != USART_SR_TC);		
		USARTx->DR = (uint8_t)(data % 10) + 48;	
	}
	if (data >= 100)
	{
		while((USARTx->SR & USART_SR_TC) != USART_SR_TC);		
		USARTx->DR = (uint8_t)((data % 1000) / 100) + 48;			
		while((USARTx->SR & USART_SR_TC) != USART_SR_TC);		
		USARTx->DR = (uint8_t)((data % 100) / 10) + 48;		
		while((USARTx->SR & USART_SR_TC) != USART_SR_TC);		
		USARTx->DR = (uint8_t)(data % 10) + 48;		
	}
	else if (data >= 10)
	{
		while((USARTx->SR & USART_SR_TC) != USART_SR_TC);		
		USARTx->DR = (uint8_t)(data / 10) + 48;	
		while((USARTx->SR & USART_SR_TC) != USART_SR_TC);		
		USARTx->DR = (uint8_t)(data % 10) + 48;
	}
	else
	{
		while((USARTx->SR & USART_SR_TC) != USART_SR_TC);		
		USARTx->DR = (uint8_t)(data % 10) + 48;
	}
}

void Usart_send_space(USART_TypeDef * USARTx)
{
	while((USARTx->SR & USART_SR_TC) != USART_SR_TC);		
	USARTx->DR = 0x20;//??			
}

void usart_send_enter(USART_TypeDef * USARTx)
{
	while((USARTx->SR & USART_SR_TC) != USART_SR_TC);		
	USARTx->DR = 0x0d;	
	while((USARTx->SR & USART_SR_TC) != USART_SR_TC);		
	USARTx->DR = 0x0a;		
}
