/**
 *  @file SERIAL_Port.h
 *
 *  @date 2022年10月18日 10:20:26 星期二
 *
 *  @author Copyright (c) 2022 aron566 <aron566@163.com>.
 *
 *  @brief 串口操作接口.
 *
 *  @version v1.0.0
 */
#ifndef _SERIAL_PORT_H
#define _SERIAL_PORT_H
/** Includes -----------------------------------------------------------------*/
#include <stdint.h> /**< need definition of uint8_t */
#include <stddef.h> /**< need definition of NULL    */
#include <stdbool.h>/**< need definition of BOOL    */
#include <stdio.h>  /**< if need printf             */
#include <stdlib.h>
#include <string.h>
// #include <limits.h> /**< need variable max value    */
// #include <stdalign.h> /**< need alignof    */
// #include <stdarg.h> /**< need va_start    */
/** Private includes ---------------------------------------------------------*/
#include "CircularQueue.h"
/** Use C compiler -----------------------------------------------------------*/
#ifdef __cplusplus ///< use C compiler
extern "C" {
#endif
/** Private defines ----------------------------------------------------------*/
#define USE_USB_CDC   0
/** Exported typedefines -----------------------------------------------------*/

/* 串口端口号 */
typedef enum
{
  SERIAL_PORT_UART_0 = 0,
  SERIAL_PORT_UART_1,
  SERIAL_PORT_UART_2,
  SERIAL_PORT_UART_3,
  SERIAL_PORT_UART_4,
  SERIAL_PORT_UART_5,
  SERIAL_PORT_UART_6,
  SERIAL_PORT_UART_7,
  SERIAL_PORT_UART_8,
  SERIAL_PORT_USB_CDC_0,
  SERIAL_PORT_USB_CDC_1,
  SERIAL_PORT_USB_CDC_2,
  SERIAL_PORT_NUM_MAX,
}SERIAL_PORT_SERIAL_NUM_Typedef_t;

/* 串口状态 */
typedef enum
{
  SERIAL_PORT_READY     = 0,
  SERIAL_PORT_TX_IDEL   = 1 << 0,
  SERIAL_PORT_RX_IDEL   = 1 << 1,
  SERIAL_PORT_RX_TX_IDEL= 3,
  SERIAL_PORT_TX_BUSY   = 1 << 2,
  SERIAL_PORT_RX_BUSY   = 1 << 3,
  SERIAL_PORT_RX_TX_BUSY= 12,
  SERIAL_PORT_ERROR     = 1 << 4,
}SERIAL_STATE_Typedef_t;



/** Exported constants -------------------------------------------------------*/

/** Exported macros-----------------------------------------------------------*/
/** Exported variables -------------------------------------------------------*/
/** Exported functions prototypes --------------------------------------------*/

/**
 * @brief 串口初始化
 *
 */
void Serial_Port_Init(void);

/**
 * @brief 串口反初始化
 *
 */
void Serial_Port_DeInit(void);

/**
 * @brief 串口数据发送
 *
 * @param Serial_Num 串口号
 * @param Data 数据，阻塞时间为0时不可使用局部变量
 * @param Len 数据长度
 * @param Block_Time 阻塞时间
 * @return true 调用发送成功
 * @return false 调用发送失败
 */
bool Serial_Port_Transmit_Data(SERIAL_PORT_SERIAL_NUM_Typedef_t Serial_Num, const uint8_t *Data, uint32_t Len, uint32_t Block_Time);

/**
 * @brief 获取串口空闲状态
 *
 * @param Serial_Num 串口号
 * @return SERIAL_STATE_Typedef_t 状态
 */
SERIAL_STATE_Typedef_t Serial_Port_Get_Idel_State(SERIAL_PORT_SERIAL_NUM_Typedef_t Serial_Num);

/**
 * @brief 获取环形句柄
 *
 * @param Serial_Num 串口号
 * @return CQ_handleTypeDef* 环形句柄
 */
CQ_handleTypeDef *Serial_Port_Get_CQ(SERIAL_PORT_SERIAL_NUM_Typedef_t Serial_Num);

/**
 * @brief 串口半中断
 *
 * @param Arg 串口句柄参数
 */
void Serial_Port_Half_IT_CallBack(void *Arg);

/**
 * @brief 串口完成中断
 *
 * @param Arg 串口句柄参数
 */
void Serial_Port_Complete_IT_CallBack(void *Arg);

/**
 * @brief 串口中断
 *
 * @param Arg 串口句柄参数
 * @param Size 数据大小字节
 */
void Serial_Port_IT_CallBack(void *Arg, uint32_t Size);

/**
 * @brief CDC串口完成中断
 *
 * @param Arg CDC句柄参数
 * @param Data 数据
 * @param Len 数据长度
 * @param EPNum 端口号
 */
void Serial_Port_Complete_CDC_IT_CallBack(void *Arg, const uint8_t *Data, uint32_t Len, uint8_t EPNum);

/**
 * @brief 设置串口波特率
 *
 * @param Serial_Num 串口号
 * @param BaudRate 波特率
 */
void Serial_Port_Set_BaudRate(SERIAL_PORT_SERIAL_NUM_Typedef_t Serial_Num, uint32_t BaudRate);

#ifdef __cplusplus ///<end extern c
}
#endif
#endif
/******************************** End of file *********************************/
