/**
 *  @file SERIAL_Port.c
 *
 *  @date 2022年10月18日 10:20:14 星期二
 *
 *  @author aron566
 *
 *  @copyright Copyright (c) 2022 aron566 <aron566@163.com>.
 *
 *  @brief 串口数据操作接口.
 *
 *  @details 2022-10-18 v1.0.0 首个版本，使用空闲中断加FIFO加乒乓双缓冲优化.
                               空闲中断：接收数据大小 = DMA通道接收总数据大小（DMA通道buf大小 - DMA通道buf剩余空间大小） - 上一次接收的总数据大小
                               半中断：接收数据大小 = DMA通道接收总数据大小（DMA通道buf大小 - DMA通道buf剩余空间大小） - 上一次接收的总数据大小
                               满中断：接收数据大小 = DMA通道buf大小 - 上一次接收的总数据大小
                               硬件配置：开循环接收，FIFO，DMA中断
                               只有开启DMA循环接收模式才有半接收中断，使用HAL_UARTEx_ReceiveToIdle_DMA接口，半接收中断为HAL_UARTEx_RxEventCallback
                               空闲中断中必须先停止接收才能拿到正确数据

 *  @version v1.0.0
 */
/** Includes -----------------------------------------------------------------*/
/* Private includes ----------------------------------------------------------*/
#include "SERIAL_Port.h"
#include "usart.h"
#include "utilities.h"
#if USE_USB_CDC
  #include "usbd_cdc_if.h"
#endif
#include "main.h"
/** Use C compiler -----------------------------------------------------------*/
#ifdef __cplusplus ///< use C compiler
extern "C" {
#endif
/** Private macros -----------------------------------------------------------*/

#define USE_IDEL_CALLBACK         1 /**< 使用空闲中断 */
#define USE_DMA_SEND_FIRST        1 /**< 当阻塞时间为0时，优先使用DMA发送数据而非中断方式 */
#define USE_DMA_BUF_ADDR          0 /**< 是否检测DMA缓冲区地址，处于这个地址内的数据才能DMA收发，避免Cache影响 */
#define USE_DMA_CIRCULAR_MODE     1 /**< 已使用硬件DMA循环接收模式，1代表打开，0代表未打开（未打开或者主动设置为0时，接收到数据将先停止接收再主动重启接收） */
#define USE_LOOPBACK              1 /**< 是否使用数据回环打印 */
#define SERIAL_PORT_DEBUG_UART    &huart2 /**< 使用Printf接口的串口 */

/* 动态内存申请接口 */
#define SERIAL_PORT_MALLOC        malloc
#define SERIAL_PORT_FREE          free

/* 空闲任务 */
#define SERIAL_PORT_WAIT_DO()     do{ \
                                      \
                                  }while(0)
#if USE_DMA_SEND_FIRST
  #if USE_DMA_BUF_ADDR
    #if defined(__CC_ARM) || (defined(__ARMCC_VERSION) && __ARMCC_VERSION >= 6000000)
      extern const uint32_t USE_DMA_BUF_SPACE$$Base;
      extern const uint32_t USE_DMA_BUF_SPACE$$Limit;
      static const uint32_t *DMA_Buf_Addr_Start = (&USE_DMA_BUF_SPACE$$Base);
      static const uint32_t *DMA_Buf_Addr_End   = (&USE_DMA_BUF_SPACE$$Limit);
    #elif defined(__ICCARM__) || defined(__ICCRX__)/**< IAR方式*/
      #pragma section="USE_DMA_BUF_SPACE"
      static const uint32_t DMA_Buf_Addr_Start = (__section_begin("USE_DMA_BUF_SPACE"));
      static const uint32_t DMA_Buf_Addr_End = (__section_end("USE_DMA_BUF_SPACE"));
    #elif defined(__GNUC__)
        extern const uint32_t _USE_DMA_BUF_SPACE_start;
        extern const uint32_t _USE_DMA_BUF_SPACE_end;
        static const uint32_t *DMA_Buf_Addr_Start = (&_USE_DMA_BUF_SPACE_start);
        static const uint32_t *DMA_Buf_Addr_End = (&_USE_DMA_BUF_SPACE_end);
    #else
      #error not supported compiler, please use command table mode
    #endif
  #else
    static const uint32_t DMA_Buf_Addr_Start = 1;
    static const uint32_t DMA_Buf_Addr_End = 0xFFFFFFFF;
  #endif
#endif
/** Private typedef ----------------------------------------------------------*/

/* 串口设备对象 */
typedef struct Serial_Port_Handle
{
  /* 串口句柄 */
  void *pSerial_Handle;

  /* 数据缓存 */
  CQ_handleTypeDef *pCQ_Handle;
  uint8_t *pReceive_Buffer;
  uint32_t Buffer_Size;

  /* 接收数据大小累积，每次满中断置0 */
  uint32_t Last_Rec_Data_Size_Cnt;

  /* 阻塞 */
  void *pSemaphoreId;
  void (*pLock_CallBack)(void *pSemaphoreId);
  void (*pUnLock_CallBack)(void *pSemaphoreId);

  /* 初始化，反初始化 */
  void (*pInit)(void *pSerial_Port_Handle);
  void (*pDeInit)(void *pSerial_Port_Handle);

  /* 发送，接收接口 */
  bool (*pSend_Data_Start)(void *pSerial_Port_Handle, const uint8_t *Data, uint32_t Len, uint32_t BlockTime);
  bool (*pReceive_Data_Start)(void *pSerial_Port_Handle, uint8_t *Buffer, uint32_t Len, uint32_t BlockTime);

  /* 启动接口 */
  bool (*pStart)(void *pSerial_Port_Handle);

  /* 获取状态 */
  SERIAL_STATE_Typedef_t (*pGet_Serial_State)(void *pSerial_Port_Handle);

  /* 设置波特率 */
  bool (*pSet_BaudRate)(void *pSerial_Port_Handle, uint32_t BaudRate);

  /* 中断回调接口 */
  void (*pReceive_Half_IT)(void *pSerial_Port_Handle);
  void (*pReceive_Complete_IT)(void *pSerial_Port_Handle);
  void (*pSerial_IT)(void *pSerial_Port_Handle);

  /* USB CDC私有 */
  uint8_t INepNum;                /**< 主机接收端点地址 */
  uint8_t OUTepNum;               /**< 主机发送端点地址 */
  void (*pSerial_Rec_IT)(void *pSerial_Port_Handle, const uint8_t *Data, uint32_t Len);
}SERIAL_PORT_HANDLE_Typedef_t;

/** Private constants --------------------------------------------------------*/
/** Public variables ---------------------------------------------------------*/
#if USE_USB_CDC
  extern USBD_HandleTypeDef hUsbDeviceFS;
  extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
#endif
/** Private variables --------------------------------------------------------*/
/* 串口设备对象列表 */
static SERIAL_PORT_HANDLE_Typedef_t Serial_Device_List[SERIAL_PORT_NUM_MAX];

/* 串口缓存 */
static uint8_t Uart2_DMA_Buf[16] MATH_PORT_SECTION("USE_DMA_BUF_SPACE") = {0};
/** Private function prototypes ----------------------------------------------*/

/** Private user code --------------------------------------------------------*/

/** Private application code -------------------------------------------------*/
/*******************************************************************************
*
*       Static code
*
********************************************************************************
*/

/**
 * @brief UART串口启动
 *
 * @param pSerial_Port_Handle 串口设备句柄
 * @return true 成功
 * @return false
 */
static bool UART_Start(void *pSerial_Port_Handle)
{
  SERIAL_PORT_HANDLE_Typedef_t *pSerial_Device = (SERIAL_PORT_HANDLE_Typedef_t *)pSerial_Port_Handle;

  /* 启动接收 */
  return pSerial_Device->pReceive_Data_Start(pSerial_Port_Handle, pSerial_Device->pReceive_Buffer, pSerial_Device->Buffer_Size, 0);
}

/**
 * @brief UART串口初始化
 *
 * @param pSerial_Port_Handle 串口设备句柄
 */
static void UART_Init(void *pSerial_Port_Handle)
{
  SERIAL_PORT_HANDLE_Typedef_t *pSerial_Device = (SERIAL_PORT_HANDLE_Typedef_t *)pSerial_Port_Handle;
  UART_HandleTypeDef *phuart = (UART_HandleTypeDef *)pSerial_Device->pSerial_Handle;
  HAL_UART_Abort(phuart);
  HAL_UART_Init(phuart);
}

/**
 * @brief UART串口反初始化
 *
 * @param pSerial_Port_Handle 串口设备句柄
 */
static void UART_DeInit(void *pSerial_Port_Handle)
{
  SERIAL_PORT_HANDLE_Typedef_t *pSerial_Device = (SERIAL_PORT_HANDLE_Typedef_t *)pSerial_Port_Handle;
  UART_HandleTypeDef *phuart = (UART_HandleTypeDef *)pSerial_Device->pSerial_Handle;
  HAL_UART_Abort(phuart);
  HAL_UART_DeInit(phuart);
}

/**
 * @brief 串口发送数据
 *
 * @param pSerial_Port_Handle 串口设备句柄
 * @param Data 数据，阻塞时间为0时不可使用局部变量
 * @param Len 数据长度
 * @param BlockTime 阻塞时间ms
 * @return true 发送正常
 * @return false 发送错误
 */
static bool UART_Send_Data_Start(void *pSerial_Port_Handle, const uint8_t *Data, uint32_t Len, uint32_t BlockTime)
{
  SERIAL_PORT_HANDLE_Typedef_t *pSerial_Device = (SERIAL_PORT_HANDLE_Typedef_t *)pSerial_Port_Handle;
  UART_HandleTypeDef *phuart = (UART_HandleTypeDef *)pSerial_Device->pSerial_Handle;

  /* 阻塞发送 */
  if(BlockTime > 0)
  {
    if(HAL_OK != HAL_UART_Transmit(phuart, Data, Len, BlockTime))
    {
      return false;
    }
    return true;
  }

  /* 不阻塞发送 */
#if USE_DMA_SEND_FIRST///<! DMA优先发送
  /* when the address in dma buf. */
  if((uint32_t)Data >= (uint32_t)DMA_Buf_Addr_Start && (uint32_t)Data < (uint32_t)DMA_Buf_Addr_End)
  {
    /* 如用等待DMA空闲 */
    while(HAL_DMA_GetState(phuart->hdmatx) == HAL_DMA_STATE_BUSY)
    {
      /* do something... */
      SERIAL_PORT_WAIT_DO();
    }

    if(phuart->gState != HAL_UART_STATE_READY)
    {
      HAL_UART_AbortTransmit(phuart);
    }

    /* 关闭DMA */
    __HAL_DMA_DISABLE(phuart->hdmatx);

    if(HAL_OK != HAL_UART_Transmit_DMA(phuart, Data, Len))
    {
      return false;
    }
  }
  else
  {
    if(HAL_OK != HAL_UART_Transmit_IT(phuart, Data, Len))
    {
      return false;
    }
  }
#else
  if(HAL_OK != HAL_UART_Transmit_IT(phuart, Data, Len))
  {
    return false;
  }
#endif

  return true;
}

/**
 * @brief 串口接收数据
 *
 * @param pSerial_Port_Handle 串口设备句柄
 * @param Buffer 接收缓冲区
 * @param Len 接收长度
 * @param BlockTime 阻塞时间，暂未使用保留
 * @return true 接收正常
 * @return false 接收失败
 */
static bool UART_Receive_Data_Start(void *pSerial_Port_Handle, uint8_t *Buffer, uint32_t Len, uint32_t BlockTime)
{
  (void)(BlockTime);
  SERIAL_PORT_HANDLE_Typedef_t *pSerial_Device = (SERIAL_PORT_HANDLE_Typedef_t *)pSerial_Port_Handle;
  UART_HandleTypeDef *phuart = (UART_HandleTypeDef *)pSerial_Device->pSerial_Handle;

#if USE_IDEL_CALLBACK

  /* 打开空闲中断，使能DMA接收 */
  if(HAL_OK != HAL_UARTEx_ReceiveToIdle_DMA(phuart, Buffer, Len))
  {
    return false;
  }
#else

  /* 单字节中断 */
  if(HAL_OK != HAL_UART_Receive_IT(phuart, Buffer, 1))
  {
    return false;
  }
#endif
  return true;
}

/**
 * @brief 获取串口空闲状态
 *
 * @param pSerial_Port_Handle 串口设备句柄
 * @return SERIAL_STATE_Typedef_t 串口状态
 */
static SERIAL_STATE_Typedef_t UART_Get_Idel_State(void *pSerial_Port_Handle)
{
  SERIAL_PORT_HANDLE_Typedef_t *pSerial_Device = (SERIAL_PORT_HANDLE_Typedef_t *)pSerial_Port_Handle;
  UART_HandleTypeDef *phuart = (UART_HandleTypeDef *)pSerial_Device->pSerial_Handle;
  uint32_t State = (uint32_t)SERIAL_PORT_READY;
  if(HAL_UART_GetState(phuart) != HAL_UART_STATE_BUSY_TX)
  {
    State |= SERIAL_PORT_TX_IDEL;
  }
  else
  {
    State |= SERIAL_PORT_TX_BUSY;
  }
  if(HAL_UART_GetState(phuart) != HAL_UART_STATE_BUSY_RX)
  {
    State |= SERIAL_PORT_RX_IDEL;
  }
  else
  {
    State |= SERIAL_PORT_RX_BUSY;
  }

  return (SERIAL_STATE_Typedef_t)State;
}

/**
 * @brief UART设置串口波特率
 *
 * @param pSerial_Port_Handle 串口设备句柄
 * @param BaudRate 波特率
 * @return true 设置成功
 * @return false 设置失败
 */
static bool UART_Set_BaudRate(void *pSerial_Port_Handle, uint32_t BaudRate)
{
  SERIAL_PORT_HANDLE_Typedef_t *pSerial_Device = (SERIAL_PORT_HANDLE_Typedef_t *)pSerial_Port_Handle;
  UART_HandleTypeDef *phuart = (UART_HandleTypeDef *)pSerial_Device->pSerial_Handle;
  switch(BaudRate)
  {
    case 1500000:
    case 115200:
    case 57600:
    case 56000:
    case 38400:
    case 19200:
    case 14400:
    case 9600:
    case 4800:
    case 2400:
      if(phuart->Init.BaudRate == BaudRate)
      {
        return false;
      }

      /* 重置 */
      pSerial_Device->pDeInit(pSerial_Port_Handle);
      phuart->Init.BaudRate = BaudRate;
      pSerial_Device->pInit(pSerial_Port_Handle);

      /* 重新打开接收 */
      pSerial_Device->pReceive_Data_Start(pSerial_Port_Handle, pSerial_Device->pReceive_Buffer, pSerial_Device->Buffer_Size, 0);
      break;
    default:
      return false;
  }
  return true;
}

/**
 * @brief 串口半接收中断
 *
 * @param pSerial_Port_Handle 串口设备句柄
 */
static void UART_Receive_Half_IT(void *pSerial_Port_Handle)
{
  SERIAL_PORT_HANDLE_Typedef_t *pSerial_Device = (SERIAL_PORT_HANDLE_Typedef_t *)pSerial_Port_Handle;
  UART_HandleTypeDef *phuart = (UART_HandleTypeDef *)pSerial_Device->pSerial_Handle;

  /* 计算已接收数据长度 */
  uint32_t Rec_Total = pSerial_Device->Buffer_Size - __HAL_DMA_GET_COUNTER(phuart->hdmarx);

  /* 计算本次长度 */
  uint32_t Rec_Len = Rec_Total - pSerial_Device->Last_Rec_Data_Size_Cnt;

  /* 非循环接收模式下，重启接收 */
#if USE_DMA_CIRCULAR_MODE == 0

  /* 停止接收 */
  HAL_UART_AbortReceive(phuart);
#endif

  /* 将数据记录至环形区 */
  CQ_putData(pSerial_Device->pCQ_Handle, pSerial_Device->pReceive_Buffer + pSerial_Device->Last_Rec_Data_Size_Cnt, Rec_Len);

  /* 数据回显 */
#if USE_LOOPBACK
  pSerial_Device->pSend_Data_Start(pSerial_Port_Handle, pSerial_Device->pReceive_Buffer + pSerial_Device->Last_Rec_Data_Size_Cnt, Rec_Len, 0xFFFF);
#endif

  /* 非循环接收模式下，需要重启接收 */
#if USE_DMA_CIRCULAR_MODE == 0

  /* 重启接收 */
  pSerial_Device->pStart(pSerial_Port_Handle);
#endif

  /* 累积已接收长度 */
  pSerial_Device->Last_Rec_Data_Size_Cnt = Rec_Total;
}

/**
 * @brief 串口接收完成中断
 *
 * @param pSerial_Port_Handle 串口设备句柄
 */
static void UART_Receive_Complete_IT(void *pSerial_Port_Handle)
{
  SERIAL_PORT_HANDLE_Typedef_t *pSerial_Device = (SERIAL_PORT_HANDLE_Typedef_t *)pSerial_Port_Handle;

#if USE_IDEL_CALLBACK

  /* 计算本次长度 */
  uint32_t Rec_Len = pSerial_Device->Buffer_Size - pSerial_Device->Last_Rec_Data_Size_Cnt;
#else

  /* 单字节接收 */
  uint32_t Rec_Len = 1;
#endif

  /* 非循环接收模式下，重启接收 */
#if USE_DMA_CIRCULAR_MODE == 0
  UART_HandleTypeDef *phuart = (UART_HandleTypeDef *)pSerial_Device->pSerial_Handle;

  /* 停止接收 */
  HAL_UART_AbortReceive(phuart);
#endif

  /* 将数据记录至环形区 */
  CQ_putData(pSerial_Device->pCQ_Handle, pSerial_Device->pReceive_Buffer + pSerial_Device->Last_Rec_Data_Size_Cnt, Rec_Len);

  /* 数据回显 */
#if USE_LOOPBACK
  pSerial_Device->pSend_Data_Start(pSerial_Port_Handle, pSerial_Device->pReceive_Buffer + pSerial_Device->Last_Rec_Data_Size_Cnt, Rec_Len, 0xFFFF);
#endif

  /* 非循环接收模式下，需要重启接收 */
#if USE_DMA_CIRCULAR_MODE == 0

  /* 重启接收 */
  pSerial_Device->pStart(pSerial_Port_Handle);
#endif

  /* 重置已接收数据长度 */
  pSerial_Device->Last_Rec_Data_Size_Cnt = 0;
}

/**
 * @brief 串口空闲中断
 *
 * @param pSerial_Port_Handle 串口设备句柄
 */
static void UART_Idel_IT(void *pSerial_Port_Handle)
{
  SERIAL_PORT_HANDLE_Typedef_t *pSerial_Device = (SERIAL_PORT_HANDLE_Typedef_t *)pSerial_Port_Handle;
  UART_HandleTypeDef *phuart = (UART_HandleTypeDef *)pSerial_Device->pSerial_Handle;

  /* 调用 HAL_UARTEx_ReceiveToIdle_DMA 接口则不需要手动清除空闲中断 */
#if 0
  /* 检测是否是空闲中断 */
  if(__HAL_UART_GET_FLAG(phuart, UART_FLAG_IDLE) == RESET)
  {
    return;
  }

  /* 清除空闲中断标志，否则会一直不断进入中断 */
  __HAL_UART_CLEAR_IDLEFLAG(phuart);
#endif

  /* 计算已接收数据长度 */
  uint32_t Rec_Total = pSerial_Device->Buffer_Size - __HAL_DMA_GET_COUNTER(phuart->hdmarx);

  /* 计算本次长度 */
  uint32_t Rec_Len = Rec_Total - pSerial_Device->Last_Rec_Data_Size_Cnt;

  /* 非循环接收模式下，重启接收 */
#if USE_DMA_CIRCULAR_MODE == 0

  /* 停止接收 */
  HAL_UART_AbortReceive(phuart);

  /* 重置为0 */
  Rec_Total = 0;
#endif

  /* 将数据记录至环形区 */
  CQ_putData(pSerial_Device->pCQ_Handle, pSerial_Device->pReceive_Buffer + pSerial_Device->Last_Rec_Data_Size_Cnt, Rec_Len);

  /* 数据回显 */
#if USE_LOOPBACK
  pSerial_Device->pSend_Data_Start(pSerial_Port_Handle, pSerial_Device->pReceive_Buffer + pSerial_Device->Last_Rec_Data_Size_Cnt, Rec_Len, 0xFFFF);
#endif

  /* 非循环接收模式下，需要重启接收 */
#if USE_DMA_CIRCULAR_MODE == 0

  /* 重启接收 */
  pSerial_Device->pStart(pSerial_Port_Handle);
#endif

  /* 累积已接收长度 */
  pSerial_Device->Last_Rec_Data_Size_Cnt = Rec_Total;
}

#if USE_USB_CDC
/**
 * @brief CDC串口初始化
 *
 * @param pSerial_Port_Handle 串口设备句柄
 */
static void CDC_Init(void *pSerial_Port_Handle)
{
  SERIAL_PORT_HANDLE_Typedef_t *pSerial_Device = (SERIAL_PORT_HANDLE_Typedef_t *)pSerial_Port_Handle;
  USBD_HandleTypeDef *phUsbDeviceFS = (USBD_HandleTypeDef *)pSerial_Device->pSerial_Handle;

  /* 唤醒USB */
  USBD_Start(phUsbDeviceFS);
  HAL_GPIO_WritePin(USB_PULL_IO_GPIO_Port, USB_PULL_IO_Pin, GPIO_PIN_SET);
}

/**
 * @brief CDC串口反初始化
 *
 * @param pSerial_Port_Handle 串口设备句柄
 */
static void CDC_DeInit(void *pSerial_Port_Handle)
{
  SERIAL_PORT_HANDLE_Typedef_t *pSerial_Device = (SERIAL_PORT_HANDLE_Typedef_t *)pSerial_Port_Handle;
  USBD_HandleTypeDef *phUsbDeviceFS = (USBD_HandleTypeDef *)pSerial_Device->pSerial_Handle;

  /* 挂起USB */
  USBD_Stop(phUsbDeviceFS);
  HAL_GPIO_WritePin(USB_PULL_IO_GPIO_Port, USB_PULL_IO_Pin, GPIO_PIN_RESET);
}

/**
 * @brief CDC发送数据
 *
 * @param pSerial_Port_Handle 串口设备句柄
 * @param Data 数据，阻塞时间为0时不可使用局部变量
 * @param Len 数据长度
 * @param BlockTime 阻塞时间ms
 * @return true 发送正常
 * @return false 发送错误
 */
static bool CDC_Send_Data_Start(void *pSerial_Port_Handle, const uint8_t *Data, uint32_t Len, uint32_t BlockTime)
{
  (void)(BlockTime);
  SERIAL_PORT_HANDLE_Typedef_t *pSerial_Device = (SERIAL_PORT_HANDLE_Typedef_t *)pSerial_Port_Handle;

  /* 数据发送 */
  if(USBD_OK != CDC_Transmit_FS((uint8_t *)Data, Len, pSerial_Device->INepNum))
  {
    return false;
  }
  return true;
}

/**
 * @brief CDC接收数据
 *
 * @param pSerial_Port_Handle 串口设备句柄
 * @param Buffer 接收缓冲区
 * @param Len 接收长度
 * @param BlockTime 阻塞时间，暂未使用保留
 * @return true 接收正常
 * @return false 接收失败
 */
static bool CDC_Receive_Data_Start(void *pSerial_Port_Handle, uint8_t *Buffer, uint32_t Len, uint32_t BlockTime)
{
  (void)(BlockTime);
  SERIAL_PORT_HANDLE_Typedef_t *pSerial_Device = (SERIAL_PORT_HANDLE_Typedef_t *)pSerial_Port_Handle;
  USBD_HandleTypeDef *phUsbDeviceFS = (USBD_HandleTypeDef *)pSerial_Device->pSerial_Handle;

  USBD_CDC_SetRxBuffer(phUsbDeviceFS, Buffer);
  if(USBD_OK != USBD_CDC_ReceivePacket(phUsbDeviceFS, pSerial_Device->OUTepNum))
  {
    return false;
  }
  return true;
}

/**
 * @brief CDC接收完成中断
 *
 * @param pSerial_Port_Handle 串口设备句柄
 * @param Data
 * @param Len
 */
static void CDC_Receive_Complete_IT(void *pSerial_Port_Handle, const uint8_t *Data, uint32_t Len)
{
  SERIAL_PORT_HANDLE_Typedef_t *pSerial_Device = (SERIAL_PORT_HANDLE_Typedef_t *)pSerial_Port_Handle;

  /* 计算本次长度 */
  uint32_t Rec_Len = Len;

  /* 将数据记录至环形区 */
  CQ_putData(pSerial_Device->pCQ_Handle, Data, Rec_Len);

  /* 数据回显 */
#if USE_LOOPBACK
  pSerial_Device->pSend_Data_Start(pSerial_Port_Handle, Data, Rec_Len, 0xFFFF);
#endif
}

/**
 * @brief 获取CDC空闲状态
 *
 * @param pSerial_Port_Handle 串口设备句柄
 * @return SERIAL_STATE_Typedef_t 串口状态
 */
static SERIAL_STATE_Typedef_t CDC_Get_Idel_State(void *pSerial_Port_Handle)
{
  uint32_t State = (uint32_t)SERIAL_PORT_READY;

  USBD_HandleTypeDef *phUsbDeviceFS = (USBD_HandleTypeDef *)pSerial_Port_Handle;
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)(phUsbDeviceFS->pClassData);
  if(hcdc->TxState == 0)
  {
    State |= SERIAL_PORT_TX_IDEL;
  }
  else
  {
    State |= SERIAL_PORT_TX_BUSY;
  }
  if(hcdc->RxState == 0)
  {
    State |= SERIAL_PORT_RX_IDEL;
  }
  else
  {
    State |= SERIAL_PORT_RX_BUSY;
  }

  return (SERIAL_STATE_Typedef_t)State;
}

#endif
/** Public application code --------------------------------------------------*/
/*******************************************************************************
*
*       Public code
*
********************************************************************************
*/
#if defined (__CC_ARM) ||  (__GNUC__)
  #pragma import(__use_no_semihosting) //确保没有从 C 库链接使用半主机的函数
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
  #ifdef USE_LINK_SMALL_PRINTF
    #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
    #define GETCHAR_PROTOTYPE int __io_getchar(void)
  #else
    #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
    #define GETCHAR_PROTOTYPE int fgetc(FILE *f)
  #endif
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
  #define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif /* __GNUC__ */

struct __FILE
{
  int handle;
};

/* FILE is typedef ’ d in stdio.h. */
FILE __stdout;

/**
 * @brief 重写
 *
 * @param ch
 */
void _ttywrch(int ch)
{
  ch = ch;
}

/**
 * @brief 退出接口
 *
 * @param x
 */
void _sys_exit(int x) //定义 _sys_exit() 以避免使用半主机模式
{
  (void)x;
}

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(SERIAL_PORT_DEBUG_UART, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

/**
 * @brief 串口接收事件中断
 *
 * @param huart 串口句柄
 * @param Size 接收数据大小
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  Serial_Port_IT_CallBack(huart, Size);
}

/**
 * @brief 串口半接收完成中断
 *
 * @param huart 串口句柄
 */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  Serial_Port_Half_IT_CallBack(huart);
}

/**
 * @brief 串口接收完成中断
 *
 * @param huart 串口句柄
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  Serial_Port_Complete_IT_CallBack(huart);
}

/**
 * @brief 串口半发送完成中断
 *
 * @param huart 串口句柄
 */
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  (void)(huart);
}

/**
 * @brief 串口发送完成中断
 *
 * @param huart 串口句柄
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  (void)(huart);
}

/**
 * @brief 串口半中断
 *
 * @param Arg 串口句柄参数
 */
void Serial_Port_Half_IT_CallBack(void *Arg)
{
  for(int i = 0; i < SERIAL_PORT_NUM_MAX; i++)
  {
    if(Arg == Serial_Device_List[i].pSerial_Handle)
    {
      if(NULL == Serial_Device_List[i].pReceive_Half_IT)
      {
        return;
      }
      Serial_Device_List[i].pReceive_Half_IT(&Serial_Device_List[i]);
    }
  }
}

/**
 * @brief 串口完成中断
 *
 * @param Arg 串口句柄参数
 */
void Serial_Port_Complete_IT_CallBack(void *Arg)
{
  for(int i = 0; i < SERIAL_PORT_NUM_MAX; i++)
  {
    if(Arg == Serial_Device_List[i].pSerial_Handle)
    {
      if(NULL == Serial_Device_List[i].pReceive_Complete_IT)
      {
        return;
      }
      Serial_Device_List[i].pReceive_Complete_IT(&Serial_Device_List[i]);
    }
  }
}

/**
 * @brief 串口中断
 *
 * @param Arg 串口句柄参数
 * @param Size 数据大小字节
 */
void Serial_Port_IT_CallBack(void *Arg, uint32_t Size)
{
  for(int i = 0; i < SERIAL_PORT_NUM_MAX; i++)
  {
    if(Arg == Serial_Device_List[i].pSerial_Handle)
    {
      /* 检测到半完成中断类型 */
      if(Size == Serial_Device_List[i].Buffer_Size / 2)
      {
        if(NULL == Serial_Device_List[i].pReceive_Half_IT)
        {
          return;
        }
        Serial_Device_List[i].pReceive_Half_IT(&Serial_Device_List[i]);
        return;
      }
      /* 检测到完成中断类型 */
      if(Size == Serial_Device_List[i].Buffer_Size)
      {
        if(NULL == Serial_Device_List[i].pReceive_Complete_IT)
        {
          return;
        }
        Serial_Device_List[i].pReceive_Complete_IT(&Serial_Device_List[i]);
        return;
      }
      /* 空闲 or 其他 */
      if(NULL == Serial_Device_List[i].pSerial_IT)
      {
        return;
      }
      Serial_Device_List[i].pSerial_IT(&Serial_Device_List[i]);
      return;
    }
  }
}

/**
 * @brief CDC串口完成中断
 *
 * @param Arg CDC句柄参数
 * @param Data 数据
 * @param Len 数据长度
 * @param EPNum 端口号
 */
void Serial_Port_Complete_CDC_IT_CallBack(void *Arg, const uint8_t *Data, uint32_t Len, uint8_t EPNum)
{
  for(int i = 0; i < SERIAL_PORT_NUM_MAX; i++)
  {
    if(Arg == Serial_Device_List[i].pSerial_Handle && EPNum == Serial_Device_List[i].OUTepNum)
    {
      if(NULL == Serial_Device_List[i].pSerial_Rec_IT)
      {
        return;
      }
      Serial_Device_List[i].pSerial_Rec_IT(&Serial_Device_List[i], Data, Len);
    }
  }
}

/**
 * @brief 设置串口波特率
 *
 * @param Serial_Num 串口号
 * @param BaudRate 波特率
 */
void Serial_Port_Set_BaudRate(SERIAL_PORT_SERIAL_NUM_Typedef_t Serial_Num, uint32_t BaudRate)
{
  /* 检测是否有设置接口 */
  if(NULL == Serial_Device_List[Serial_Num].pSet_BaudRate)
  {
    return;
  }
  Serial_Device_List[Serial_Num].pSet_BaudRate(&Serial_Device_List[Serial_Num], BaudRate);
}

/**
 * @brief 获取环形句柄
 *
 * @param Serial_Num 串口号
 * @return CQ_handleTypeDef* 环形句柄
 */
CQ_handleTypeDef *Serial_Port_Get_CQ(SERIAL_PORT_SERIAL_NUM_Typedef_t Serial_Num)
{
  return Serial_Device_List[Serial_Num].pCQ_Handle;
}

/**
 * @brief 获取串口空闲状态
 *
 * @param Serial_Num 串口号
 * @return SERIAL_STATE_Typedef_t 状态
 */
SERIAL_STATE_Typedef_t Serial_Port_Get_Idel_State(SERIAL_PORT_SERIAL_NUM_Typedef_t Serial_Num)
{
  return Serial_Device_List[Serial_Num].pGet_Serial_State(&Serial_Device_List[Serial_Num]);
}

/**
 * @brief 串口数据发送
 *
 * @param Serial_Num 串口号
 * @param Data 数据
 * @param Len 数据长度
 * @param Block_Time 阻塞时间
 * @return true 调用发送成功
 * @return false 调用发送失败
 */
bool Serial_Port_Transmit_Data(SERIAL_PORT_SERIAL_NUM_Typedef_t Serial_Num, const uint8_t *Data, uint32_t Len, uint32_t Block_Time)
{
  /* 检测是否存在发送接口 */
  if(NULL == Serial_Device_List[Serial_Num].pSend_Data_Start)
  {
    return false;
  }
  return Serial_Device_List[Serial_Num].pSend_Data_Start(&Serial_Device_List[Serial_Num], Data, Len, Block_Time);
}

/**
 * @brief 串口反初始化
 *
 */
void Serial_Port_DeInit(void)
{
  for(int i = 0; i < SERIAL_PORT_NUM_MAX; i++)
  {
    if(NULL != Serial_Device_List[i].pDeInit)
    {
      Serial_Device_List[i].pDeInit(&Serial_Device_List[i]);
    }
  }
}

/**
 * @brief 串口初始化
 *
 */
void Serial_Port_Init(void)
{
  /* 初始化串口 */
  SERIAL_PORT_HANDLE_Typedef_t *pSerial_Device = &Serial_Device_List[SERIAL_PORT_UART_2];
  pSerial_Device->pSerial_Handle = &huart2;

  pSerial_Device->pCQ_Handle = cb_create(1024);
  if(NULL == pSerial_Device->pCQ_Handle)
  {
    printf("Serial Port Uart 2 Malloc Buf Faild.\r\n");
    return;
  }
  pSerial_Device->pReceive_Buffer = Uart2_DMA_Buf;//(uint8_t *)SERIAL_PORT_MALLOC(1024);
  pSerial_Device->Buffer_Size = 16;

  pSerial_Device->pStart = UART_Start;
  pSerial_Device->pInit = UART_Init;
  pSerial_Device->pDeInit = UART_DeInit;

  pSerial_Device->pSend_Data_Start = UART_Send_Data_Start;
  pSerial_Device->pReceive_Data_Start = UART_Receive_Data_Start;
  pSerial_Device->pGet_Serial_State = UART_Get_Idel_State;
  pSerial_Device->pSet_BaudRate = UART_Set_BaudRate;

  pSerial_Device->pSemaphoreId = NULL;
  pSerial_Device->pLock_CallBack = NULL;
  pSerial_Device->pUnLock_CallBack = NULL;

  pSerial_Device->pReceive_Half_IT = UART_Receive_Half_IT;
  pSerial_Device->pReceive_Complete_IT = UART_Receive_Complete_IT;
  pSerial_Device->pSerial_IT = UART_Idel_IT;
  pSerial_Device->Last_Rec_Data_Size_Cnt = 0;

  /* 启动接收 */
  pSerial_Device->pStart(pSerial_Device);

  /* 初始化CDC */
#if USE_USB_CDC
  pSerial_Device = &Serial_Device_List[SERIAL_PORT_USB_CDC_0];
  pSerial_Device->pSerial_Handle = &hUsbDeviceFS;

  pSerial_Device->pCQ_Handle = cb_create(1024);
  if(NULL == pSerial_Device->pCQ_Handle)
  {
    printf("Serial Port Usb 0 Malloc Buf Faild.\r\n");
    return;
  }
  pSerial_Device->pReceive_Buffer = (uint8_t *)UserRxBufferFS;
  pSerial_Device->Buffer_Size = APP_RX_DATA_SIZE;

  pSerial_Device->pStart = NULL;
  pSerial_Device->pInit = CDC_Init;
  pSerial_Device->pDeInit = CDC_DeInit;

  pSerial_Device->pSend_Data_Start = CDC_Send_Data_Start;
  pSerial_Device->pReceive_Data_Start = CDC_Receive_Data_Start;
  pSerial_Device->pGet_Serial_State = CDC_Get_Idel_State;
  pSerial_Device->pSet_BaudRate = NULL;

  pSerial_Device->pSemaphoreId = NULL;
  pSerial_Device->pLock_CallBack = NULL;
  pSerial_Device->pUnLock_CallBack = NULL;

  pSerial_Device->pReceive_Half_IT = NULL;
  pSerial_Device->pReceive_Complete_IT = NULL;
  pSerial_Device->pSerial_IT = NULL;
  pSerial_Device->Last_Rec_Data_Size_Cnt = 0;

  pSerial_Device->INepNum = CDC_IN_EP;
  pSerial_Device->OUTepNum = CDC_OUT_EP;
  pSerial_Device->pSerial_Rec_IT = CDC_Receive_Complete_IT;
#endif
}

#ifdef __cplusplus ///<end extern c
}
#endif
/******************************** End of file *********************************/
