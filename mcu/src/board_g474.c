/* G474 pin/clock setup derived from the operator supplied v9 baseline.
 * This is the production entry point. It contains no programmer or bench API. */
#include "stm32g4xx_hal.h"
#include "drive.h"
#include "mq.h"
#include "wire.h"
#include <string.h>

static UART_HandleTypeDef uart;
static FDCAN_HandleTypeDef can;
static Drive drive;
static MQ queue;
static Wire wire;
static uint8_t rx_byte;
static volatile bool uart_tx_busy,uart_error;

void _init(void){}
void _fini(void){}
void SysTick_Handler(void){HAL_IncTick();}
void LPUART1_IRQHandler(void){HAL_UART_IRQHandler(&uart);}

static void check(HAL_StatusTypeDef value){
  if(value==HAL_OK)return;
  /* No CAN traffic keeps the independent motor-driver watchdog alive here. */
  for(;;){}
}
void HAL_MspInit(void){__HAL_RCC_SYSCFG_CLK_ENABLE();__HAL_RCC_PWR_CLK_ENABLE();HAL_PWREx_DisableUCPDDeadBattery();}
static void clock_init(void){
  RCC_OscInitTypeDef o={0};RCC_ClkInitTypeDef c={0};
  o.OscillatorType=RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  o.HSIState=RCC_HSI_ON;o.HSI48State=RCC_HSI48_ON;o.HSICalibrationValue=RCC_HSICALIBRATION_DEFAULT;o.PLL.PLLState=RCC_PLL_NONE;
  check(HAL_RCC_OscConfig(&o));
  c.ClockType=RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  c.SYSCLKSource=RCC_SYSCLKSOURCE_HSI;c.AHBCLKDivider=RCC_SYSCLK_DIV1;c.APB1CLKDivider=RCC_HCLK_DIV1;c.APB2CLKDivider=RCC_HCLK_DIV1;
  check(HAL_RCC_ClockConfig(&c,FLASH_LATENCY_0));
}
void HAL_UART_MspInit(UART_HandleTypeDef *h){
  if(h->Instance!=LPUART1)return;
  __HAL_RCC_GPIOB_CLK_ENABLE();__HAL_RCC_GPIOC_CLK_ENABLE();__HAL_RCC_LPUART1_CONFIG(RCC_LPUART1CLKSOURCE_HSI);
  __HAL_RCC_LPUART1_CLK_ENABLE();__HAL_RCC_LPUART1_FORCE_RESET();__HAL_RCC_LPUART1_RELEASE_RESET();
  GPIO_InitTypeDef g={0};g.Mode=GPIO_MODE_AF_PP;g.Speed=GPIO_SPEED_FREQ_HIGH;g.Alternate=GPIO_AF8_LPUART1;
  g.Pin=GPIO_PIN_1;g.Pull=GPIO_NOPULL;HAL_GPIO_Init(GPIOC,&g);
  g.Pin=GPIO_PIN_0;g.Pull=GPIO_PULLUP;HAL_GPIO_Init(GPIOC,&g);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);g.Pin=GPIO_PIN_1;g.Mode=GPIO_MODE_OUTPUT_PP;g.Pull=GPIO_NOPULL;HAL_GPIO_Init(GPIOB,&g);
}
static void uart_init(void){
  uart.Instance=LPUART1;uart.Init.BaudRate=115200;uart.Init.WordLength=UART_WORDLENGTH_8B;uart.Init.StopBits=UART_STOPBITS_1;
  uart.Init.Parity=UART_PARITY_NONE;uart.Init.Mode=UART_MODE_TX_RX;uart.Init.HwFlowCtl=UART_HWCONTROL_NONE;
  uart.Init.OverSampling=UART_OVERSAMPLING_16;uart.Init.OneBitSampling=UART_ONE_BIT_SAMPLE_DISABLE;
  uart.Init.ClockPrescaler=UART_PRESCALER_DIV1;uart.AdvancedInit.AdvFeatureInit=UART_ADVFEATURE_NO_INIT;
  check(HAL_RS485Ex_Init(&uart,UART_DE_POLARITY_HIGH,2,2));
  GPIO_InitTypeDef g={0};g.Pin=GPIO_PIN_1;g.Mode=GPIO_MODE_AF_PP;g.Pull=GPIO_NOPULL;g.Speed=GPIO_SPEED_FREQ_HIGH;g.Alternate=GPIO_AF12_LPUART1;
  HAL_GPIO_Init(GPIOB,&g);
  HAL_NVIC_DisableIRQ(LPUART1_IRQn);HAL_NVIC_SetPriority(LPUART1_IRQn,5,0);
  check(HAL_UART_Receive_IT(&uart,&rx_byte,1));
  HAL_NVIC_ClearPendingIRQ(LPUART1_IRQn);HAL_NVIC_EnableIRQ(LPUART1_IRQn);
}
/* 中断接收串口字节并重新挂接接收；协议处理留在主循环。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h){
  if(h!=&uart)return;
  /* Preserve every received byte, including a complete STOP across TX/TC.
     The wire parser consumes a whole telemetry echo without dispatching it. */
  wire_rx_byte(&wire,rx_byte,HAL_GetTick());
  if(HAL_UART_Receive_IT(&uart,&rx_byte,1)!=HAL_OK){uart_error=true;wire_transport_error(&wire);}
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *h){
  if(h==&uart)uart_tx_busy=false; /* HAL calls this after TC; hardware DE is now released. */
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *h){
  if(h!=&uart)return;
  uart_error=true;wire_transport_error(&wire);
  if(h->RxState==HAL_UART_STATE_READY&&HAL_UART_Receive_IT(&uart,&rx_byte,1)!=HAL_OK)uart_error=true;
}
static bool wire_tx_ready(void *u){(void)u;return !uart_tx_busy&&uart.gState==HAL_UART_STATE_READY;}
static uint32_t wire_now_ms(void *u){(void)u;return HAL_GetTick();}
static uint32_t wire_critical_enter(void *u){
  (void)u;uint32_t saved=__get_PRIMASK();__disable_irq();__DMB();return saved;
}
static void wire_critical_exit(void *u,uint32_t saved){
  (void)u;__DMB();__set_PRIMASK(saved);
}
static bool wire_rx_pending(void *u){
  (void)u;
  /* IRQ masking does not stop the peripheral. Observe pending hardware as well
     as the ISR ring; never consume RDR or clear flags in this commit gate. */
  uint32_t status=uart.Instance->ISR;
  uint32_t errors=status&(USART_ISR_PE|USART_ISR_FE|USART_ISR_NE|USART_ISR_ORE|USART_ISR_RTOF);
  if(errors){uart_error=true;wire_transport_error(&wire);}
  return errors||(status&(USART_ISR_RXNE_RXFNE|USART_ISR_BUSY));
}
static void uart_service_fault(uint32_t now){
  uint32_t saved=wire_critical_enter(NULL);(void)wire_rx_pending(NULL);
  /* Publish the UART cause before the derived driver failure. No CAN send is
     performed by either of these state transitions. */
  if(wire_service_fault(&wire,now)||uart_error)drive_uart_fault(&drive,now);
  wire_critical_exit(NULL,saved);
}
static bool wire_tx_start(void *u,const uint8_t *bytes,uint16_t count,uint32_t expected_tick){
  /* Called inside Wire's short commit transaction. A final peripheral read
     also covers RX which became pending while checking the software ring. */
  (void)u;if(!wire_tx_ready(NULL)||HAL_GetTick()!=expected_tick||wire_rx_pending(NULL))return false;
  uart_tx_busy=true;
  if(HAL_UART_Transmit_IT(&uart,(uint8_t *)bytes,count)==HAL_OK)return true;
  uart_tx_busy=false;uart_error=true;wire_transport_error(&wire);return false;
}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *h){
  if(h->Instance!=FDCAN3)return;
  RCC_PeriphCLKInitTypeDef c={0};
  c.PeriphClockSelection=RCC_PERIPHCLK_FDCAN;c.FdcanClockSelection=RCC_FDCANCLKSOURCE_PCLK1;check(HAL_RCCEx_PeriphCLKConfig(&c));
  __HAL_RCC_FDCAN_CLK_ENABLE();__HAL_RCC_FDCAN_FORCE_RESET();__HAL_RCC_FDCAN_RELEASE_RESET();__HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef g={0};g.Pin=GPIO_PIN_3|GPIO_PIN_4;g.Mode=GPIO_MODE_AF_PP;g.Pull=GPIO_NOPULL;g.Speed=GPIO_SPEED_FREQ_VERY_HIGH;g.Alternate=GPIO_AF11_FDCAN3;HAL_GPIO_Init(GPIOB,&g);
}
static void can_init(void){
  can.Instance=FDCAN3;can.Init.ClockDivider=FDCAN_CLOCK_DIV1;can.Init.FrameFormat=FDCAN_FRAME_CLASSIC;can.Init.Mode=FDCAN_MODE_NORMAL;
  can.Init.AutoRetransmission=DISABLE;can.Init.TransmitPause=DISABLE;can.Init.ProtocolException=DISABLE;
  can.Init.NominalPrescaler=2;can.Init.NominalSyncJumpWidth=1;can.Init.NominalTimeSeg1=13;can.Init.NominalTimeSeg2=2;
  can.Init.DataPrescaler=1;can.Init.DataSyncJumpWidth=1;can.Init.DataTimeSeg1=1;can.Init.DataTimeSeg2=1;
  can.Init.StdFiltersNbr=1;can.Init.ExtFiltersNbr=0;can.Init.TxFifoQueueMode=FDCAN_TX_FIFO_OPERATION;check(HAL_FDCAN_Init(&can));
  FDCAN_FilterTypeDef f={0};f.IdType=FDCAN_STANDARD_ID;f.FilterIndex=0;f.FilterType=FDCAN_FILTER_MASK;f.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;f.FilterID1=0x581;f.FilterID2=0x7ff;
  check(HAL_FDCAN_ConfigFilter(&can,&f));check(HAL_FDCAN_ConfigGlobalFilter(&can,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE));check(HAL_FDCAN_Start(&can));
  check(HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN)==16000000u?HAL_OK:HAL_ERROR);
}
static bool can_ready(void *u){(void)u;return HAL_FDCAN_GetTxFifoFreeLevel(&can)>0;}
static bool can_send(void *u,const uint8_t bytes[8]){
  (void)u;FDCAN_TxHeaderTypeDef h={0};h.Identifier=0x601;h.IdType=FDCAN_STANDARD_ID;h.TxFrameType=FDCAN_DATA_FRAME;h.DataLength=FDCAN_DLC_BYTES_8;
  h.ErrorStateIndicator=FDCAN_ESI_ACTIVE;h.BitRateSwitch=FDCAN_BRS_OFF;h.FDFormat=FDCAN_CLASSIC_CAN;h.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
  uint32_t saved=wire_critical_enter(NULL);(void)wire_rx_pending(NULL);
  /* A UART event can arrive after the main-loop fault service. Do not commit
     another ARM/target transaction before that event has entered the STOP
     state machine. Already stopping traffic keeps its original proof rules. */
  bool allowed=!wire.transport_fault||drive.uart_failed||drive.view.state==MQ_DRIVE_STOPPING;
  bool sent=allowed&&HAL_FDCAN_AddMessageToTxFifoQ(&can,&h,(uint8_t *)bytes)==HAL_OK;
  wire_critical_exit(NULL,saved);return sent;
}
static uint32_t can_errors(void *u){
  (void)u;uint32_t ecr=can.Instance->ECR&0x7fffu;
  return ((can.Instance->PSR&FDCAN_PSR_BO)?1u:0u)|(ecr<<1);
}
/* CAN 数据交给驱动层并保留实际接收时间。 */
static void can_receive(void){
  unsigned budget=16;
  while(budget--&&HAL_FDCAN_GetRxFifoFillLevel(&can,FDCAN_RX_FIFO0)){
    FDCAN_RxHeaderTypeDef h;uint8_t bytes[64];
    if(HAL_FDCAN_GetRxMessage(&can,FDCAN_RX_FIFO0,&h,bytes)!=HAL_OK)break;
    if(h.IdType==FDCAN_STANDARD_ID&&h.RxFrameType==FDCAN_DATA_FRAME&&h.DataLength==FDCAN_DLC_BYTES_8)
      drive_receive_can(&drive,h.Identifier,8,bytes,HAL_GetTick());
  }
}
static void watchdog_init(void){
  __HAL_DBGMCU_UNFREEZE_IWDG();__HAL_RCC_LSI_ENABLE();uint32_t start=HAL_GetTick();
  while(!__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY)){if((uint32_t)(HAL_GetTick()-start)>100u)check(HAL_ERROR);}
  IWDG->KR=0xcccc;IWDG->KR=0x5555;IWDG->PR=3;IWDG->RLR=999;IWDG->WINR=0xfff;
  start=HAL_GetTick();while(IWDG->SR){IWDG->KR=0xaaaa;if((uint32_t)(HAL_GetTick()-start)>100u)check(HAL_ERROR);}IWDG->KR=0xaaaa;
}
/* 主循环协调 CAN、里程计、协议、驱动和看门狗。 */
int main(void){
  __disable_irq();SCB->VTOR=FLASH_BASE;SysTick->CTRL=0;
  for(unsigned i=0;i<8;i++){NVIC->ICER[i]=0xffffffffu;NVIC->ICPR[i]=0xffffffffu;}__DSB();__ISB();
  HAL_Init();__enable_irq();clock_init();
  __HAL_RCC_RNG_CONFIG(RCC_RNGCLKSOURCE_HSI48);__HAL_RCC_RNG_CLK_ENABLE();
  RNG_HandleTypeDef rng={0};rng.Instance=RNG;uint32_t nonce[2]={0};bool rng_ok=HAL_RNG_Init(&rng)==HAL_OK;
  if(rng_ok)rng_ok=HAL_RNG_GenerateRandomNumber(&rng,&nonce[0])==HAL_OK&&HAL_RNG_GenerateRandomNumber(&rng,&nonce[1])==HAL_OK;
  uint64_t boot=((uint64_t)nonce[0]<<32)|nonce[1];
  can_init();
  static const DriveBusOps bus_ops={can_send,can_ready,can_errors};drive_init(&drive,&bus_ops,NULL,HAL_GetTick());
  mq_init(&queue,rng_ok?boot:0,drive_ops(),&drive);
  static const WireOps wire_ops={wire_tx_ready,wire_tx_start,wire_now_ms,
    wire_critical_enter,wire_critical_exit,wire_rx_pending};
  wire_init(&wire,&queue,&wire_ops,NULL,HAL_GetTick());
  uart_init();watchdog_init();
  for(;;){
    uint32_t now=HAL_GetTick();
    uart_service_fault(now);
    can_receive();
    /* RX completion uses HAL_GetTick(). Refresh after draining CAN so a
       SysTick during RX cannot turn a new sample's age into UINT32_MAX. */
    now=HAL_GetTick();drive_poll(&drive,now);
    uart_service_fault(now);
    MQSample sample;
    while(drive_pop_sample(&drive,&sample))mq_sample(&queue,&sample);
    uart_service_fault(now);
    mq_tick(&queue,now);wire_poll(&wire,now);IWDG->KR=0xaaaa;HAL_Delay(1);
  }
}
