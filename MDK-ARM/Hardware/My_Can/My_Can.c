#include "My_Can.h"
#include "fdcan.h"

extern AllFlag allflag;                //标志位变量
extern CANFD_Message CanFD_Message;    //CAN通信消息结构体
extern PID pid_m1;                     //PID参数结构体
extern Encoder_Struct encoder_str;     //编码器结构体

FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
FDCAN_FilterTypeDef sFilter;
uint8_t CANFD_TxData[8];
uint8_t CANFD_RxData[8];

uint16_t IDD;

void CANFD_Init_Config(void)
{
	//过滤器配置,一共有14个过滤器，根据需求开启
    /* ---------- 过滤器 0： ---------- */
    sFilter.IdType       = FDCAN_STANDARD_ID;         //使用标准帧
    sFilter.FilterIndex  = 0;                         // 第 0 个过滤器
    sFilter.FilterType   = FDCAN_FILTER_MASK;         // 掩码模式
    sFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;   // 放进 FIFO0
    sFilter.FilterID1    = 0x000;                      // 期望 ID  0x0F    000 0000 1111
    sFilter.FilterID2    = 0x000;                      // 掩码：           000 0000 1111
                                                      //掩码1对应的位置意味着要比对一致的，0代表通过
    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilter);

    /* ---------- 过滤器 1： ---------- */
//    sFilter.IdType       = FDCAN_STANDARD_ID;         //使用标准帧
//    sFilter.FilterIndex  = 1;                         // 第 1 个过滤器
//    sFilter.FilterType   = FDCAN_FILTER_MASK;         // 掩码模式
//    sFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;   // 放进 FIFO0
//    sFilter.FilterID1    = 0x00F;                      // 期望 ID    
//    sFilter.FilterID2    = 0x7FF;                      // 掩码：
//                                                     
//    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilter);

    /* 使能 FIFO0 新消息中断 */
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	
	//开启CAN
	HAL_FDCAN_Start(&hfdcan1);
}

/********************
CANFD数据发送：
	目前是经典CAN
	参数：ID，指令类型，目标值，方向
********************/
uint8_t CANFD_Send(uint8_t motor_id,uint8_t command_type,uint32_t target_value,uint8_t direction)
{
	//配置发送信息
	TxHeader.Identifier = motor_id;                        //设置标准帧ID为电机ID,0-7FF,这里是8位数据，取0-FF够用了
	TxHeader.IdType = FDCAN_STANDARD_ID;				   //使用标准帧
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;               //数据帧
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;               //数据长度为8字节
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;       //主动错误状态
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;                //不启用比特率切换
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;                 //经典CAN格式
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;      //不存储发送事件到事件FIFIO,默认不存储
	TxHeader.MessageMarker = 0x52;                         //消息标记,默认任意给值
	//TxHeader.TxEventFifoControl如果开启，会在发送中断里面将本帧时间戳等写入Tx Event FIFO，默认不存储
	//如果开启TxEventFifoControl，那么MessageMarker才有用，只是定义一个标签随意给值，方便让你知道是哪个有问题
	
	//发送数据填入
	CANFD_TxData[0] = command_type;                           //指令类型
	CANFD_TxData[1] = (target_value>>16)&0xFF;                //目标位置高字节  //目前只用了24位数据，有需要可以继续移位到32位
	CANFD_TxData[2] = (target_value>>8)&0xFF;                 //目标位置中字节 
	CANFD_TxData[3] = target_value&0xFF;                      //目标位置低字节
	CANFD_TxData[4] = direction;                          	  //方向
	CANFD_TxData[5] = 0x00;                               //预留高字节
	CANFD_TxData[6] = 0x00;                                //预留低字节
	CANFD_TxData[7] = 0x00;                                //预留字节
	
	//发送数据
	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&TxHeader,CANFD_TxData)!=HAL_OK) return 1;
	return 0;
}

/********************
CANFD数据接收：
	目前是经典CAN
********************/
void CANFD_ReceiveDate(uint8_t * recbuf,uint16_t Identifier,uint16_t len)
{
	//接收数据定义
	uint8_t command_type = 0;        //指令类型
	uint32_t target_value = 0;       //目标位置
	uint8_t direction = 0;           //方向
	float Control_Target = 0;        //目标位置(带方向)
	
	//检查数据长度
	if(len!=8) return ;
	
	//解析数据
	command_type = recbuf[0];
	target_value = (recbuf[1]<<16)|(recbuf[2]<<8)|recbuf[3];
	direction = recbuf[4];
	
	//方向处理
	if(direction==0) Control_Target = target_value;
	else if(direction==1) Control_Target = (-1.0f)*target_value;
	
	//根据电机ID和指令类型处理数据
	switch(Identifier)
	{
		case 0x10:   //识别到电机ID0
			CanFD_Message.command_type = command_type;
			CanFD_Message.direction = direction;    //用于接收对比，无其他作用，数据在前面以及确定好了
			CanFD_Message.target_value = target_value;
			//控制操作
			break;
		case 0x01:   //识别到电机ID1
			//控制操作
			break;
		case 0x02:   //识别到电机ID2
			//控制操作
			break;
		case 0x03:   //识别到电机ID3
			//控制操作
			break;
	}
}


//FIFO0新消息接收中断
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (RxFifo0ITs == FDCAN_IT_RX_FIFO0_NEW_MESSAGE)  //如果是FIFO0新消息
    {

        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, CANFD_RxData) == HAL_OK)
        {
            /* 只处理标准帧且长度为 8 */
            if (RxHeader.IdType == FDCAN_STANDARD_ID && RxHeader.DataLength == FDCAN_DLC_BYTES_8 && RxHeader.Identifier == 0x10)
            {
				//可以在这进行ID判断，符合的接收，但是用JLink看看不出来，因为在外面用CANFD_RxData接收了，可以自己定义一个变量看
				IDD = RxHeader.Identifier;
                CANFD_ReceiveDate(CANFD_RxData, RxHeader.Identifier, 8);
            }
        }
    }
}




//// 在错误回调中增加总线恢复逻辑
//void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
//{
//  // 简单恢复：任何错误都重启CAN
//  HAL_FDCAN_Stop(hfdcan);
//  HAL_Delay(10);
//  HAL_FDCAN_Start(hfdcan);
//  HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
//}

uint8_t erro_flag;

//Timer7的4KHz定时中断，用于CAN发送
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint16_t i=0;
  if (htim->Instance == TIM7)          // 250us 到
  { 
	  i++;
	  if(i>=4)   //1K发送频率
	  {
		  allflag.CANFD_Tx_flag=CANFD_Send(0x20,3,encoder_str.Shaft_Angle,0);  //0代表发送成功，1代表发送失败
		  if (allflag.CANFD_Tx_flag == 0);
		  else
		  {
			erro_flag++;
			HAL_FDCAN_Stop(&hfdcan1);
		  }
		  if(erro_flag>6)
		  {
			erro_flag=0;  
			CANFD_Init_Config();
		  }
	
		  i=0;
	  }
  }
}
