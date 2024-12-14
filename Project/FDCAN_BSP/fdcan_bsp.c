#include "fdcan_bsp.h"


/**
 * @brief FDCAN1 初始化，关联FIFO0
 * @param hfdcan hfdcan1
 * @return uint8_t 成功返回0
 */

uint8_t FDCAN1_Init(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef RXFilter;

    //配置RX滤波器
    RXFilter.IdType = FDCAN_STANDARD_ID;              /*标准ID*/
    RXFilter.FilterIndex = 0;                         /*滤波器索引*/
    RXFilter.FilterType = FDCAN_FILTER_MASK;          /*过滤器类型，掩码模式*/
    RXFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;  /*关联过滤器到 RXFIFO0*/
    RXFilter.FilterID1 = 0x0000;                      /*32位ID*/
    RXFilter.FilterID2 = 0x0000;                      /*如果FDCAN配置为传统模式的话，这里是32位掩码*/
    /*滤波器初始化*/
    if (HAL_FDCAN_ConfigFilter(hfdcan, &RXFilter) != HAL_OK)
    {
        return 2;
    }
    
//    RXFilter.IdType = FDCAN_EXTENDED_ID;             /*拓展ID*/
//    RXFilter.FilterIndex = 2;                        /*滤波器索引*/
//    RXFilter.FilterType = FDCAN_FILTER_MASK;         /*过滤器类型，掩码模式*/
//    RXFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; /*关联过滤器到 RXFIFO0*/
//    RXFilter.FilterID1 = 0x00000000;                 /*32位ID*/
//    RXFilter.FilterID2 = 0x00000000;                 /*如果FDCAN配置为传统模式的话，这里是32位掩码*/
//    /*滤波器初始化*/
//    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &RXFilter) != HAL_OK)
//    {                                              
//        return 2;                                    /*这里是拓展帧的配置*/
//    }                                               

    /* 设置FDCAN1滤波器0全局配置  */
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, DISABLE, DISABLE) != HAL_OK)
    {
        Error_Handler();
    }
	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_BUS_OFF, 0) != HAL_OK)               //激活FDCAN1的总线关闭中断
	{
		Error_Handler();
	}
	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)  //打开FDCAN1的接收中断
	{
		Error_Handler();
	}

    
	HAL_FDCAN_Start(&hfdcan1);                          /*开启FDCAN*/

	return 0;
}

/**
 * @brief FDCAN2 初始化，关联FIFO1
 * @param hfdcan hfdcan2
 * @return uint8_t 成功返回0
 */

uint8_t FDCAN2_Init(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef RXFilter;

    //配置RX滤波器
    RXFilter.IdType = FDCAN_STANDARD_ID;              /*标准ID*/
    RXFilter.FilterIndex = 0;                         /*滤波器索引*/
    RXFilter.FilterType = FDCAN_FILTER_MASK;          /*过滤器类型，掩码模式*/
    RXFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;  /*关联过滤器到 RXFIFO1*/
    RXFilter.FilterID1 = 0x0000;                      /*32位ID*/
    RXFilter.FilterID2 = 0x0000;                      /*如果FDCAN配置为传统模式的话，这里是32位掩码*/
    /*滤波器初始化*/
    if (HAL_FDCAN_ConfigFilter(hfdcan, &RXFilter) != HAL_OK)
    {
        return 2;
    }
    
//    RXFilter.IdType = FDCAN_EXTENDED_ID;             /*拓展ID*/
//    RXFilter.FilterIndex = 2;                        /*滤波器索引*/
//    RXFilter.FilterType = FDCAN_FILTER_MASK;         /*过滤器类型，掩码模式*/
//    RXFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1; /*关联过滤器到 RXFIFO1*/
//    RXFilter.FilterID1 = 0x00000000;                 /*32位ID*/
//    RXFilter.FilterID2 = 0x00000000;                 /*如果FDCAN配置为传统模式的话，这里是32位掩码*/
//    /*滤波器初始化*/
//    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &RXFilter) != HAL_OK)
//    {                                              
//        return 2;                                    /*这里是拓展帧的配置*/
//    }                                               

    /* 设置FDCAN2滤波器0全局配置  */
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, DISABLE, DISABLE) != HAL_OK)
    {
        Error_Handler();
    }
	if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_BUS_OFF, 0) != HAL_OK)               //激活FDCAN2的总线关闭中断
	{
		Error_Handler();
	}
	if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)  //打开FDCAN2的接收中断
	{
		Error_Handler();
	}

    
	HAL_FDCAN_Start(&hfdcan2);                          /*开启FDCAN*/

	return 0;
}

/**
 * @brief FDCAN3 初始化，关联FIFO0
 * @param hfdcan hfdcan3
 * @return uint8_t 成功返回0
 */

uint8_t FDCAN3_Init(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef RXFilter;

    //配置RX滤波器
    RXFilter.IdType = FDCAN_STANDARD_ID;              /*标准ID*/
    RXFilter.FilterIndex = 0;                         /*滤波器索引*/
    RXFilter.FilterType = FDCAN_FILTER_MASK;          /*过滤器类型，掩码模式*/
    RXFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;  /*关联过滤器到 RXFIFO0*/
    RXFilter.FilterID1 = 0x0000;                      /*32位ID*/
    RXFilter.FilterID2 = 0x0000;                      /*如果FDCAN配置为传统模式的话，这里是32位掩码*/
    /*滤波器初始化*/
    if (HAL_FDCAN_ConfigFilter(hfdcan, &RXFilter) != HAL_OK)
    {
        return 2;
    }
    
//    RXFilter.IdType = FDCAN_EXTENDED_ID;             /*拓展ID*/
//    RXFilter.FilterIndex = 2;                        /*滤波器索引*/
//    RXFilter.FilterType = FDCAN_FILTER_MASK;         /*过滤器类型，掩码模式*/
//    RXFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; /*关联过滤器到 RXFIFO0*/
//    RXFilter.FilterID1 = 0x00000000;                 /*32位ID*/
//    RXFilter.FilterID2 = 0x00000000;                 /*如果FDCAN配置为传统模式的话，这里是32位掩码*/
//    /*滤波器初始化*/
//    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &RXFilter) != HAL_OK)
//    {                                              
//        return 2;                                    /*这里是拓展帧的配置*/
//    }                                               

    /* 设置FDCAN3滤波器0全局配置  */
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, DISABLE, DISABLE) != HAL_OK)
    {
        Error_Handler();
    }
	if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_BUS_OFF, 0) != HAL_OK)               //激活FDCAN3的总线关闭中断
	{
		Error_Handler();
	}
	if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)  //打开FDCAN3的接收中断
	{
		Error_Handler();
	}

    
	HAL_FDCAN_Start(&hfdcan3);                          /*开启FDCAN*/

	return 0;
}
/**
 * @brief FDCAN发送函数
 * @param hfdcan hfdcan句柄
 * @param TxData 发送数据
 * @param StdId 标准帧ID
 * @param Length 长度 1~8
 * @return uint8_t 发送成功返回0
 */

uint8_t FDCAN_SendData(FDCAN_HandleTypeDef *hfdcan, uint8_t *TxData, uint32_t StdId, uint32_t Length)
{

    FDCAN_TxHeaderTypeDef TxHeader = {0};
    TxHeader.Identifier = StdId;             /*32位 ID*/
    TxHeader.IdType = FDCAN_STANDARD_ID;     /*标准ID*/
    TxHeader.TxFrameType = FDCAN_DATA_FRAME; /*数据帧*/
    TxHeader.DataLength = Length;      /*数据长度有专门的格式  FDCAN_DLC_BYTES_8*/
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;           /*关闭速率切换*/
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;            /*传统的CAN模式*/
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; /*无发送事件*/
    TxHeader.MessageMarker = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, TxData) != HAL_OK)
        return 1; //发送
    
	return 0;
}

uint8_t FDCAN_SendData_Ext(FDCAN_HandleTypeDef *hfdcan, uint8_t *TxData, uint32_t ExtId, uint32_t Length, uint32_t Data_type)
{

    FDCAN_TxHeaderTypeDef TxHeader = {0};
    TxHeader.Identifier = ExtId;             /*32位 ID*/
    TxHeader.IdType = FDCAN_EXTENDED_ID;     /*拓展ID*/
    TxHeader.TxFrameType = Data_type; /*帧类型*/
    TxHeader.DataLength = Length;      /*数据长度有专门的格式*/
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;   /*关闭速率切换*/
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;            /*传统的CAN模式*/
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; /*无发送事件*/
    TxHeader.MessageMarker = 0;
    
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, TxData) != HAL_OK)
        return 1; //发送
    return 0;
}

