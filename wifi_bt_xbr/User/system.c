/****************************************Copyright (c)*************************
**                               ��Ȩ���� (C), 2015-2017, Ϳѻ�Ƽ�
**
**                                 http://www.tuya.com
**
**--------------�ļ���Ϣ-------------------------------------------------------
**��   ��   ��: system.c
**��        ��: bluetooth���ݴ�����
**ʹ �� ˵ �� : �û�������ĸ��ļ�ʵ������
**
**
**--------------��ǰ�汾�޶�---------------------------------------------------
** ��  ��: v1.0
** �ա���: 2017��5��3��
** �衡��: 1:����Ϳѻbluetooth�Խ�MCU_SDK
**
**-----------------------------------------------------------------------------
******************************************************************************/
#define SYSTEM_GLOBAL

#include "bluetooth.h"
//
//
/*
extern u8 idata groupaddr1 ;
extern u8 idata groupaddr2 ;
extern u8 idata groupaddr3 ;
extern u8 idata groupaddr4 ;
extern u8 idata groupaddr5 ;
extern u8 idata groupaddr6 ;
extern u8 idata groupaddr7 ;
extern u8 idata groupaddr8 ;
*/

extern u16 idata groupaddr[8];

void savevar(void);
extern const DOWNLOAD_CMD_S xdata download_cmd[];

/*****************************************************************************
�������� : set_bt_uart_byte
�������� : дbt_uart�ֽ�
������� : dest:��������ʵ��ַ;
           byte:д���ֽ�ֵ
���ز��� : д����ɺ���ܳ���
*****************************************************************************/
unsigned short set_bt_uart_byte(unsigned short dest, unsigned char byte)
{
  unsigned char *obj = (unsigned char *)bt_uart_tx_buf + DATA_START + dest;
  
  *obj = byte;
  dest += 1;
  
  return dest;
}
/*****************************************************************************
�������� : set_bt_uart_buffer
�������� : дbt_uart_buffer
������� : dest:Ŀ���ַ
           src:Դ��ַ
           len:���ݳ���
���ز��� : ��
*****************************************************************************/
unsigned short set_bt_uart_buffer(unsigned short dest, unsigned char *src, unsigned short len)
{
  unsigned char *obj = (unsigned char *)bt_uart_tx_buf + DATA_START + dest;
  
  my_memcpy(obj,src,len);
  
  dest += len;
  return dest;
}
/*****************************************************************************
�������� : bt_uart_write_data
�������� : ��bt uartд����������
������� : in:���ͻ���ָ��
           len:���ݷ��ͳ���
���ز��� : ��
*****************************************************************************/
static void bt_uart_write_data(unsigned char *in, unsigned short len)
{
  if((NULL == in) || (0 == len))
  {
    return;
  }
  
  while(len --)
  {
    uart_transmit_output(*in);
    in ++;
  }
}
/*****************************************************************************
�������� : get_check_sum
�������� : ����У���
������� : pack:����Դָ��
           pack_len:����У��ͳ���
���ز��� : У���
*****************************************************************************/
unsigned char get_check_sum(unsigned char *pack, unsigned short pack_len)
{
  unsigned short i;
  unsigned char check_sum = 0;
  
  for(i = 0; i < pack_len; i ++)
  {
    check_sum += *pack ++;
  }
  
  return check_sum;
}
/*****************************************************************************
�������� : bt_uart_write_frame
�������� : ��bt���ڷ���һ֡����
������� : fr_type:֡����
           len:���ݳ���
���ز��� : ��
*****************************************************************************/
void bt_uart_write_frame(unsigned char fr_type, unsigned short len)
{
  unsigned char check_sum = 0;
  
  bt_uart_tx_buf[HEAD_FIRST] = 0x55;
  bt_uart_tx_buf[HEAD_SECOND] = 0xaa;
  bt_uart_tx_buf[PROTOCOL_VERSION] = 0x00;
  bt_uart_tx_buf[FRAME_TYPE] = fr_type;
  bt_uart_tx_buf[LENGTH_HIGH] = len >> 8;
  bt_uart_tx_buf[LENGTH_LOW] = len & 0xff;
  
  len += PROTOCOL_HEAD;
  check_sum = get_check_sum((unsigned char *)bt_uart_tx_buf, len - 1);
  bt_uart_tx_buf[len - 1] = check_sum;
  //
  bt_uart_write_data((unsigned char *)bt_uart_tx_buf, len);
}


void bt_uart_mesh_write_frame(unsigned char fr_type, unsigned short len)
{
  unsigned char check_sum = 0;

len = len+2;

bt_uart_tx_buf[0] = 0x55;
bt_uart_tx_buf[1] = 0xaa;
bt_uart_tx_buf[2] = 0x00;
bt_uart_tx_buf[3] = fr_type;

bt_uart_tx_buf[4] = len >> 8;
bt_uart_tx_buf[5] = len & 0xff;
bt_uart_tx_buf[6] = 0XFF;
bt_uart_tx_buf[7] = 0xff;


len += PROTOCOL_HEAD;
check_sum = get_check_sum((unsigned char *)bt_uart_tx_buf, len - 1);
bt_uart_tx_buf[len - 1] = check_sum;
//
bt_uart_write_data((unsigned char *)bt_uart_tx_buf, len);


}

/*****************************************************************************
�������� : heat_beat_check
�������� : ���������
������� : ��
���ز��� : ��
*****************************************************************************/
static void heat_beat_check(void)
{
  unsigned char length = 0;
  static unsigned char mcu_reset_state = FALSE;
  
  if(FALSE == mcu_reset_state)
  {
    length = set_bt_uart_byte(length,FALSE);
    mcu_reset_state = TRUE;
  }
  else
  {
    length = set_bt_uart_byte(length,TRUE);
  }
  
  bt_uart_write_frame(HEAT_BEAT_CMD, length);
}
/*****************************************************************************
��������  : product_info_update
��������  : ��Ʒ��Ϣ�ϴ�
������� : ��
���ز��� : ��
*****************************************************************************/
static void product_info_update(void)
{
  unsigned char length = 0;
  
  length = set_bt_uart_buffer(length,(unsigned char *)PRODUCT_KEY,my_strlen((unsigned char *)PRODUCT_KEY));
  length = set_bt_uart_buffer(length,(unsigned char *)MCU_VER,my_strlen((unsigned char *)MCU_VER));
  
  bt_uart_write_frame(PRODUCT_INFO_CMD, length);
}
/*****************************************************************************
�������� : get_mcu_bt_mode
�������� : ��ѯmcu��bt�Ĺ���ģʽ
������� : ��
���ز��� : ��
*****************************************************************************/
static void get_mcu_bt_mode(void)
{
  unsigned char length = 0;
  
#ifdef BT_CONTROL_SELF_MODE                                   //ģ���Դ���
  length = set_bt_uart_byte(length, BT_STATE_KEY);
  length = set_bt_uart_byte(length, BT_RESERT_KEY);
#else                                                           
  //���账������
#endif
  
  bt_uart_write_frame(WORK_MODE_CMD, length);
}
/*****************************************************************************
�������� : get_update_dpid_index
�������� : �����ƶ�DPID�������е����
������� : dpid:dpid
���ز��� : index:dp���
*****************************************************************************/
static unsigned char get_dowmload_dpid_index(unsigned char dpid)
{
  unsigned char index;
  unsigned char total = get_download_cmd_total();
  
  for(index = 0; index < total; index ++)
  {
    if(download_cmd[index].dp_id == dpid)
    {
      break;
    }
  }
  
  return index;
}
/*****************************************************************************
�������� : data_point_handle
�������� : �·����ݴ���
������� : value:�·�����Դָ��
���ز��� : ret:�������ݴ�����
*****************************************************************************/
static unsigned char data_point_handle(const unsigned char value[])
{
  unsigned char dp_id,index;
  unsigned char dp_type;
  unsigned char ret;
  unsigned short dp_len;
  
  dp_id = value[0];
  dp_type = value[1];
  dp_len = value[2] * 0x100;
  dp_len += value[3];
  
  index = get_dowmload_dpid_index(dp_id);

  if(dp_type != download_cmd[index].dp_type)
  {
    //������ʾ
    return FALSE;
  }
  else
  {
    ret = dp_download_handle(dp_id,value + 4,dp_len);
  }
  
  return ret;
}
/*****************************************************************************
�������� : data_handle
�������� : ����֡����
������� : offset:������ʼλ
���ز��� : ��
*****************************************************************************/
void data_handle(unsigned short offset)
{
#ifdef SUPPORT_MCU_FIRM_UPDATE
  unsigned char *firmware_addr;
  static unsigned long firm_length;                                             //MCU�����ļ�����
  static unsigned char firm_update_flag;                                        //MCU������־
  unsigned long dp_len;
#else
  unsigned short dp_len;
#endif
  
  unsigned char ret;
  unsigned short i,total_len;
  unsigned char cmd_type = bt_uart_rx_buf[offset + FRAME_TYPE];
  
  switch(cmd_type)
  {
  case HEAT_BEAT_CMD:                                   //������
    heat_beat_check();
    break;
    
  case PRODUCT_INFO_CMD:                                //��Ʒ��Ϣ
    product_info_update();
    break;
    
  case WORK_MODE_CMD:                                   //��ѯMCU�趨��ģ�鹤��ģʽ
    get_mcu_bt_mode();
    break;
    
#ifndef BT_CONTROL_SELF_MODE
  case BT_STATE_CMD:                                  //bt����״̬	
    bt_work_state = bt_uart_rx_buf[offset + DATA_START];
    bt_uart_write_frame(BT_STATE_CMD,0);
    break;

  case BT_RESET_CMD:                                  //����bt(bt���سɹ�)
    reset_bt_flag = RESET_BT_SUCCESS;
    break;
#endif
    
  case DATA_QUERT_CMD:                                  //�����·�
    total_len = bt_uart_rx_buf[offset + LENGTH_HIGH] * 0x100;
    total_len += bt_uart_rx_buf[offset + LENGTH_LOW];
    
    for(i = 0;i < total_len;)
    {
      dp_len = bt_uart_rx_buf[offset + DATA_START + i + 2] * 0x100;
      dp_len += bt_uart_rx_buf[offset + DATA_START + i + 3];
      //
      ret = data_point_handle((unsigned char *)bt_uart_rx_buf + offset + DATA_START + i);
      
      if(SUCCESS == ret)
      {
        //�ɹ���ʾ
				savevar();
      }
      else
      {
        //������ʾ
      }
      
      i += (dp_len + 4);
    }
    
    break;
  case BT_Check_meshgroup:                                  //��ѯȺ��
    total_len = bt_uart_rx_buf[offset + LENGTH_HIGH] * 0x100;
    total_len += bt_uart_rx_buf[offset + LENGTH_LOW];


	groupaddr[0] = bt_uart_rx_buf[offset + 7] * 0x100;
	groupaddr[0] += bt_uart_rx_buf[offset + 8] ;

	groupaddr[1] = bt_uart_rx_buf[offset + 9] * 0x100;
	groupaddr[1] += bt_uart_rx_buf[offset + 10] ;

	groupaddr[2] = bt_uart_rx_buf[offset + 11] * 0x100;
	groupaddr[2] += bt_uart_rx_buf[offset + 12] ;

	groupaddr[3] = bt_uart_rx_buf[offset + 13] * 0x100;
	groupaddr[3] += bt_uart_rx_buf[offset + 14] ;

	groupaddr[4] = bt_uart_rx_buf[offset + 15] * 0x100;
	groupaddr[4] += bt_uart_rx_buf[offset + 16] ;

	groupaddr[5] = bt_uart_rx_buf[offset + 17] * 0x100;
	groupaddr[5] += bt_uart_rx_buf[offset + 18] ;

	groupaddr[6] = bt_uart_rx_buf[offset + 19] * 0x100;
	groupaddr[6] += bt_uart_rx_buf[offset + 20] ;

	groupaddr[7] = bt_uart_rx_buf[offset + 21] * 0x100;
	groupaddr[7] += bt_uart_rx_buf[offset + 22] ;

	

    break;
  case STATE_QUERY_CMD:                                 //״̬��ѯ
    all_data_update();                               
    break;
    
#ifdef SUPPORT_MCU_FIRM_UPDATE
  case UPDATE_START_CMD:                                //������ʼ
    firm_length = bt_uart_rx_buf[offset + DATA_START];
    firm_length <<= 8;
    firm_length |= bt_uart_rx_buf[offset + DATA_START + 1];
    firm_length <<= 8;
    firm_length |= bt_uart_rx_buf[offset + DATA_START + 2];
    firm_length <<= 8;
    firm_length |= bt_uart_rx_buf[offset + DATA_START + 3];
    //
    bt_uart_write_frame(UPDATE_START_CMD,0);
    firm_update_flag = UPDATE_START_CMD;
     break;
    
  case UPDATE_TRANS_CMD:                                //��������
    if(firm_update_flag == UPDATE_START_CMD)
    {
      //ֹͣһ�������ϱ�
      stop_update_flag = ENABLE;                                                 
      
      total_len = bt_uart_rx_buf[offset + LENGTH_HIGH] * 0x100;
      total_len += bt_uart_rx_buf[offset + LENGTH_LOW];
      
      dp_len = bt_uart_rx_buf[offset + DATA_START];
      dp_len <<= 8;
      dp_len |= bt_uart_rx_buf[offset + DATA_START + 1];
      dp_len <<= 8;
      dp_len |= bt_uart_rx_buf[offset + DATA_START + 2];
      dp_len <<= 8;
      dp_len |= bt_uart_rx_buf[offset + DATA_START + 3];
      
      firmware_addr = bt_uart_rx_buf + offset + DATA_START + 4;
      if((total_len == 4) && (dp_len == firm_length))
      {
        //���һ��
        ret = mcu_firm_update_handle(firmware_addr,dp_len,0);
        
        firm_update_flag = 0;
      }
      else if((total_len - 4) <= FIRM_UPDATA_SIZE)
      {
        ret = mcu_firm_update_handle(firmware_addr,dp_len,total_len - 4);
      }
      else
      {
        firm_update_flag = 0;
        ret = ERROR;
      }
      
      if(ret == SUCCESS)
      {
        bt_uart_write_frame(UPDATE_TRANS_CMD,0);
      }
      //�ָ�һ�������ϱ�
      stop_update_flag = DISABLE;    
    }
    break;
#endif      

  default:
    break;
  }
}
/*****************************************************************************
�������� : get_queue_total_data
�������� : ��ȡ����������
������� : ��
���ز��� : ��
*****************************************************************************/
unsigned char get_queue_total_data(void)
{
  if(queue_in != queue_out)
    return 1;
  else
    return 0;
}
/*****************************************************************************
�������� : Queue_Read_Byte
�������� : ��ȡ����1�ֽ�����
������� : ��
���ز��� : ��
*****************************************************************************/
unsigned char Queue_Read_Byte(void)
{
  unsigned char value;
  
  if(queue_out != queue_in)
  {
    //������
    if(queue_out >= (unsigned char *)(bt_queue_buf + sizeof(bt_queue_buf)))
    {
      //�����Ѿ���ĩβ
      queue_out = (unsigned char *)(bt_queue_buf);
    }
    
    value = *queue_out ++;   
  }
  
  return value;
}

