/****************************************Copyright (c)*************************
**                               ��Ȩ���� (C), 2015-2017, Ϳѻ�Ƽ�
**
**                                 http://www.tuya.com
**
**--------------�ļ���Ϣ-------------------------------------------------------
**��   ��   ��: mcu_api.c
**��        ��: �·�/�ϱ����ݴ�����
**ʹ �� ˵ �� : ���ļ��º��������û��޸�,�û���Ҫ���õ��ļ����ڸ��ļ���
**
**
**--------------��ǰ�汾�޶�---------------------------------------------------
** ��  ��: v1.0
** �ա���: 2017��5��3��
** �衡��: 1:����Ϳѻbluetooth�Խ�MCU_SDK
**
**-----------------------------------------------------------------------------
******************************************************************************/
#define MCU_API_GLOBAL

#include "bluetooth.h"

/*****************************************************************************
�������� : hex_to_bcd
�������� : hexתbcd
������� : Value_H:���ֽ�/Value_L:���ֽ�
���ز��� : bcd_value:ת����ɺ�����
*****************************************************************************/
unsigned char hex_to_bcd(unsigned char Value_H,unsigned char Value_L)
{
  unsigned char bcd_value;
  
  if((Value_H >= '0') && (Value_H <= '9'))
    Value_H -= '0';
  else if((Value_H >= 'A') && (Value_H <= 'F'))
    Value_H = Value_H - 'A' + 10;
  else if((Value_H >= 'a') && (Value_H <= 'f'))
    Value_H = Value_H - 'a' + 10;
   
  bcd_value = Value_H & 0x0f;
  
  bcd_value <<= 4;
  if((Value_L >= '0') && (Value_L <= '9'))
    Value_L -= '0';
  else if((Value_L >= 'A') && (Value_L <= 'F'))
    Value_L = Value_L - 'a' + 10;
  else if((Value_L >= 'a') && (Value_L <= 'f'))
    Value_L = Value_L - 'a' + 10;
  
  bcd_value |= Value_L & 0x0f;

  return bcd_value;
}
/*****************************************************************************
�������� : my_strlen
�������� : ���ַ�������
������� : src:Դ��ַ
���ز��� : len:���ݳ���
*****************************************************************************/
unsigned long my_strlen(unsigned char *str)  
{
  unsigned long len = 0;
  if(str == NULL)
  { 
    return 0;
  }
  
  for(len = 0; *str ++ != '\0'; )
  {
    len ++;
  }
  
  return len;
}
/*****************************************************************************
�������� : my_memset
�������� : ��src��ָ�ڴ������ǰcount���ֽ����ó��ַ�c
������� : src:Դ��ַ
           ch:�����ַ�
           count:�������ݳ���
���ز��� : src:���ݴ�������Դ��ַ
*****************************************************************************/
void *my_memset(void *src,unsigned char ch,unsigned short count)
{
  unsigned char *tmp = (unsigned char *)src;
  
  if(src == NULL)
  {
    return NULL;
  }
  
  while(count --)
  {
    *tmp ++ = ch;
  }
  
  return src;
}
/*****************************************************************************
�������� : mymemcpy
�������� : �ڴ濽��
������� : dest:Ŀ���ַ
           src:Դ��ַ
           count:���ݿ�������
���ز��� : src:���ݴ�������Դ��ַ
*****************************************************************************/
void *my_memcpy(void *dest, const void *src, unsigned short count)  
{  
  unsigned char *pdest = (unsigned char *)dest;  
  const unsigned char *psrc  = (const unsigned char *)src;  
  unsigned short i;
  
  if(dest == NULL || src == NULL)
  { 
    return NULL;
  }
  
  if((pdest <= psrc) || (pdest > psrc + count))
  {  
    for(i = 0; i < count; i ++)
    {  
      pdest[i] = psrc[i];  
    }  
  }
  else
  {
    for(i = count; i > 0; i --)
    {  
      pdest[i - 1] = psrc[i - 1];  
    }  
  }  
  
  return dest;  
}
/*****************************************************************************
�������� : int_to_byte
�������� : ��int���Ͳ���ĸ��ֽ�
������� : number:4�ֽ�ԭ����;value:������ɺ�4�ֽ�����
���ز��� :��
****************************************************************************/
void int_to_byte(unsigned long number,unsigned char value[4])
{
  value[0] = number >> 24;
  value[1] = number >> 16;
  value[2] = number >> 8;
  value[3] = number & 0xff;
}
/*****************************************************************************
�������� : byte_to_int
�������� : ��4�ֽںϲ�Ϊ1��32bit����
������� : value:4�ֽ�����
���ز��� : number:�ϲ���ɺ��32bit����
****************************************************************************/
unsigned long byte_to_int(const unsigned char value[4])
{
  unsigned long nubmer = 0;

  nubmer = (unsigned long)value[0];
  nubmer <<= 8;
  nubmer |= (unsigned long)value[1];
  nubmer <<= 8;
  nubmer |= (unsigned long)value[2];
  nubmer <<= 8;
  nubmer |= (unsigned long)value[3];
  
  return nubmer;
}
#ifndef BT_CONTROL_SELF_MODE
/*****************************************************************************
�������� : mcu_get_reset_bt_flag
�������� : MCU��ȡ��λbt�ɹ���־
������� : ��
���ز��� : ��λ��־:RESET_BT_ERROR:ʧ��/RESET_BT_SUCCESS:�ɹ�
ʹ��˵�� : 1:MCU��������mcu_reset_bt()����øú�����ȡ��λ״̬
           2:���Ϊģ���Դ���ģʽ,MCU������øú���
*****************************************************************************/
unsigned char mcu_get_reset_bt_flag(void)
{
  return reset_bt_flag;
}
/*****************************************************************************
�������� : mcu_reset_bt
�������� : MCU��������bt����ģʽ
������� : ��
���ز��� : ��
ʹ��˵�� : 1:MCU��������,ͨ��mcu_get_reset_bt_flag()������ȡ����bt�Ƿ�ɹ�
           2:���Ϊģ���Դ���ģʽ,MCU������øú���
*****************************************************************************/
void mcu_reset_bt(void)
{
  reset_bt_flag = RESET_BT_ERROR;
  
  bt_uart_write_frame(BT_RESET_CMD, 0);
}
/*****************************************************************************
�������� : mcu_get_bt_work_state
�������� : MCU������ȡ��ǰbt����״̬
������� : ��
���ز��� : BT_WORK_SATE_E:
            BT_UN_BIND:����δ��
            BT_UNCONNECT:�����Ѱ�δ����
            BT_CONNECTED:�����Ѱ󶨲�������
ʹ��˵�� : ��
*****************************************************************************/
unsigned char mcu_get_bt_work_state(void)
{
  return bt_work_state;
}
#endif

/*****************************************************************************
�������� : mcu_dp_raw_update
�������� : raw��dp�����ϴ�
������� : dpid:id��
           value:��ǰdpֵָ��
           len:���ݳ���
���ز��� : ��
*****************************************************************************/
unsigned char mcu_dp_raw_update(unsigned char dpid,const unsigned char value[],unsigned short len)
{
  unsigned short length = 0;
  
  if(stop_update_flag == ENABLE)
    return SUCCESS;
  //
  length = set_bt_uart_byte(length,dpid);
  length = set_bt_uart_byte(length,DP_TYPE_RAW);
  //
  length = set_bt_uart_byte(length,len / 0x100);
  length = set_bt_uart_byte(length,len % 0x100);
  //
  length = set_bt_uart_buffer(length,(unsigned char *)value,len);
  
  bt_uart_write_frame(STATE_UPLOAD_CMD,length);
  
  return SUCCESS;
}
/*****************************************************************************
�������� : mcu_dp_bool_update
�������� : bool��dp�����ϴ�
������� : dpid:id��
           value:��ǰdpֵ
���ز��� : ��
*****************************************************************************/
unsigned char mcu_dp_bool_update(unsigned char dpid,unsigned char value)
{
  unsigned short length = 0;
  
  if(stop_update_flag == ENABLE)
    return SUCCESS;
  
  length = set_bt_uart_byte(length,dpid);
  length = set_bt_uart_byte(length,DP_TYPE_BOOL);
  //
  length = set_bt_uart_byte(length,0);
  length = set_bt_uart_byte(length,1);
  //
  if(value == FALSE)
  {
    length = set_bt_uart_byte(length,FALSE);
  }
  else
  {
    length = set_bt_uart_byte(length,1);
  }
  
  bt_uart_write_frame(STATE_UPLOAD_CMD,length);
  
  return SUCCESS;
}
unsigned char mcu_dp_bool_mesh_update(unsigned char dpid,unsigned char value,unsigned int groupa)
{
  //unsigned short length = 0;
    unsigned char check_sum = 0;
	unsigned short len = 0;
	unsigned char buf[15];
	  unsigned short i;
  //unsigned char check_sum = 0;
  
 
  //55 AA 00 07 00 05 01 01 00 01 01 0F
  if(stop_update_flag == ENABLE)
    return SUCCESS;
  
  buf[0] = 0x55;
  buf[1] = 0xaa;
  buf[2] = 0x00;
  buf[3] = 0xb2;
  
  buf[4] = 0x0;
  buf[5] = 0x7;
  buf[6] = groupa >> 8;;
  buf[7] = groupa;
  buf[8] = dpid;
  buf[9] = 0x01;
  buf[10] = 0x00;
  buf[11] = 0x01;
  if(value == 0)
  {
	buf[12] = 0;
  }
  else
  {
	buf[12] = 1;
  }
  
  for(i = 0; i < 13; i ++)
  {
    check_sum += buf[i];
  }
  buf[13] = check_sum;
  len = 14;
  i = 0;
   while(len --)
  {
    uart_transmit_output(buf[i]);
    i ++;
  }

  
  return SUCCESS;
}

unsigned char mcu_dp_enum_mesh_update(unsigned char dpid,unsigned char value,unsigned int groupa)
{
  //unsigned short length = 0;
    unsigned char check_sum = 0;
	unsigned short len = 0;
	unsigned char buf[15];
	  unsigned short i;
  //unsigned char check_sum = 0;
  
 
  //55 AA 00 07 00 05 01 01 00 01 01 0F
  if(stop_update_flag == ENABLE)
    return SUCCESS;
  
  buf[0] = 0x55;
  buf[1] = 0xaa;
  buf[2] = 0x00;
  buf[3] = 0xb2;
  
  buf[4] = 0x0;
  buf[5] = 0x7;
  buf[6] = groupa >> 8;
  buf[7] = groupa;
  buf[8] = dpid;
  buf[9] = 0x01;
  buf[10] = 0x00;
  buf[11] = 0x01;
  if(value == 0)
  {
	buf[12] = 0;
  }
  else if(value == 1)
  {
	buf[12] = 1;
  }
  else if(value == 2)
  {
	buf[12] = 2;
  }
  else if(value == 3)
  {
	buf[12] = 3;
  }
  else if(value == 4)
  {
	buf[12] = 4;
  }
  else if(value == 5)
  {
	buf[12] = 5;
  }
  else if(value == 6)
  {
	buf[12] = 6;
  }
  
  for(i = 0; i < 13; i ++)
  {
    check_sum += buf[i];
  }
  buf[13] = check_sum;
  len = 14;
  i = 0;
   while(len --)
  {
    uart_transmit_output(buf[i]);
    i ++;
  }

  
  return SUCCESS;
}





/*****************************************************************************
�������� : mcu_dp_value_update
�������� : value��dp�����ϴ�
������� : dpid:id��
           value:��ǰdpֵ
���ز��� : ��
*****************************************************************************/
unsigned char mcu_dp_value_update(unsigned char dpid,unsigned long value)
{
  unsigned short length = 0;
  
  if(stop_update_flag == ENABLE)
    return SUCCESS;
  
  length = set_bt_uart_byte(length,dpid);
  length = set_bt_uart_byte(length,DP_TYPE_VALUE);
  //
  length = set_bt_uart_byte(length,0);
  length = set_bt_uart_byte(length,4);
  //
  length = set_bt_uart_byte(length,value >> 24);
  length = set_bt_uart_byte(length,value >> 16);
  length = set_bt_uart_byte(length,value >> 8);
  length = set_bt_uart_byte(length,value & 0xff);
  
  bt_uart_write_frame(STATE_UPLOAD_CMD,length);
  
  return SUCCESS;
}

unsigned char mcu_dp_value_mesh_update(unsigned char dpid,unsigned long value,unsigned int groupa)
{
  //unsigned short length = 0;
    unsigned char check_sum = 0;
	unsigned short len = 0;
	unsigned char idata buf[17];
	  unsigned short i;
  //unsigned char check_sum = 0;
  
 
  //55 AA 00 07 00 05 01 01 00 01 01 0F
  if(stop_update_flag == ENABLE)
    return SUCCESS;
  
  buf[0] = 0x55;
  buf[1] = 0xaa;
  buf[2] = 0x00;
  buf[3] = 0xb2;
  
  buf[4] = 0x0;
  buf[5] = 0xA;
  buf[6] = groupa >> 8;
  buf[7] = groupa;
  buf[8] = dpid;
  buf[9] = 0x02;
  buf[10] = 0x00;
  buf[11] = 0x04;
  buf[12] = value >> 24;
  buf[13] = value >> 16;
  buf[14] = value >> 8;
  buf[15] = value & 0xff;
  

  
  for(i = 0; i < 16; i ++)
  {
    check_sum += buf[i];
  }
  buf[16] = check_sum;
  len = 17;
  i = 0;
   while(len --)
  {
    uart_transmit_output(buf[i]);
    i ++;
  }

  
  return SUCCESS;
}


/*****************************************************************************
�������� : mcu_dp_string_update
�������� : rstring��dp�����ϴ�
������� : dpid:id��
           value:��ǰdpֵָ��
           len:���ݳ���
���ز��� : ��
*****************************************************************************/
unsigned char mcu_dp_string_update(unsigned char dpid,const unsigned char value[],unsigned short len)
{
  unsigned short length = 0;
  
  if(stop_update_flag == ENABLE)
    return SUCCESS;
  //
  length = set_bt_uart_byte(length,dpid);
  length = set_bt_uart_byte(length,DP_TYPE_STRING);
  //
  length = set_bt_uart_byte(length,len / 0x100);
  length = set_bt_uart_byte(length,len % 0x100);
  //
  length = set_bt_uart_buffer(length,(unsigned char *)value,len);
  
  bt_uart_write_frame(STATE_UPLOAD_CMD,length);
  
  return SUCCESS;
}
/*****************************************************************************
�������� : mcu_dp_enum_update
�������� : enum��dp�����ϴ�
������� : dpid:id��
           value:��ǰdpֵ
���ز��� : ��
*****************************************************************************/
unsigned char mcu_dp_enum_update(unsigned char dpid,unsigned char value)
{
  unsigned short length = 0;
  
  if(stop_update_flag == ENABLE)
    return SUCCESS;
  
  length = set_bt_uart_byte(length,dpid);
  length = set_bt_uart_byte(length,DP_TYPE_ENUM);
  //
  length = set_bt_uart_byte(length,0);
  length = set_bt_uart_byte(length,1);
  //
  length = set_bt_uart_byte(length,value);
  
  bt_uart_write_frame(STATE_UPLOAD_CMD,length);
  
  return SUCCESS;
}
/*****************************************************************************
�������� : mcu_dp_fault_update
�������� : fault��dp�����ϴ�
������� : dpid:id��
           value:��ǰdpֵ
���ز��� : ��
*****************************************************************************/
unsigned char mcu_dp_fault_update(unsigned char dpid,unsigned long value)
{
  unsigned short length = 0;
   
  if(stop_update_flag == ENABLE)
    return SUCCESS;
  
  length = set_bt_uart_byte(length,dpid);
  length = set_bt_uart_byte(length,DP_TYPE_FAULT);
  //
  length = set_bt_uart_byte(length,0);
  
  if((value | 0xff) == 0xff)
  {
    length = set_bt_uart_byte(length,1);
    length = set_bt_uart_byte(length,value);
  }
  else if((value | 0xffff) == 0xffff)
  {
    length = set_bt_uart_byte(length,2);
    length = set_bt_uart_byte(length,value >> 8);
    length = set_bt_uart_byte(length,value & 0xff);
  }
  else
  {
    length = set_bt_uart_byte(length,4);
    length = set_bt_uart_byte(length,value >> 24);
    length = set_bt_uart_byte(length,value >> 16);
    length = set_bt_uart_byte(length,value >> 8);
    length = set_bt_uart_byte(length,value & 0xff);
  }    
  
  bt_uart_write_frame(STATE_UPLOAD_CMD,length);

  return SUCCESS;
}
/*****************************************************************************
�������� : mcu_get_dp_download_bool
�������� : mcu��ȡbool���·�dpֵ
������� : value:dp���ݻ�������ַ
           length:dp���ݳ���
���ز��� : bool:��ǰdpֵ
*****************************************************************************/
unsigned char mcu_get_dp_download_bool(const unsigned char value[],unsigned short len)
{
	int i;
	i = len;
  return(value[0]);
}
/*****************************************************************************
�������� : mcu_get_dp_download_enum
�������� : mcu��ȡenum���·�dpֵ
������� : value:dp���ݻ�������ַ
           length:dp���ݳ���
���ز��� : enum:��ǰdpֵ
*****************************************************************************/
unsigned char mcu_get_dp_download_enum(const unsigned char value[],unsigned short len)
{
	int i;
	i = len;
  return(value[0]);
}
/*****************************************************************************
�������� : mcu_get_dp_download_value
�������� : mcu��ȡvalue���·�dpֵ
������� : value:dp���ݻ�������ַ
           length:dp���ݳ���
���ز��� : value:��ǰdpֵ
*****************************************************************************/
unsigned long mcu_get_dp_download_value(const unsigned char value[],unsigned short len)
{
	int i;
	i = len;
  return(byte_to_int(value));
}
/*****************************************************************************
�������� : uart_receive_input
�������� : �����ݴ���
������� : value:�����յ��ֽ�����
���ز��� : ��
ʹ��˵�� : ��MCU���ڽ��պ����е��øú���,�������յ���������Ϊ��������
*****************************************************************************/
void uart_receive_input(unsigned char value)
{
  if((queue_in > queue_out) && ((queue_in - queue_out) >= sizeof(bt_queue_buf)))
  {
    //���ݶ�����
  }
  else if((queue_in < queue_out) && ((queue_out  - queue_in) == 0))
  {
    //���ݶ�����
  }
  else
  {
    //���в���
    if(queue_in >= (unsigned char *)(bt_queue_buf + sizeof(bt_queue_buf)))
    {
      queue_in = (unsigned char *)(bt_queue_buf);
    }
    
    *queue_in ++ = value;
  }
}
/*****************************************************************************
��������  : bt_uart_service
��������  : bt���ڴ������
������� : ��
���ز��� : ��
ʹ��˵�� : ��MCU������whileѭ���е��øú���
*****************************************************************************/
void bt_uart_service(void)
{
  static unsigned short rx_in = 0;
  unsigned short offset = 0;
  unsigned short rx_value_len = 0;             //����֡����
  
  while((rx_in < sizeof(bt_uart_rx_buf)) && get_queue_total_data() > 0)
  {
    bt_uart_rx_buf[rx_in ++] = Queue_Read_Byte();
  }
  
  if(rx_in < PROTOCOL_HEAD)
    return;
  
  while((rx_in - offset) >= PROTOCOL_HEAD)
  {
    if(bt_uart_rx_buf[offset + HEAD_FIRST] != FRAME_FIRST)
    {
      offset ++;
      continue;
    }
    
    if(bt_uart_rx_buf[offset + HEAD_SECOND] != FRAME_SECOND)
    {
      offset ++;
      continue;
    }  
    
    if(bt_uart_rx_buf[offset + PROTOCOL_VERSION] != VERSION)
    {
      offset += 2;
      continue;
    }      
    
    rx_value_len = bt_uart_rx_buf[offset + LENGTH_HIGH] * 0x100 + bt_uart_rx_buf[offset + LENGTH_LOW] + PROTOCOL_HEAD;
    if(rx_value_len > sizeof(bt_uart_rx_buf) + PROTOCOL_HEAD)
    {
      offset += 3;
      continue;
    }
    
    if((rx_in - offset) < rx_value_len)
    {
      break;
    }
    
    //���ݽ������
    if(get_check_sum((unsigned char *)bt_uart_rx_buf + offset,rx_value_len - 1) != bt_uart_rx_buf[offset + rx_value_len - 1])
    {
      //У�����
      offset += 3;
      continue;
    }
    
    data_handle(offset);
    offset += rx_value_len;
  }//end while

  rx_in -= offset;
  if(rx_in > 0)
  {
    my_memcpy(bt_uart_rx_buf,bt_uart_rx_buf + offset,rx_in);
  }
}
/*****************************************************************************
�������� :  bt_protocol_init
�������� : Э�鴮�ڳ�ʼ������
������� : ��
���ز��� : ��
ʹ��˵�� : ������MCU��ʼ�������е��øú���
*****************************************************************************/
void bt_protocol_init(void)
{
  queue_in = (unsigned char *)bt_queue_buf;
  queue_out = (unsigned char *)bt_queue_buf;
  //
#ifndef BT_CONTROL_SELF_MODE
  bt_work_state = BT_SATE_UNKNOW;
#endif
}


