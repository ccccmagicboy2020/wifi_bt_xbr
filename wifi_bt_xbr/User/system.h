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
#ifndef __SYSTEM_H_
#define __SYSTEM_H_

#ifdef SYSTEM_GLOBAL
  #define SYSTEM_EXTERN
#else
  #define SYSTEM_EXTERN   extern
#endif

//=============================================================================
//֡���ֽ�˳��
//=============================================================================
#define         HEAD_FIRST                      0
#define         HEAD_SECOND                     1        
#define         PROTOCOL_VERSION                2
#define         FRAME_TYPE                      3
#define         LENGTH_HIGH                     4
#define         LENGTH_LOW                      5
#define         DATA_START                      6

//=============================================================================
//����֡����
//=============================================================================
#define         HEAT_BEAT_CMD                   0                               //������
#define         PRODUCT_INFO_CMD                1                               //��Ʒ��Ϣ
#define         WORK_MODE_CMD                   2                               //��ѯMCU �趨��ģ�鹤��ģʽ	
#define         BT_STATE_CMD                    3                               //bluetooth����״̬	
#define         BT_RESET_CMD                    4                               //����bluetooth
#define         DATA_QUERT_CMD                  6                               //�����·�
#define         STATE_UPLOAD_CMD                7                               //״̬�ϱ�	 
#define         STATE_QUERY_CMD                 8                               //״̬��ѯ   
//#define         UPDATE_START_CMD                0x0a                            //������ʼ
//#define         UPDATE_TRANS_CMD                0x0b                            //��������
#define         BT_MESH_CMD                0xB2                            //MESH����
#define         BT_Check_meshgroup         0xB4                            //��ѯȺ���ַ
//=============================================================================
#define         VERSION                 0x00                                            //Э��汾��
#define         PROTOCOL_HEAD           0x07                                            //�̶�Э��ͷ����
//#define         FIRM_UPDATA_SIZE        256                                            //��������С
#define         FRAME_FIRST             0x55
#define         FRAME_SECOND            0xaa
//============================================================================= 
SYSTEM_EXTERN unsigned char  volatile  xdata bt_queue_buf[PROTOCOL_HEAD + BT_UART_QUEUE_LMT];  //���ڶ��л���
SYSTEM_EXTERN unsigned char xdata  bt_uart_rx_buf[PROTOCOL_HEAD + BT_UART_RECV_BUF_LMT];         //���ڽ��ջ���
SYSTEM_EXTERN unsigned char  xdata bt_uart_tx_buf[PROTOCOL_HEAD + BT_UART_SEND_BUF_LMT];        //���ڷ��ͻ���
//
SYSTEM_EXTERN volatile unsigned char *queue_in;
SYSTEM_EXTERN volatile unsigned char *queue_out;

SYSTEM_EXTERN unsigned char  xdata stop_update_flag;

#ifndef BT_CONTROL_SELF_MODE
SYSTEM_EXTERN unsigned char xdata reset_bt_flag;                                                  //����bt��־(TRUE:�ɹ�/FALSE:ʧ��)
SYSTEM_EXTERN unsigned char xdata set_btmode_flag;                                                //����bluetooth����ģʽ��־(TRUE:�ɹ�/FALSE:ʧ��)
SYSTEM_EXTERN unsigned char xdata bt_work_state;                                                  //btģ�鵱ǰ����״̬
#endif


/*****************************************************************************
�������� : set_bt_uart_byte
�������� : дbt_uart�ֽ�
������� : dest:��������ʵ��ַ;
           byte:д���ֽ�ֵ
���ز��� : д����ɺ���ܳ���
*****************************************************************************/
unsigned short set_bt_uart_byte(unsigned short dest, unsigned char byte);

/*****************************************************************************
�������� : set_bt_uart_buffer
�������� : дbt_uart_buffer
������� : dest:Ŀ���ַ
           src:Դ��ַ
           len:���ݳ���
���ز��� : ��
*****************************************************************************/
unsigned short set_bt_uart_buffer(unsigned short dest, unsigned char *src, unsigned short len);

/*****************************************************************************
�������� : bt_uart_write_frame
�������� : ��bt���ڷ���һ֡����
������� : fr_type:֡����
           len:���ݳ���
���ز��� : ��
*****************************************************************************/
void bt_uart_write_frame(unsigned char fr_type, unsigned short len);
void bt_uart_mesh_write_frame(unsigned char fr_type, unsigned short len);

/*****************************************************************************
�������� : get_check_sum
�������� : ����У���
������� : pack:����Դָ��
           pack_len:����У��ͳ���
���ز��� : У���
*****************************************************************************/
unsigned char get_check_sum(unsigned char *pack, unsigned short pack_len);

/*****************************************************************************
�������� : data_handle
�������� : ����֡����
������� : offset:������ʼλ
���ز��� : ��
*****************************************************************************/
void data_handle(unsigned short offset);

/*****************************************************************************
�������� : get_queue_total_data
�������� : ��ȡ����������
������� : ��
���ز��� : ��
*****************************************************************************/
unsigned char get_queue_total_data(void);

/*****************************************************************************
�������� : Queue_Read_Byte
�������� : ��ȡ����1�ֽ�����
������� : ��
���ز��� : ��
*****************************************************************************/
unsigned char Queue_Read_Byte(void);

#endif
  
  