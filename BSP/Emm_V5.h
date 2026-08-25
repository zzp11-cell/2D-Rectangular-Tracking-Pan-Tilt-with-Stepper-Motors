#ifndef __EMM_V5_H
#define __EMM_V5_H

#include "usart.h"
#include <stdbool.h>

/**********************************************************
***	Emm_V5.0�����ջ���������
***	��д���ߣ�ZHANGDATOU
***	����֧�֣��Ŵ�ͷ�ջ��ŷ�
***	�Ա����̣�https://zhangdatou.taobao.com
***	CSDN���ͣ�http s://blog.csdn.net/zhangdatou666
***	qq����Ⱥ��262438510
**********************************************************/

#define					ABS(x)							((x) > 0 ? (x) : -(x)) 

typedef enum {
	S_VBUS  = 5,	// ��ȡ���ߵ�ѹ
	S_CBUS  = 6,	// ��ȡ���ߵ���
	S_CPHA  = 7,	// ��ȡ�����
	S_ENCO  = 8,	// ��ȡ������ԭʼֵ
	S_CLKC  = 9,	// ��ȡʵʱ������
	S_ENCL  = 10,	// ��ȡ�������Ի�У׼��ı�����ֵ
	S_CLKI  = 11,	// ��ȡ����������
	S_TPOS  = 12,	// ��ȡ���Ŀ��λ��
	S_SPOS  = 13,	// ��ȡ���ʵʱ�趨��Ŀ��λ��
	S_VEL   = 14,	// ��ȡ���ʵʱת��
	S_CPOS  = 15,	// ��ȡ���ʵʱλ��
	S_PERR  = 16,	// ��ȡ���λ�����
	S_VBAT  = 17,	// ��ȡ��Ȧ��������ص�ѹ��Y42��
	S_TEMP  = 18,	// ��ȡ���ʵʱ�¶ȣ�Y42��
	S_FLAG  = 19,	// ��ȡ���״̬��־λ
	S_OFLAG = 20, // ��ȡ����״̬��־λ
	S_OAF   = 21,	// ��ȡ���״̬��־λ + ����״̬��־λ��Y42��
	S_PIN   = 22,	// ��ȡ����״̬��Y42��
}SysParams_t;

#define		MMCL_LEN		512
extern __IO uint16_t MMCL_count, MMCL_cmd[MMCL_LEN];

/**
***********************************************************
***********************************************************
*** 
***
*** @brief	��׺���У�Y42��ΪY42�������X42�����ã�����ͨ��
***
*** 
***********************************************************
***********************************************************
***/
/**********************************************************
*** ������������
**********************************************************/
// ����������У׼
void Emm_V5_Trig_Encoder_Cal(uint8_t addr);
// ���������Y42��
void Emm_V5_Reset_Motor(uint8_t addr);
// ����ǰλ������
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr);
// �����ת����
void Emm_V5_Reset_Clog_Pro(uint8_t addr);
// �ָ���������
void Emm_V5_Restore_Motor(uint8_t addr);
/**********************************************************
*** �˶���������
**********************************************************/
// �������Y42��
void Emm_V5_Multi_Motor_Cmd(uint8_t addr);
// ���ʹ�ܿ���
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF);
// �ٶ�ģʽ����
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);
// λ��ģʽ����
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, uint8_t raF, bool snF);
// ���ÿ���λ��ģʽ���˶�����
void Emm_V5_Set_QPos_Params(uint8_t addr, uint16_t vel, uint8_t acc, uint8_t raF, bool snF);
// ����λ��ģʽ����
void Emm_V5_QPos_Control(uint8_t addr, int32_t clk);
// �õ������ֹͣ�˶�
void Emm_V5_Stop_Now(uint8_t addr, bool snF);
// �������ͬ����ʼ�˶�
void Emm_V5_Synchronous_motion(uint8_t addr);
/**********************************************************
*** ԭ���������
**********************************************************/
// ���õ�Ȧ��������λ��
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF);
// ��������
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF);
// ǿ���жϲ��˳�����
void Emm_V5_Origin_Interrupt(uint8_t addr);
// ��ȡ�������
void Emm_V5_Origin_Read_Params(uint8_t addr);
// �޸Ļ������
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF);
// ��ȡ��ײ���㷵�ؽǶȣ�X42S/Y42��
void X_V2_Origin_Read_SL_RP(uint8_t addr);
// �޸���ײ���㷵�ؽǶȣ�X42S/Y42��
void X_V2_Origin_Modify_SL_RP(uint8_t addr, bool svF, uint16_t sl_rp);
/**********************************************************
*** ��ȡϵͳ��������
**********************************************************/
// ��ʱ������Ϣ���Y42��
void Emm_V5_Auto_Return_Sys_Params_Timed(uint8_t addr, SysParams_t s, uint16_t time_ms);
// ��ȡϵͳ����
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s);
/**********************************************************
*** ��д������������
**********************************************************/
// �޸ĵ��ID��ַ
void Emm_V5_Modify_Motor_ID(uint8_t addr, bool svF, uint8_t id);
// �޸�ϸ��ֵ
void Emm_V5_Modify_MicroStep(uint8_t addr, bool svF, uint8_t mstep);
// �޸ĵ����־
void Emm_V5_Modify_PDFlag(uint8_t addr, bool pdf);
// ��ȡѡ�����״̬��Y42��
void Emm_V5_Read_Opt_Param_Sta(uint8_t addr);
// �޸ĵ�����ͣ�Y42��
void Emm_V5_Modify_Motor_Type(uint8_t addr, bool svF, bool mottype);
// �޸Ĺ̼����ͣ�Y42��
void Emm_V5_Modify_Firmware_Type(uint8_t addr, bool svF, bool fwtype);
// �޸Ŀ���/�ջ�����ģʽ��Y42��
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, bool ctrl_mode);
// �޸ĵ���˶�������Y42��
void Emm_V5_Modify_Motor_Dir(uint8_t addr, bool svF, bool dir);
// �޸������������ܣ�Y42��
void Emm_V5_Modify_Lock_Btn(uint8_t addr, bool svF, bool lockbtn);
// �޸������ٶ�ֵ�Ƿ���С10�����루Y42��
void Emm_V5_Modify_S_Vel(uint8_t addr, bool svF, bool s_vel);
// �޸Ŀ���ģʽ��������
void Emm_V5_Modify_OM_ma(uint8_t addr, bool svF, uint16_t om_ma);
// �޸ıջ�ģʽ������
void Emm_V5_Modify_FOC_mA(uint8_t addr, bool svF, uint16_t foc_mA);
// ��ȡPID����
void Emm_V5_Read_PID_Params(uint8_t addr);
// �޸�PID����
void Emm_V5_Modify_PID_Params(uint8_t addr, bool svF, uint32_t kp, uint32_t ki, uint32_t kd);
// ��ȡDMX512Э�������Y42��
void Emm_V5_Read_DMX512_Params(uint8_t addr);
// �޸�DMX512Э�������Y42��
void Emm_V5_Modify_DMX512_Params(uint8_t addr, bool svF, uint16_t tch, uint8_t nch, uint8_t mode, uint16_t vel, uint16_t acc, uint16_t vel_step, uint32_t pos_step);
// ��ȡλ�õ��ﴰ�ڣ�Y42��
void Emm_V5_Read_Pos_Window(uint8_t addr);
// �޸�λ�õ��ﴰ�ڣ�Y42��
void Emm_V5_Modify_Pos_Window(uint8_t addr, bool svF, uint16_t prw);
// ��ȡ���ȹ������������ֵ��Y42��
void Emm_V5_Read_Otocp(uint8_t addr);
// �޸Ĺ��ȹ������������ֵ��Y42��
void Emm_V5_Modify_Otocp(uint8_t addr, bool svF, uint16_t otp, uint16_t ocp, uint16_t time_ms);
// ��ȡ������������ʱ�䣨Y42��
void Emm_V5_Read_Heart_Protect(uint8_t addr);
// �޸�������������ʱ�䣨Y42��
void Emm_V5_Modify_Heart_Protect(uint8_t addr, bool svF, uint32_t hp);
// ��ȡ�����޷�/����ϵ����Y42��
void Emm_V5_Read_Integral_Limit(uint8_t addr);
// �޸Ļ����޷�/����ϵ����Y42��
void Emm_V5_Modify_Integral_Limit(uint8_t addr, bool svF, uint32_t il);
/**********************************************************
*** ��ȡ����������������
**********************************************************/
// ��ȡϵͳ״̬����
void Emm_V5_Read_System_State_Params(uint8_t addr);
// ��ȡ�������ò���
void Emm_V5_Read_Motor_Conf_Params(uint8_t addr);

/**
***********************************************************
***********************************************************
*** 
***
*** @brief	�����ǰ���Ӧ������ص�Y42���������ϵĺ�����Y42��
***
*** 
***********************************************************
***********************************************************
***/
/**********************************************************
*** ������������
**********************************************************/
// ����������У׼ - ���ص�����ָ����
void Emm_V5_MMCL_Trig_Encoder_Cal(uint8_t addr);
// ������� - ���ص�����ָ����
void Emm_V5_MMCL_Reset_Motor(uint8_t addr);
// ����ǰλ������ - ���ص�����ָ����
void Emm_V5_MMCL_Reset_CurPos_To_Zero(uint8_t addr);
// �����ת���� - ���ص�����ָ����
void Emm_V5_MMCL_Reset_Clog_Pro(uint8_t addr);
// �ָ��������� - ���ص�����ָ����
void Emm_V5_MMCL_Restore_Motor(uint8_t addr);
/**********************************************************
*** �˶���������
**********************************************************/
// ���ʹ�ܿ��� - ���ص�����ָ����
void Emm_V5_MMCL_En_Control(uint8_t addr, bool state, bool snF);
// �ٶ�ģʽ���� - ���ص�����ָ����
void Emm_V5_MMCL_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);
// λ��ģʽ���� - ���ص�����ָ����
void Emm_V5_MMCL_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, uint8_t raF, bool snF);
// ���ÿ���λ��ģʽ���˶�����
void Emm_V5_MMCL_Set_QPos_Params(uint8_t addr, uint16_t vel, uint8_t acc, uint8_t raF, bool snF);
// ����λ��ģʽ����
void Emm_V5_MMCL_QPos_Control(uint8_t addr, int32_t clk);
// �õ������ֹͣ�˶� - ���ص�����ָ����
void Emm_V5_MMCL_Stop_Now(uint8_t addr, bool snF);
// �������ͬ����ʼ�˶� - ���ص�����ָ����
void Emm_V5_MMCL_Synchronous_motion(uint8_t addr);
/**********************************************************
*** ԭ���������
**********************************************************/
// ���õ�Ȧ��������λ�� - ���ص�����ָ����
void Emm_V5_MMCL_Origin_Set_O(uint8_t addr, bool svF);
// �������� - ���ص�����ָ����
void Emm_V5_MMCL_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF);
// ǿ���жϲ��˳����� - ���ص�����ָ����
void Emm_V5_MMCL_Origin_Interrupt(uint8_t addr);
// �޸Ļ������ - ���ص�����ָ����
void Emm_V5_MMCL_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF);
// ��ȡ��ײ���㷵�ؽǶȣ�X42S/Y42�� - ���ص�����ָ����
void X_V2_MMCL_Origin_Read_SL_RP(uint8_t addr);
// �޸���ײ���㷵�ؽǶȣ�X42S/Y42�� - ���ص�����ָ����
void X_V2_MMCL_Origin_Modify_SL_RP(uint8_t addr, bool svF, uint16_t sl_rp);
/**********************************************************
*** ��ȡϵͳ��������
**********************************************************/
// ��ʱ������Ϣ���Y42�� - ���ص�����ָ����
void Emm_V5_MMCL_Auto_Return_Sys_Params_Timed(uint8_t addr, SysParams_t s, uint16_t time_ms);
// ��ȡϵͳ���� - ���ص�����ָ����
void Emm_V5_MMCL_Read_Sys_Params(uint8_t addr, SysParams_t s);
/**********************************************************
*** ��д������������
**********************************************************/

#endif
