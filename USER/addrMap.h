#ifndef ADDR_MAP_H
#define ADDR_MAP_H

#define			SP_SLAVE_ADDR					0x0000		//�ӻ���ַ
#define			SP_SLAVE_LEN					1

//0x0001~0x0002
#define			SP_CLEAR_PERIOD_ADDR					0x0001	//�������
#define			SP_CLEAR_PERIOD_LEN				2

#define			SP_DARK_SPECTRUM_ADDR			0x1000	//�����׿�ʼλ��
#define			SP_DARK_SPECTRUM_LEN			2048	//�����׳���

#define			SP_REFENCE_SPECTRUM_ADDR		((SP_DARK_SPECTRUM_ADDR)+(SP_DARK_SPECTRUM_LEN))//�ο�����
#define			SP_REFENCE_SPECTRUM_LEN			2048


#define			SP_WHITE_SPECTRUM_ADDR			((SP_REFENCE_SPECTRUM_ADDR)+(SP_REFENCE_SPECTRUM_LEN))//�׹���
#define			SP_WHITE_SPECTRUM_LEN			2048

//ϵ�����ʮ��
//220���ֽ� 0x100~0x1dc

#define			SP_RATIO_ADDR					0x0100			//ϵ����ŵĿ�ʼ��ַ
#define			SP_RATIO_LEN					22				//ϵ������

//���ڻָ��������õ�ϵ��
//��ַ 0x01dc~0x02b8
#define			SP_RATIO_FACTORY_ADDR			0x01dc

//0x02b9 ~ 0x2f5
//ÿ��������������ֵ
//һ��ϵ��λ
//��6���ֽ�
//10����
//60
#define 		SP_MATERIAL_ADDR				0x02b9
#define			SP_MATERIAL_LEN					6

#endif

