#ifndef ADDR_MAP_H
#define ADDR_MAP_H

#define			SP_SLAVE_ADDR					0x0000		//从机地址
#define			SP_SLAVE_LEN					1

//0x0001~0x0002
#define			SP_CLEAR_PERIOD_ADDR					0x0001	//清洁周期
#define			SP_CLEAR_PERIOD_LEN				2

#define			SP_DARK_SPECTRUM_ADDR			0x1000	//暗光谱开始位置
#define			SP_DARK_SPECTRUM_LEN			2048	//暗光谱长度

#define			SP_REFENCE_SPECTRUM_ADDR		((SP_DARK_SPECTRUM_ADDR)+(SP_DARK_SPECTRUM_LEN))//参考光谱
#define			SP_REFENCE_SPECTRUM_LEN			2048


#define			SP_WHITE_SPECTRUM_ADDR			((SP_REFENCE_SPECTRUM_ADDR)+(SP_REFENCE_SPECTRUM_LEN))//白光谱
#define			SP_WHITE_SPECTRUM_LEN			2048

//系数最多十个
//220个字节 0x100~0x1dc

#define			SP_RATIO_ADDR					0x0100			//系数存放的开始地址
#define			SP_RATIO_LEN					22				//系数长度

//用于恢复出厂设置的系数
//地址 0x01dc~0x02b8
#define			SP_RATIO_FACTORY_ADDR			0x01dc

//0x02b9 ~ 0x2f5
//每种物质两个波长值
//一个系数位
//共6个字节
//10物质
//60
#define 		SP_MATERIAL_ADDR				0x02b9
#define			SP_MATERIAL_LEN					6

#endif

