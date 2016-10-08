#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/gpio.h>
#endif
#include "lcm_drv.h"


#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
/* #include <mach/mt_pm_ldo.h> */
/* #include <mach/mt_gpio.h> */
#endif

#ifdef BUILD_LK
#define LCD_DEBUG(fmt)  dprintf(CRITICAL, fmt)
#else
#define LCD_DEBUG(fmt)  printk(fmt)
#endif

static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)			(lcm_util.set_reset_pin((v)))
#define MDELAY(n)					(lcm_util.mdelay(n))

/* --------------------------------------------------------------------------- */
/* Local Functions */
/* --------------------------------------------------------------------------- */
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
	lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) \
	lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
	lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
	lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
	lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

static unsigned int lcm_compare_id(void);


static const unsigned char LCD_MODULE_ID = 0x01;
/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */
#define LCM_DSI_CMD_MODE								0	/* 1 */
#define FRAME_WIDTH										(1080)
#define FRAME_HEIGHT									(1920)

#define REGFLAG_DELAY									0xFC
#define REGFLAG_END_OF_TABLE							0xFD

#ifdef BUILD_LK
#define LCD_BIAS_EN_PIN 				GPIO4
#define LCM_RESET_PIN 					GPIO146

#else

#define LCD_BIAS_EN_PIN				4
#define LCM_RESET_PIN 					146

#endif



#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/* --------------------------------------------------------------------------- */
/* Local Variables */
/* --------------------------------------------------------------------------- */

struct LCM_setting_table {
	unsigned char cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28, 0, {} },
	{0x10, 0, {} },
	{REGFLAG_DELAY, 120, {} }
};

static struct LCM_setting_table lcm_initialization_setting[] = {
		{0x00,1,{0x00}},
		{0xFF,4,{0x19,0x01,0x01,0x00}},
		{REGFLAG_DELAY, 10, {}},

		{0x00,1,{0x80}},
		{0xFF,2,{0x19,0x01}},
		{REGFLAG_DELAY, 10, {}},


		{0x00,1,{0x00}},
		{0x1C,1,{0x33}},//Can not OTP
		{REGFLAG_DELAY, 10, {}},

		{0x00,1,{0xA0}},
		{0xC1,1,{0xE8}},
		{REGFLAG_DELAY, 10, {}},

//For compression -->BP Offset Fixed Code 
		{0x00,1,{0xA7}},
		{0xC1,1,{0x00}},
		{REGFLAG_DELAY, 10, {}},

//Shift1 2 3 -->Shift Fixed Code
		{0x00,1,{0x90}},
		{0xC0,6,{0x00,0x2F,0x00,0x00,0x00,0x01}},
		{REGFLAG_DELAY, 10, {}},
		
//Shift1 2 3 -->Shift Fixed Code
		{0x00,1,{0xC0}},
		{0xC0,6,{0x00,0x2F,0x00,0x00,0x00,0x01}},
		{REGFLAG_DELAY, 10, {}},

		{0x00,1,{0x9A}},
		{0xC0,1,{0x1E}},
		{REGFLAG_DELAY, 10, {}},


		{0x00,1,{0xAC}},
		{0xC0,1,{0x06}},
		{REGFLAG_DELAY, 10, {}},

		{0x00,1,{0xDC}},
		{0xC0,1,{0x06}},
		{REGFLAG_DELAY, 10, {}},


		{0x00,1,{0x81}},  
		{0xA5,1,{0x04}}, 
		{REGFLAG_DELAY, 10, {}},


		{0x00,1,{0x92}},    
		{0xE9,1,{0x00}}, 
		{REGFLAG_DELAY, 10, {}},

		{0x00,1,{0x90}},    
		{0xF3,1,{0x01}}, 
		{REGFLAG_DELAY, 10, {}},

		{0x00,1,{0x93}},
		{0xC5,1,{0x1E}},
		{REGFLAG_DELAY, 10, {}},

		{0x00,1,{0x95}},
		{0xC5,1,{0x32}},
		{REGFLAG_DELAY, 10, {}},
		
		{0x00,1,{0x97}},
		{0xC5,1,{0x19}},
		{REGFLAG_DELAY, 10, {}},

		{0x00,1,{0x99}},
		{0xC5,1,{0x2D}},
		{REGFLAG_DELAY, 10, {}},

		{0x00,1,{0x00}},
		{0xD8,2,{0x1F,0x1F}},
		{REGFLAG_DELAY, 10, {}},

		{0x00,1,{0xB3}},
		{0xC0,1,{0xCC}},
		
		//	{0x00,1,{0xB3}},
		//{0xC0,1,{0x7C}},
		{REGFLAG_DELAY, 10, {}},

		{0x00,1,{0xBC}},
		{0xC0,1,{0xCC}},
		{REGFLAG_DELAY, 10, {}},

		{0x00,1,{0xCD}},//ltps power
		{0xF5,1,{0x19}},//reg_analog_en_ltps_pwr_sel_l
		{REGFLAG_DELAY, 10, {}},       

		{0x00,1,{0xDB}},  
		{0xF5,1,{0x19}},//reg_analog_gs_en_lsh_sel_l
		{REGFLAG_DELAY, 10, {}},

		{0x00,1,{0xF5}}, 
		{0xC1,1,{0x40}},//reg_ltps_gdpch_avdd=1'b0=c1f6[6]
		{REGFLAG_DELAY, 10, {}},

		{0x00,1,{0xB9}},//enable power off 2 frame
		{0xC0,1,{0x11}},//reg_tcon_f_powof_2=2'h0=c0ba[1:0]
		{REGFLAG_DELAY, 10, {}},

		{0x00,1,{0x8D}},//VCOM shift
		{0xF5,1,{0x20}},//reg_analog_en_vcom_sel_l
		{REGFLAG_DELAY, 10, {}},

		{0x00,1,{0x80}},
		{0xC0,14,{0x00,0x87,0x00,0x04,0x06,0x00,0x87,0x04,0x06,0x00,0x87,0x00,0x04,0x06}},

		{0x00,1,{0xF0}},
		{0xC3,6,{0x00,0x00,0x00,0x00,0x00,0x80}},

		{0x00,1,{0xA0}},
		{0xC0,7,{0x00,0x00,0x0C,0x00,0x00,0x1D,0x06}},

		{0x00,1,{0xD0}},
		{0xC0,7,{0x00,0x00,0x0C,0x00,0x00,0x1D,0x06}},

		{0x00,1,{0x90}},
		{0xC2,4,{0x84,0x01,0x45,0x45}},

		{0x00,1,{0x80}},
		{0xC3,12,{0x85,0x03,0x03,0x01,0x00,0x02,0x83,0x04,0x03,0x01,0x00,0x02}},

		{0x00,1,{0x90}},
		{0xC3,12,{0x84,0x04,0x03,0x01,0x00,0x02,0x82,0x04,0x03,0x01,0x00,0x02}},

		{0x00,1,{0x80}},
		{0xCC,15,{0x09,0x0A,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x28,0x28,0x28,0x28,0x28}},

		{0x00,1,{0x90}},
		{0xCC,15,{0x09,0x0A,0x14,0x13,0x12,0x11,0x18,0x17,0x16,0x15,0x28,0x28,0x28,0x28,0x28}},

		{0x00,1,{0xA0}},
		{0xCC,15,{0x1D,0x1E,0x1F,0x19,0x1A,0x1B,0x1C,0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27}},

		{0x00,1,{0xB0}},
		{0xCC,8,{0x01,0x02,0x03,0x05,0x06,0x07,0x04,0x08}},

		{0x00,1,{0xC0}},
		{0xCC,12,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x70}},

		{0x00,1,{0xD0}},
		{0xCC,12,{0xC0,0x0C,0x00,0x00,0x05,0xC0,0x00,0x00,0x33,0x03,0x00,0x70}},

		{0x00,1,{0x80}},//modify CLK=CKV1=CB82=0xFF->0x00, XCLK=CKV4=CB85=0x00->0xFF
		{0xCB,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
		
		{0x00,1,{0x90}},
		{0xCB,15,{0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

		{0x00,1,{0xA0}},
		{0xCB,15,{0x15,0x00,0x05,0xF5,0x05,0xF5,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

		{0x00,1,{0xB0}},
		{0xCB,15,{0x00,0x01,0xFD,0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

		{0x00,1,{0xC0}},
		{0xCB,8,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x77}},

		{0x00,1,{0xD0}},
		{0xCB,8,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x77}},

		{0x00,1,{0xE0}},
		{0xCB,8,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x77}},

		{0x00,1,{0xF0}},
		{0xCB,8,{0x11,0x11,0x11,0x00,0x00,0x00,0x00,0x77}},

		{0x00,1,{0x80}},
		{0xCD,15,{0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x01,0x12,0x11,0x03,0x04,0x3F,0x17}},

		{0x00,1,{0x90}},
		{0xCD,11,{0x18,0x3F,0x3D,0x25,0x25,0x25,0x1F,0x20,0x21,0x26,0x26}},

		{0x00,1,{0xA0}},
		{0xCD,15,{0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x01,0x12,0x11,0x05,0x06,0x3F,0x17}},

		{0x00,1,{0xB0}},
		{0xCD,11,{0x18,0x3F,0x3D,0x25,0x25,0x25,0x1F,0x20,0x21,0x26,0x26}},
		{REGFLAG_DELAY, 10, {}},
		
				{0x00,1,{0x00}},
		{0xD8,2,{0x1F,0x1F}},
		{REGFLAG_DELAY, 10, {}},
		{0x00,1,{0x00}},
		{0xD9,2,{0x00,0xB6}},
		{REGFLAG_DELAY, 10, {}},

		{0x00,1,{0x00}},
		{0xE1,24,{0x6B,0x6D,0x6E,0x70,0x73,0x75,0x79,0x81,0x82,0x8D,0x92,0x97,0x66,0x64,0x61,0x53,0x43,0x34,0x2A,0x24,0x1E,0x16,0xF,0x3}},
		{0x00,1,{0x00}},
		{0xE2,24,{0x6B,0x6D,0x6E,0x70,0x73,0x75,0x79,0x81,0x82,0x8D,0x92,0x97,0x66,0x64,0x61,0x53,0x43,0x34,0x2A,0x24,0x1E,0x16,0xF,0x3}},
		{0x00,1,{0x00}},
		{0xE3,24,{0x4A,0x4E,0x50,0x54,0x59,0x5C,0x63,0x6F,0x74,0x83,0x8C,0x92,0x69,0x66,0x63,0x55,0x45,0x36,0x2A,0x24,0x1E,0x14,0xF,0x3}},
		{0x00,1,{0x00}},
		{0xE4,24,{0x4A,0x4E,0x50,0x54,0x59,0x5C,0x63,0x6F,0x74,0x83,0x8C,0x92,0x69,0x66,0x63,0x55,0x45,0x36,0x2A,0x24,0x1E,0x14,0xF,0x3}},
		{0x00,1,{0x00}},
		{0xE5,24,{0x1,0xC,0x19,0x27,0x31,0x39,0x46,0x58,0x63,0x77,0x83,0x8C,0x6D,0x69,0x66,0x58,0x47,0x37,0x2C,0x26,0x1E,0x14,0xF,0x6}},
		{0x00,1,{0x00}},
		{0xE6,24,{0x1,0xC,0x19,0x27,0x31,0x39,0x46,0x58,0x63,0x77,0x83,0x8C,0x6D,0x69,0x66,0x58,0x47,0x37,0x2C,0x26,0x1E,0x14,0xF,0x6}},

#if 1
		{0x00,1,{0x89}},
		{0xF3,1,{0x5A}},
		{0x00,1,{0x90}},
		{0xF3,1,{0x01}},
		{0x00,1,{0x82}},
		{0xA5,1,{0x1F}},
		{0x00,1,{0xC2}},
		{0xC5,1,{0x1C}},
		{0x00,1,{0x80}}, 
		{0xc4,1,{0x06}},
		{0x00,1,{0xC1}},
		{0xF5,1,{0x15}},  
		{0x00,1,{0xC3}},
		{0xF5,1,{0x15}},  
		{0x00,1,{0xC9}},
		{0xF5,1,{0x15}},
		{0x00,1,{0xCB}},
		{0xF5,1,{0x15}},
		{0x00,1,{0xCD}},  
		{0xF5,1,{0x15}},  
		{0x00,1,{0x97}},  
		{0xF5,1,{0x19}},  
		{0x00,1,{0x99}},
		{0xF5,1,{0x19}},  
		{0x00,1,{0xB9}},
		{0xC0,1,{0x11}},
		{0x00,1,{0x8D}},
		{0xF5,1,{0x20}},  
		{0x00,1,{0xDB}},
		{0xf5,1,{0x19}},
#endif

	//	{0x00,1,{0x00}},
	 //	{0xFF,3,{0xFF,0xFF,0xFF}}, 
	{0x11,1,{0x00}},
	{REGFLAG_DELAY, 120, {}},
	{0x29,1,{0x00}},	
	{REGFLAG_DELAY, 20, {}},
	
							{0x22,0,{0x00}},                  
            {REGFLAG_DELAY, 150, {}}, 
			
										{0x22,0,{0x00}},                  
            {REGFLAG_DELAY, 150, {}}, 
            
            {0x00,1,{0xB3}},         
			{0xC0,1,{0xcc}}, //Initial must 1+2dot
			    	
			    	{0x00,1,{0x00}},             //Orise mode disable
			    	{0xff,3,{0xff,0xff,0xff}}, 
            
						{0x13,0,{0x00}}, 
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

#ifdef BUILD_LK
static struct LCM_setting_table page1_select[] = {
	//CMD_Page 1
	{0xFF, 3,{0x98,0x81,0x01}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif

static void push_table(struct LCM_setting_table *table, unsigned int count,
		       unsigned char force_update)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		unsigned cmd;

		cmd = table[i].cmd;

		switch (cmd) {

		case REGFLAG_DELAY:
			if (table[i].count <= 10)
				MDELAY(table[i].count);
			else
				MDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

/* --------------------------------------------------------------------------- */
/* LCM Driver Implementations */
/* --------------------------------------------------------------------------- */

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode = CMD_MODE;
#else
	params->dsi.mode = SYNC_PULSE_VDO_MODE;
#endif

	params->dsi.LANE_NUM = LCM_FOUR_LANE;

	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	params->dsi.packet_size = 256;

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active = 2;
	params->dsi.vertical_backporch = 18;
	params->dsi.vertical_frontporch = 10;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 10;
	params->dsi.horizontal_backporch = 100;
	params->dsi.horizontal_frontporch = 40;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	/* params->dsi.ssc_disable                                                       = 1; */
#if (LCM_DSI_CMD_MODE)
	params->dsi.PLL_CLOCK = 200;
#else
	params->dsi.PLL_CLOCK = 450;
#endif
}


static void lcm_init(void)
{
#ifdef BUILD_LK
	mt_set_gpio_mode(LCD_BIAS_EN_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(LCD_BIAS_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(LCD_BIAS_EN_PIN, GPIO_OUT_ONE);	

	mt_set_gpio_mode(LCM_RESET_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(LCM_RESET_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(LCM_RESET_PIN, GPIO_OUT_ONE);
	MDELAY(1);
	
	mt_set_gpio_out(LCM_RESET_PIN, GPIO_OUT_ZERO);
	MDELAY(20);
	
	mt_set_gpio_out(LCM_RESET_PIN, GPIO_OUT_ONE);
	MDELAY(100);
	
#else
	gpio_set_value(LCD_BIAS_EN_PIN, 1);	
	gpio_set_value(LCM_RESET_PIN, 1);
	MDELAY(1);

	gpio_set_value(LCM_RESET_PIN, 0);
	MDELAY(20);

	gpio_set_value(LCM_RESET_PIN, 1);
	MDELAY(100);
#endif

	push_table(lcm_initialization_setting,
		   sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
	push_table(lcm_suspend_setting,
		   sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);

#ifdef BUILD_LK

	mt_set_gpio_mode(LCD_BIAS_EN_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(LCD_BIAS_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(LCD_BIAS_EN_PIN, GPIO_OUT_ZERO);

	mt_set_gpio_mode(LCM_RESET_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(LCM_RESET_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(LCM_RESET_PIN, GPIO_OUT_ONE);
	MDELAY(10);
	mt_set_gpio_out(LCM_RESET_PIN, GPIO_OUT_ZERO);
	MDELAY(10);

#else
	gpio_set_value(LCM_RESET_PIN, 1);
	MDELAY(10);

	gpio_set_value(LCM_RESET_PIN, 0);
	MDELAY(10);
	
	gpio_set_value(LCD_BIAS_EN_PIN, 0);
#endif
}

static void lcm_resume(void)
{
	lcm_init();
}

#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);
	unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
	unsigned char y0_LSB = (y0 & 0xFF);
	unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
	unsigned char y1_LSB = (y1 & 0xFF);

	unsigned int data_array[16];

	data_array[0] = 0x00053902;
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00053902;
	data_array[1] = (y1_MSB << 24) | (y0_LSB << 16) | (y0_MSB << 8) | 0x2b;
	data_array[2] = (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
}
#endif

#define LCM_ID (0x0119)

static unsigned int lcm_compare_id(void)
{
	unsigned int id = 0;
	
#ifdef BUILD_LK
	unsigned int buffer[5];
	unsigned int array[16];
	
	mt_set_gpio_mode(LCD_BIAS_EN_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(LCD_BIAS_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(LCD_BIAS_EN_PIN, GPIO_OUT_ONE);

	mt_set_gpio_mode(LCM_RESET_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(LCM_RESET_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(LCM_RESET_PIN, GPIO_OUT_ONE);
	MDELAY(10);
	
	mt_set_gpio_out(LCM_RESET_PIN, GPIO_OUT_ZERO);
	MDELAY(50);
	
	mt_set_gpio_out(LCM_RESET_PIN, GPIO_OUT_ONE);
	MDELAY(50);

	push_table(page1_select, sizeof(page1_select) / sizeof(struct LCM_setting_table), 1);

	array[0] = 0x00043700;	// read id return 4 byte
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0xA1, buffer, 4);
	
    id = (buffer[0]>>16); //use the 4th paramater here
	dprintf(0, "%s, LK otm1901 debug: otm1901 id = 0x%08x\n", __func__, id);
	
#else
	gpio_set_value(LCD_BIAS_EN_PIN, 1);	
	gpio_set_value(LCM_RESET_PIN, 1);
	MDELAY(10);

	gpio_set_value(LCM_RESET_PIN, 0);
	MDELAY(50);

	gpio_set_value(LCM_RESET_PIN, 1);
	MDELAY(50);
	/*****no need to read id in kernel*****/
	printk("%s, Kernel  read otm1901 id but do not thing\n", __func__);
#endif

	if (id == LCM_ID)
		return 1;
	else
		return 0;

}

LCM_DRIVER boyi_otm1901_fhd_dsi_vdo_lcm_drv = {
	.name = "boyi_otm1901_fhd_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.compare_id = lcm_compare_id,
#if (LCM_DSI_CMD_MODE)
	.update = lcm_update,
#endif
};
