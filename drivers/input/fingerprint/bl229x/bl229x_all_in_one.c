#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/spi/spidev.h>
#include <linux/semaphore.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/ioctl.h>
#include <linux/version.h>
#include <linux/wait.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/kobject.h>
#include <linux/signal.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif
#include <linux/delay.h>
#if defined(CONFIG_HAS_EARLYSUSPEND) 
#include <linux/earlysuspend.h>
#endif

#ifndef CONFIG_ARCH_MSM
#define ARCH_MTK_BTL
#else
#undef ARCH_MTK_BTL
#endif
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/of_gpio.h>
#if defined(ARCH_MTK_BTL)
#include <mach/irqs.h>
#include <mt_spi.h>
#include <mt_gpio.h>
#include <mach/emi_mpu.h>
#include <mach/mt_clkmgr.h>
#include <mach/gpio_const.h>
//#include <cust_eint.h>
//#include <cust_gpio_usage.h>
#endif

#define SPI_DRV_NAME	"bl229x"

#define bl229x_height  96
#define bl229x_width   112
#define bl229x_image_size (bl229x_height * bl229x_width)
#define BL229X_SPI_CLOCK_SPEED 6*1000*1000//10*1000*1000

#define READIMAGE_BUF_SIZE	(12288) 
#define BL229X_IOCTL_MAGIC_NO			0xFC

#define INIT_BL229X				_IO(BL229X_IOCTL_MAGIC_NO, 0)
#define BL229X_GETIMAGE			_IOW(BL229X_IOCTL_MAGIC_NO, 1,uint32_t)
#define BL229X_INITERRUPT_MODE		_IOW(BL229X_IOCTL_MAGIC_NO, 2, uint32_t)

#define BL229X_AGC_CADJUST1     _IOW(BL229X_IOCTL_MAGIC_NO, 3, unsigned int)
#define BL229X_AGC_CADJUST2				_IOW(BL229X_IOCTL_MAGIC_NO, 3, uint8_t)

#define BL229X_POWERDOWN_MODE1			_IO (BL229X_IOCTL_MAGIC_NO,4)
#define BL229X_POWERDOWN_MODE2			_IO (BL229X_IOCTL_MAGIC_NO,5)

#define BL229X_INTERRUPT_FLAGS1     _IOW(BL229X_IOCTL_MAGIC_NO, 4, uint32_t)
#define BL229X_INTERRUPT_FLAGS2     _IOW(BL229X_IOCTL_MAGIC_NO, 5, uint32_t)


//selftest
#define RESET_PIN_FAILED	(1)
#define SPI_PIN_FAILED		(2)
#define INT_PIN_FAILED		(3)

#define PRESS_MAX_TIMEOUT_COUNT	(50)
#define ESD_RECOVERY_RETRY_TIMES	(3)

static int key_interrupt = KEY_F10;
static int keycode_last_saved = KEY_CAMERA;
int keycode = KEY_CAMERA;
int report_delay = 20;
int press_timeout_count = 0;

int is_busy = 0;

static int bl229x_log = 1;
//module_param(bl229x_log, int, 00664);
#define BTL_DEBUG(fmt,arg...)          do{\
	if(bl229x_log)\
	printk("<<btl-dbg>>[%s:%d]"fmt"\n",__func__, __LINE__, ##arg);\
}while(0)

int is_eint_enabled = 0;
static void mt_eint_mask(unsigned int eint_num)
{
	if(is_eint_enabled == 1)
	{
		is_eint_enabled = 0;
		disable_irq(eint_num);
	}
}
static void mt_eint_unmask(unsigned int eint_num)
{
	if(is_eint_enabled == 0)
	{
		enable_irq(eint_num);
		is_eint_enabled = 1;
	}
}

struct bl229x_data {
	struct spi_device *spi;
	u8 *image_buf;
	struct semaphore mutex;
	u32 reset_gpio;
	u32 irq_gpio;
	u32 irq_num;
	u32 power_en_gpio;
	
	struct pinctrl *pinctrl1;
	struct pinctrl_state *spi_pins_default;
	struct pinctrl_state *power_en_output0;
	struct pinctrl_state *power_en_output1;
	struct pinctrl_state *rst_output0;
	struct pinctrl_state *rst_output1;
	struct pinctrl_state *int_default;

	struct notifier_block fb_notify;
};

static DEFINE_MUTEX(device_list_lock);
static LIST_HEAD(device_list);

//driver init
static int  mt_spi_init(void);
static void  mt_spi_exit(void);
static int   bl229x_probe(struct spi_device *spi);
static int  bl229x_open(struct inode *inode, struct file *file);
static ssize_t bl229x_write(struct file *file, const char *buff,size_t count, loff_t *ppos);
static ssize_t bl229x_read(struct file *file, char *buff,size_t count, loff_t *ppos);
static long bl229x_ioctl(struct file *filp, unsigned int cmd,unsigned long arg) ;
//static long bl229x_compat_ioctl(struct file *filp, unsigned int cmd,unsigned long arg);
static int bl229x_release(struct inode *inode, struct file *file);

static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data);


//spi func and dev init spi cmd
static int spi_send_cmd(struct bl229x_data *bl229x,u8 *tx,u8 *rx,u16 spilen);

static int  bl229x_dev_init(struct bl229x_data *spidev);
static int bl229x_read_image(struct bl229x_data *bl229x,uint32_t timeout);
static int  bl229x_dev_interrupt_init(struct bl229x_data *bl229x);
static int bl229x_clear_interrupt(struct bl229x_data *bl229x);
static int bl229x_agc_init(struct bl229x_data *bl229x,unsigned long arg);

//hardware init gpio and irq, dma
//static int bl229x_eint_gpio_init(struct bl229x_data *bl229x);
irqreturn_t bl229x_eint_handler(int irq,void *data);

static int bl229x_create_inputdev(void);
static int bl229x_thread_func(void *unused);
static int mtspi_set_dma_en(int mode);
static int bl229x_power_on(struct bl229x_data *bl229x,bool enable);
static int bl229x_gpio_select_and_init(struct bl229x_data *bl229x);
static int bl229x_parse_dt(struct device *dev,struct bl229x_data *pdata);
static void bl229x_powerdown_mode(struct bl229x_data* bl229x);

static int bl229x_suspend(struct device *dev);
static int bl229x_resume(struct device *dev);

static int bl229x_async_fasync(int fd,struct file *filp,int mode);



static atomic_t suspended;
//图像数据大小
//static u8 imagebuf[10752]={0x00};
// 定义一个全局变量 bl229x 的数据结构
static struct bl229x_data *g_bl229x= NULL;
static struct input_dev *bl229x_inputdev = NULL;
static struct task_struct *bl229x_thread = NULL;
static struct kobject *bl229x_kobj=NULL;
//申明并初始化队列
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static int bl229x_interrupt_flag = 0;
static int interrupt_mode_flag = 0;
static u8 *imagetxcmd;//u8 imagetxcmd [12288] = {0x00} ;
static u8 *imagerxpix;//u8 imagerxpix [12288] = {0x00};
static u8 *imagebuf;
static unsigned int g_agc_value = 0xfd;
static unsigned int last_agc = 0xfc;

//Asynchronous notification struct
static struct fasync_struct *async_queue;

#if defined(ARCH_MTK_BTL)
static struct mt_chip_conf spi_conf=
{

	.setuptime = 10,
	.holdtime = 10,
	.high_time = 7, //此处决定slk的频率
	.low_time =  7,
	.cs_idletime = 10,
	//.ulthgh_thrsh = 0,

	.cpol = 0,
	.cpha = 0,

	.rx_mlsb = 1,  //先传高位
	.tx_mlsb = 1,

	.tx_endian = 0, //tx_endian 表示大端模式
	.rx_endian = 0,

	.com_mod = DMA_TRANSFER,
	.pause = 1,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,	


};
/*
static struct spi_board_info spi_board_bl229x[] __initdata = {
	[0] = {
		.modalias= SPI_DRV_NAME,
		.bus_num = 0,
		.chip_select=0,
		.mode = SPI_MODE_0,
		.max_speed_hz = BL229X_SPI_CLOCK_SPEED,
	},
};
*/
#endif

static const struct dev_pm_ops bl229x_pm = {
   .suspend = bl229x_suspend,
   .resume = bl229x_resume
};

static struct of_device_id bl229x_match_table[] = {
	{.compatible = "blestech,BL229X",},	
	{},
};

static struct spi_driver bl229x_driver = {
	.driver = {
		.name	= SPI_DRV_NAME,
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
      	.of_match_table = bl229x_match_table,
      	.pm = &bl229x_pm,
	},
	.probe	= bl229x_probe,
};

/*----------------------------------------------------------------------------*/
static const struct file_operations bl229x_fops = {
	.owner = THIS_MODULE,
	.open  = bl229x_open,
	.write = bl229x_write,
	.read  = bl229x_read,
	.release = bl229x_release,
	.unlocked_ioctl = bl229x_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = bl229x_ioctl,
#endif
    .fasync = bl229x_async_fasync,
};

/*----------------------------------------------------------------------------*/
static struct miscdevice bl229x_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = SPI_DRV_NAME,
	.fops = &bl229x_fops,
};

// 配置 IO 口使其工作在 SPI 模式 	
static void spi_io_set_mode(int enable)
{
	pinctrl_select_state(g_bl229x->pinctrl1, g_bl229x->spi_pins_default);
}

// 选择工作与那种模式
static int mtspi_set_dma_en(int mode)
{
#if defined(ARCH_MTK_BTL)
	struct mt_chip_conf* spi_par;
	spi_par = &spi_conf;
	if (!spi_par)
	{
		return -1;
	}
	if (1 == mode)
	{
		if (spi_par-> com_mod == DMA_TRANSFER)
		{
			return 0;
		}
		spi_par-> com_mod = DMA_TRANSFER;
	}
	else
	{
		if (spi_par-> com_mod == FIFO_TRANSFER)
		{
			return 0;
		}
		spi_par-> com_mod = FIFO_TRANSFER;
	}

	spi_setup(g_bl229x->spi);
#endif
	return 0;
}


static int spi_send_cmd(struct bl229x_data *bl229x,u8 *tx,u8 *rx,u16 spilen)
{
	int ret=0;
	struct spi_message m;
	struct spi_transfer t = {
		.cs_change = 0,
		.delay_usecs = 5,
		.speed_hz = BL229X_SPI_CLOCK_SPEED,
		.tx_buf = tx,
		.rx_buf = rx,
		.len = spilen,
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 0,
	};

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	ret= spi_sync(bl229x->spi,&m);
	return ret;   
}
#if 0
static int spi_send_cmd_fifo(struct bl229x_data *bl229x,u8 *tx,u8 *rx,u16 spilen)
{
	int ret=0;
	struct spi_message m;
	struct spi_transfer t = {
		.cs_change = 0,
		.delay_usecs = 5,
		.speed_hz = BL229X_SPI_CLOCK_SPEED,
		.tx_buf = tx,
		.rx_buf = rx,
		.len = spilen,
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 0,
	};
	mtspi_set_dma_en(0);  // fifo 模式
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	ret= spi_sync(bl229x->spi,&m);
	return ret;   
}
#endif
//-------------------------------------------------------------------------------------------------
static ssize_t bl229x_show_agc(struct device *ddri,struct device_attribute *attr,char *buf)
{
	//int i = 0;
	u8 reset [1] = {0x0c};
	u8 start[1]={0x18};

	u8 rx_data1[1]={0x00};
	u8 rx_data2[1]={0x00};
	u8 test_tx[7];
	u8 test_rx[7];

	//set_spi_mode(1);
	spi_send_cmd(g_bl229x,reset,rx_data1,1);
	spi_send_cmd(g_bl229x,reset,rx_data1,1);
	spi_send_cmd(g_bl229x,start,rx_data2,1);

	test_tx[0] = 0x21;
	test_tx[1] = 0x66;
	test_tx[2] = 0x66;
	test_tx[3] = 0xB5;
	test_tx[4] = 0x00;
	test_tx[5] = last_agc;
	test_tx[6] = 0x70; //0x50

	spi_send_cmd(g_bl229x,test_tx,test_rx,7);
	spi_send_cmd(g_bl229x,test_tx,test_rx,7);

	//---------------------------------------------
	//for(i=0;i<7;i++) BTL_DEBUG("%s test_rx[%d]=0x%x\n",__func__,i,test_rx[i]);

	//---------------------------------------------	
	interrupt_mode_flag = 0;
	return sprintf(buf,"g_agc_value=%x sky %x,%x,%x,%x,%x,%x,%x",g_agc_value,test_rx[0],test_rx[1],test_rx[2],    \
			test_rx[3],test_rx[4],test_rx[5],test_rx[6]);
}

static ssize_t bl229x_store_agc(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	char *next;

	g_agc_value = simple_strtoul(buf, &next, 16);

	return size;
}

static DEVICE_ATTR(agc,0664,bl229x_show_agc,bl229x_store_agc);

static int inttrupt_enabled;
static ssize_t bl229x_show_interrupt_mode(struct device *ddri,struct device_attribute *attr,char *buf)
{
	bl229x_clear_interrupt(g_bl229x);
	return sprintf(buf, "sky register rst%d =%x\n",g_bl229x->reset_gpio,gpio_get_value(g_bl229x->reset_gpio));
}

static ssize_t bl229x_store_interrupt_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	char *next;
	inttrupt_enabled = simple_strtoul(buf, &next, 10);
	bl229x_power_on(g_bl229x,inttrupt_enabled);
	
	if(inttrupt_enabled)
		mt_eint_unmask(g_bl229x->irq_num);
	else
		mt_eint_mask(g_bl229x->irq_num);
	return size;
}

static DEVICE_ATTR(interrupt,0664,bl229x_show_interrupt_mode,bl229x_store_interrupt_mode);

static ssize_t bl229x_show_readimage(struct device *ddri,struct device_attribute *attr,char *buf)
{
	bl229x_read_image(g_bl229x,100);
	return 0;
}

static DEVICE_ATTR(readimage,S_IWUSR|S_IRUGO,bl229x_show_readimage,NULL);

static u8 spicmd_rsp[256];
static u32 param_len = 0;

static ssize_t bl229x_show_spicmd(struct device *ddri,struct device_attribute *attr,char *buf)
{
	int count = 0;
	int buflen = 0;
	for(count = 0; count < param_len; count++)
	{
		buflen += sprintf(buf + buflen, "rsp[%d]=%x\n", count, spicmd_rsp[count]);
	}

	return buflen;
}

static ssize_t bl229x_store_spicmd(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	char *next;
	int count=0;
	u8 param[256];

	param_len = simple_strtoul(buf, &next, 16);
	BTL_DEBUG("\n\n spicmd len=%d\n\n ",param_len);
	while(*next == '.')
	{
		param[count] = simple_strtoul(next+1, &next, 16);
		BTL_DEBUG("\nsky test param[%d]=%x \n ",count,param[count]);
		++count;
	}
	spi_send_cmd(g_bl229x, param, spicmd_rsp, count);

	for(count = 0; count < param_len; count++)
	{
		BTL_DEBUG("rsp[%d]=%x\n", count, spicmd_rsp[count]);
	}

	return size;
}
static DEVICE_ATTR(spicmd, 0664, bl229x_show_spicmd, bl229x_store_spicmd);


//
//async fun
static int bl229x_async_fasync(int fd,struct file *filp,int mode)
{
 return fasync_helper(fd,filp,mode,&async_queue);
}

//异步上报函数接口
static void bl229x_async_Report(void)
{   
   //Send signal to user space,POLL_IN is enable write
   printk("bl229x_async_Report \n");
   if (async_queue){
   	  printk("bl229x kill_fasync\n ");
      kill_fasync(&async_queue,SIGIO,POLL_IN);
   }
}

//keycode
static ssize_t bl229x_show_keycode(struct device *ddri,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "\nkeycode=%d\n", keycode);
}

static ssize_t bl229x_store_keycode(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	char *next;
	keycode = simple_strtoul(buf, &next, 10);
	keycode_last_saved = keycode;
	is_busy = 0;
	bl229x_dev_interrupt_init(g_bl229x);
	mt_eint_unmask(g_bl229x->irq_num);
	return size;
}
static DEVICE_ATTR(keycode, 0664, bl229x_show_keycode, bl229x_store_keycode);

//key_interrupt
static ssize_t bl229x_show_key_interrupt(struct device *ddri,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "\nkey_interrupt=%d\n", key_interrupt);
}

static ssize_t bl229x_store_key_interrupt(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	char *next;
	key_interrupt = simple_strtoul(buf, &next, 10);
	return size;
}
static DEVICE_ATTR(key_interrupt, 0664, bl229x_show_key_interrupt, bl229x_store_key_interrupt);

//report_delay
static ssize_t bl229x_show_report_delay(struct device *ddri,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "\nreport_delay=%d\n", report_delay);
}

static ssize_t bl229x_store_report_delay(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	char *next;
	report_delay = simple_strtoul(buf, &next, 10);
	return size;
}
static DEVICE_ATTR(report_delay, 0664, bl229x_show_report_delay, bl229x_store_report_delay);

static int hw_reset(struct bl229x_data *bl229x)
{
	//test reset pin
	u32 pin_val= -1;
	bl229x_power_on(bl229x, 0);
	msleep(200);
	pin_val = gpio_get_value(bl229x->reset_gpio);
	if(GPIO_OUT_ZERO != pin_val)
		return -RESET_PIN_FAILED;
	BTL_DEBUG("%s rst pin_val=%d\n",__func__,pin_val);
	bl229x_power_on(bl229x, 1);
	pin_val = gpio_get_value(bl229x->reset_gpio);
	if(GPIO_OUT_ONE != pin_val)
		return -RESET_PIN_FAILED;
	BTL_DEBUG("%s rst pin_val=%d\n",__func__,pin_val);
	return 0;
}

static int recovey_from_esd_failed(struct bl229x_data *bl229x)
{
	int i = 0;
	int recovery_count = 0;
	u8 reset [1] = {0x0c};
	u8 start[1]={0x18};

	u8 rx_data1[1]={0x00};
	u8 rx_data2[1]={0x00};

	u8 test_tx[7];
	u8 test_rx[7];

	test_tx[0] = 0x21;
	test_tx[1] = 0x66;
	test_tx[2] = 0x66;
	test_tx[3] = 0xB5;
	test_tx[4] = 0x00;
	test_tx[5] = last_agc;
	test_tx[6] = 0x70; //0x50
	
	mt_eint_mask(g_bl229x->irq_num);
	do
	{
		if(recovery_count++ < ESD_RECOVERY_RETRY_TIMES )
		{
			BTL_DEBUG("recovery_count=%d\n",recovery_count);
			hw_reset(bl229x);
			spi_send_cmd(bl229x,reset,rx_data1,1);
			spi_send_cmd(bl229x,reset,rx_data1,1);
			spi_send_cmd(bl229x,start,rx_data2,1);

			spi_send_cmd(bl229x,test_tx,test_rx,7);
			spi_send_cmd(bl229x,test_tx,test_rx,7);
			for(i=0;i<7;i++) BTL_DEBUG("%s test_rx[%d]=0x%x\n",__func__,i,test_rx[i]);
		}
		else
			break;
	}
	while(test_rx[6]!=0x70);
	mt_eint_unmask(g_bl229x->irq_num);
	return 0;
}

static int  bl229x_dev_selftest(struct bl229x_data *bl229x)
{	
	//复位信号
	u32 pin_val= -1;
	//char *curbuf = testbuf;

	int i = 0;

	u8 reset [1] = {0x0c};
	u8 start[1]={0x18};

	u8 rx_data1[1]={0x00};
	u8 rx_data2[1]={0x00};

	u8 test_tx[7];
	u8 test_rx[7];
	
	interrupt_mode_flag = 0;
	//test reset pin
	hw_reset(bl229x);
	//test spi pins
	spi_send_cmd(bl229x,reset,rx_data1,1);
	spi_send_cmd(bl229x,reset,rx_data1,1);
	spi_send_cmd(bl229x,start,rx_data2,1);

	test_tx[0] = 0x21;
	test_tx[1] = 0x66;
	test_tx[2] = 0x66;
	test_tx[3] = 0x05;
	test_tx[4] = 0x00;
	test_tx[5] = 0x70;
	test_tx[6] = 0x70; //0x50

	spi_send_cmd(bl229x,test_tx,test_rx,7);
	spi_send_cmd(bl229x,test_tx,test_rx,7);
	//---------------------------------------------
	//for(i=0;i<7;i++) BTL_DEBUG("%s test_rx[%d]=0x%x\n",__func__,i,test_rx[i]);
	for(i = 3; i < 7;i++)
		if(test_rx[i] != test_tx[i])
			return -SPI_PIN_FAILED;
	//---------------------------------------------	

	//test interrupt pin
	bl229x_read_image(g_bl229x,100);
	pin_val = gpio_get_value(g_bl229x->irq_gpio);
	msleep(5);
	if(GPIO_OUT_ONE != pin_val)
		return -INT_PIN_FAILED;

	BTL_DEBUG("%s int pin_val=%d\n",__func__,pin_val);

	return 0;
}

static ssize_t bl229x_show_selftest(struct device *ddri,struct device_attribute *attr,char *buf)
{
	int ret = 0;
	mt_eint_mask(g_bl229x->irq_num);
	ret = bl229x_dev_selftest(g_bl229x);
	mt_eint_unmask(g_bl229x->irq_num);
	return sprintf(buf, "\nselftest=%d\n", ret);
}

static ssize_t bl229x_store_selftest(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	return size;
}
static DEVICE_ATTR(selftest, 0664, bl229x_show_selftest, bl229x_store_selftest);

static struct device_attribute *bl229x_attr_list[] = {
	&dev_attr_agc,
	&dev_attr_interrupt,
	&dev_attr_readimage,
	&dev_attr_spicmd,
	&dev_attr_keycode,
	&dev_attr_key_interrupt,
	&dev_attr_report_delay,
	&dev_attr_selftest
};

static void fps_create_attributes(struct device *dev)
{
	int num = (int)(sizeof(bl229x_attr_list)/sizeof(bl229x_attr_list[0]));
	for (; num > 0;)
		device_create_file(dev, bl229x_attr_list[--num]);
}

static int fps_sysfs_init(void)
{
	int ret;
	bl229x_kobj = kobject_create_and_add("bl229x_sysfs",NULL);
	if(bl229x_kobj == NULL)
	{
		BTL_DEBUG("%s  subsystem_register failed\n",__func__);
		ret = -ENOMEM;
		return ret;
	}

	ret = sysfs_create_file(bl229x_kobj,&dev_attr_agc.attr);
	ret = sysfs_create_file(bl229x_kobj,&dev_attr_interrupt.attr);
	ret = sysfs_create_file(bl229x_kobj,&dev_attr_readimage.attr);
	ret = sysfs_create_file(bl229x_kobj,&dev_attr_spicmd.attr);
	ret = sysfs_create_file(bl229x_kobj,&dev_attr_keycode.attr);
	ret = sysfs_create_file(bl229x_kobj,&dev_attr_key_interrupt.attr);
	ret = sysfs_create_file(bl229x_kobj,&dev_attr_report_delay.attr);
	ret = sysfs_create_file(bl229x_kobj,&dev_attr_selftest.attr);

	if(ret)
	{
		BTL_DEBUG("%s sysfs_create_file failed\n",__func__);
	}
	return ret;
}
//-------------------------------------------------------------------------------------------------
//控制函数 
static long bl229x_ioctl(struct file *filp, unsigned int cmd,unsigned long arg) 
{
	struct bl229x_data *bl229x = filp->private_data;
	struct spi_device *spi;
	int error=0;
	uint32_t user_regval = 0;

	if (_IOC_DIR(cmd) & _IOC_READ)
		error = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		error = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

	if (error) {
		BTL_DEBUG("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}
	
	spi = spi_dev_get(bl229x->spi) ;
	if (down_interruptible(&bl229x->mutex))
		return -ENOTTY;
	switch (cmd)
	{
		case INIT_BL229X:
			error= bl229x_dev_init(bl229x);
			break;
		case BL229X_GETIMAGE:
			error = bl229x_read_image(bl229x,arg);
			break;
		case BL229X_INITERRUPT_MODE:
			is_busy = 0;
			error = bl229x_dev_interrupt_init(bl229x);
			mt_eint_unmask(g_bl229x->irq_num);
			break; 
		case BL229X_AGC_CADJUST1:
		case BL229X_AGC_CADJUST2:
			is_busy = 1;
			mt_eint_mask(g_bl229x->irq_num);
			error= bl229x_agc_init(bl229x,arg);
			break;
        case BL229X_POWERDOWN_MODE1:
		case BL229X_POWERDOWN_MODE2:
			bl229x_powerdown_mode(g_bl229x);
			is_busy = 0;
			error = bl229x_dev_interrupt_init(bl229x);
			mt_eint_unmask(g_bl229x->irq_num);
			break;
		case BL229X_INTERRUPT_FLAGS1:
		case BL229X_INTERRUPT_FLAGS2:		
			user_regval = gpio_get_value(bl229x->irq_gpio);
		    if (copy_to_user((void __user*)arg, &user_regval, sizeof(user_regval)) != 0) {
                    error = -EFAULT;
                }
			break;	
		default:
			error = -ENOTTY;
			break;

	}
	up(&bl229x->mutex);
	return error;

}
//#ifdef CONFIG_COMPAT
#if 0
static long bl229x_compat_ioctl(struct file *filp, unsigned int cmd,unsigned long arg) 
{
	struct bl229x_data *bl229x = filp->private_data;
	struct spi_device *spi;
	int error=0;
	uint32_t user_regval = 0;

	
	void __user *arg32 = compat_ptr(arg);
	BTL_DEBUG("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));

	if (!filp->f_op || !filp->f_op->unlocked_ioctl)
		return -ENOTTY;
	
	spi = spi_dev_get(bl229x->spi) ;
	if (down_interruptible(&bl229x->mutex))
		return -ENOTTY;
	switch (cmd)
	{
		case INIT_BL229X:
			error= bl229x_dev_init(bl229x);
			break;
		case BL229X_GETIMAGE:
			error = bl229x_read_image(bl229x,arg32);
			break;
		case BL229X_INITERRUPT_MODE:
			is_busy = 0;
			error = bl229x_dev_interrupt_init(bl229x);
			mt_eint_unmask(g_bl229x->irq_num);
			break; 
		case BL229X_AGC_CADJUST1:
		case BL229X_AGC_CADJUST2:
			is_busy = 1;
			mt_eint_mask(g_bl229x->irq_num);
			error= bl229x_agc_init(bl229x,arg32);
			break;
        case BL229X_POWERDOWN_MODE1:
		case BL229X_POWERDOWN_MODE2:
			bl229x_powerdown_mode(g_bl229x);
			is_busy = 0;
			error = bl229x_dev_interrupt_init(bl229x);
			mt_eint_unmask(g_bl229x->irq_num);
			break;
		case BL229X_INTERRUPT_FLAGS1:
		case BL229X_INTERRUPT_FLAGS2:		
			user_regval = gpio_get_value(bl229x->irq_gpio);
		    if (copy_to_user((void __user*)arg32, &user_regval, sizeof(user_regval)) != 0) {
                    error = -EFAULT;
                }
			break;	
		default:
			error = -ENOTTY;
			break;

	}
	up(&bl229x->mutex);
	return error;

}
#endif
// 打开设备

static int bl229x_open(struct inode *inode, struct file *file)
{
	struct bl229x_data *bl229x;
	spi_io_set_mode(1);
	bl229x = g_bl229x;
	if (down_interruptible(&bl229x->mutex))
		return -ERESTARTSYS;
	file->private_data = bl229x;
	up(&bl229x->mutex);
	return 0;
}

//写操作，指纹传感不需要对其进行写操作 。故直接返回 操作
static ssize_t bl229x_write(struct file *file, const char *buff,size_t count, loff_t *ppos)
{
	return -ENOMEM;
}

// 读操作
static ssize_t bl229x_read(struct file *file, char  *buff,size_t count, loff_t *ppos)
{
	int ret=0;
	struct bl229x_data *bl229x = file->private_data;
	ssize_t status = 0;
	struct spi_device *spi;
	spi = spi_dev_get(bl229x->spi) ;
	ret = copy_to_user(buff,imagebuf,count);
	if (ret)
	{
		status = -EFAULT;
	}
	return status;
}

// 释放函数
static int bl229x_release(struct inode *inode, struct file *file)
{
	int status = 0 ;
	struct bl229x_data *bl229x = file->private_data;

	bl229x_async_fasync(-1, file, 0);  

	if (down_interruptible(&bl229x->mutex))
	{
		return -EFAULT;
	}
	if (!atomic_read(&file->f_count)) 
	{
		// 此处增加休眠工作的代
	}
	up(&bl229x->mutex);
	return status;
}

static int bl229x_suspend (struct device *dev)
{

	struct bl229x_data *bl229x = dev_get_drvdata(dev);
	dev_err (&bl229x->spi->dev,"[bl229x]%s\n", __func__);
	atomic_set(&suspended, 1);
	is_busy = 0;
   
   return 0;
}

/* -------------------------------------------------------------------- */
static int bl229x_resume (struct device *dev)
{
	struct bl229x_data *bl229x = dev_get_drvdata(dev);
	dev_err (&bl229x->spi->dev,"[bl229x]%s\n", __func__);
	is_busy = 0;
	atomic_set(&suspended, 0);
   return 0;
}


static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct fb_event *evdata = data;

	int *blank;


	if (evdata && evdata->data && event == FB_EVENT_BLANK )//&&

		{
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			bl229x_resume(&g_bl229x->spi->dev);
		else if (*blank == FB_BLANK_POWERDOWN)
			bl229x_suspend(&g_bl229x->spi->dev);
	}

	return 0;
}

static int is_connected(struct bl229x_data *bl229x)
{
	u8 reset [1] = {0x0c};
	u8 start[1]={0x18};

	u8 rx_data1[1]={0x00};
	u8 rx_data2[1]={0x00};
	u8 test_tx[7];
	u8 test_rx[7];

	//set_spi_mode(1);
	spi_send_cmd(bl229x,reset,rx_data1,1);
	spi_send_cmd(bl229x,reset,rx_data1,1);
	spi_send_cmd(bl229x,start,rx_data2,1);

	test_tx[0] = 0x21;
	test_tx[1] = 0x66;
	test_tx[2] = 0x66;
	test_tx[3] = 0xB5;
	test_tx[4] = 0x00;
	test_tx[5] = 0xfd;
	test_tx[6] = 0x70; //0x50

	spi_send_cmd(bl229x,test_tx,test_rx,7);
	spi_send_cmd(bl229x,test_tx,test_rx,7);
	if(test_tx[5] == test_rx[5])
		return 0;
	else return -1;
}

static int  bl229x_probe(struct spi_device *spi)
{
	struct bl229x_data *bl229x = NULL;
	int err = 0;
	g_bl229x = bl229x = kzalloc(sizeof(struct bl229x_data),GFP_KERNEL);
	if (!bl229x)
	{
		return -ENOMEM;
	}
		printk("lixf bl229 probe\n");
	//申请图像缓冲区空间 
	imagebuf = bl229x->image_buf = (u8*)__get_free_pages(GFP_KERNEL,get_order(READIMAGE_BUF_SIZE));
	imagetxcmd = (u8*)__get_free_pages(GFP_KERNEL,get_order(READIMAGE_BUF_SIZE));;
	imagerxpix = (u8*)__get_free_pages(GFP_KERNEL,get_order(READIMAGE_BUF_SIZE));;
	//清零
	memset(bl229x->image_buf,0x00,get_order(READIMAGE_BUF_SIZE));
	memset(imagetxcmd,0x00,get_order(READIMAGE_BUF_SIZE));
	memset(imagetxcmd,0x00,get_order(READIMAGE_BUF_SIZE));
	if (!bl229x->image_buf)
	{
		return -ENOMEM;
	}
	spi_set_drvdata(spi,bl229x);

	//spi io初始化 
	BTL_DEBUG("bl229x_probe 0\n");

	//spi 控制器参数，要与IC匹配
	bl229x->spi = spi;
	bl229x->spi->bits_per_word = 8;
	bl229x->spi->mode = SPI_MODE_0;
#if defined(ARCH_MTK_BTL)
	bl229x->spi->controller_data = (void*)&spi_conf;
#endif
	spi_setup(bl229x->spi); 

	bl229x_gpio_select_and_init(bl229x);

	//电源初始化
	bl229x_power_on(bl229x,1);

	err = is_connected(bl229x);
	if(err)
	{
		BTL_DEBUG("bl229x_dev_selftest failed\n");
		goto exit_misc_device_register_failed;
	}
	//注册input 设备 
	bl229x_create_inputdev();
	BTL_DEBUG("bl229x_probe 1\n");

	sema_init(&bl229x->mutex, 0);
	BTL_DEBUG("bl229x_probe 2\n");

	//注册字符设备ioctl，核心操作
	err = misc_register(&bl229x_misc_device);
	if(err)
	{
		BTL_DEBUG("bl229x_misc_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	// 将bl229x 的数据保存在全局变量中
	g_bl229x = bl229x;
	//初始化中断引脚  
	//bl229x_eint_gpio_init(bl229x);
	err = request_threaded_irq(spi->irq, NULL, bl229x_eint_handler, IRQF_TRIGGER_HIGH  | IRQF_ONESHOT, SPI_DRV_NAME, bl229x);
	bl229x->irq_num = spi->irq;

	//spi dma or fifo 方式
	mtspi_set_dma_en(0);

	BTL_DEBUG("bl229x_probe 3\n");
	//debug 调试节点
	fps_sysfs_init();
	BTL_DEBUG("bl229x_probe 31\n");

	fps_create_attributes(&spi->dev);
	//注册睡眠唤醒函数
	BTL_DEBUG("bl229x_probe 32\n");
	bl229x->fb_notify.notifier_call = fb_notifier_callback;

	err = fb_register_client(&bl229x->fb_notify);

	if (err)
		dev_err(&bl229x->spi->dev, "Unable to register fb_notifier: %d\n",
			err);

	up(&bl229x->mutex); 
	is_busy = 0;
	bl229x_dev_interrupt_init(bl229x);
	return 0;

exit_misc_device_register_failed:
	kfree(bl229x);
	return -1;
}

// bl229x 初始化
static int  bl229x_dev_init(struct bl229x_data *bl229x)
{
	int i = 0;
	u8 reset [1] = {0x0c};
	u8 start[1]={0x18};

	u8 rx_data1[1]={0x00};
	u8 rx_data2[1]={0x00};

	u8 test_tx[7];
	u8 test_rx[7];

	spi_io_set_mode(1);
	spi_send_cmd(bl229x,reset,rx_data1,1);
	spi_send_cmd(bl229x,reset,rx_data1,1);
	spi_send_cmd(bl229x,start,rx_data2,1);

	test_tx[0] = 0x21;
	test_tx[1] = 0x66;
	test_tx[2] = 0x66;
	test_tx[3] = 0xB5;
	test_tx[4] = 0x00;
	test_tx[5] = 0xf4;
	test_tx[6] = 0x70; //0x50

	spi_send_cmd(bl229x,test_tx,test_rx,7);
	spi_send_cmd(bl229x,test_tx,test_rx,7);
	//---------------------------------------------
	//for(i=0;i<7;i++) BTL_DEBUG("%s test_rx[%d]=0x%x\n",__func__,i,test_rx[i]);
	for(i = 3; i < 7;i++)
	{
		if(test_rx[i] != test_tx[i])
		{
			BTL_DEBUG("esd check find error\n");
			//hw_reset(bl229x);
			recovey_from_esd_failed(bl229x);
			break;
		}
	}
	//---------------------------------------------
	interrupt_mode_flag = 0;
	return 0;
}


static int bl229x_agc_init(struct bl229x_data *bl229x,unsigned long arg)
{
	int i = 0;
	u8 reset [1] = {0x0c};
	u8 start[1]={0x18};

	u8 rx_data1[1]={0x00};
	u8 rx_data2[1]={0x00};
	u8 test_tx[7];
	u8 test_rx[7];

	//set_spi_mode(1);
	spi_send_cmd(bl229x,reset,rx_data1,1);
	spi_send_cmd(bl229x,reset,rx_data1,1);
	spi_send_cmd(bl229x,start,rx_data2,1);

	test_tx[0] = 0x21;
	test_tx[1] = 0x66;
	test_tx[2] = 0x66;
	test_tx[3] = 0xB5;
	test_tx[4] = 0x00;
	last_agc = test_tx[5] = arg;
	test_tx[6] = 0x70; //0x50

	spi_send_cmd(bl229x,test_tx,test_rx,7);
	spi_send_cmd(bl229x,test_tx,test_rx,7);

	//---------------------------------------------
	//for(i=0;i<7;i++) BTL_DEBUG("%s test_rx[%d]=0x%x\n",__func__,i,test_rx[i]);
	for(i = 3; i < 7;i++)
	{
		if(test_rx[i] != test_tx[i])
		{
			BTL_DEBUG("esd check find error\n");
			//hw_reset(bl229x);
			recovey_from_esd_failed(bl229x);
			break;
		}
	}
	//---------------------------------------------	
	interrupt_mode_flag = 0;
	return 0;
}

// 传感器工作于手指监测模式
static int  bl229x_dev_interrupt_init(struct bl229x_data *bl229x)
{
	int i = 0;
	u8 reset [1] = {0x0c};
	u8 start[1]={0x18};
	u8 interrupt[1] = {0x14};

	u8 rx_data1[1]={0x00};
	u8 rx_data2[1]={0x00};

	u8 test_tx[7];
	u8 test_rx[7];

	spi_send_cmd(bl229x,reset,rx_data1,1);
	spi_send_cmd(bl229x,reset,rx_data1,1);
	spi_send_cmd(bl229x,start,rx_data2,1);

	test_tx[0] = 0x21;
	test_tx[1] = 0x66;
	test_tx[2] = 0x66;
	test_tx[3] = 0xB5;
	test_tx[4] = 0x00;
	test_tx[5] = last_agc = g_agc_value;//0xfA;
	test_tx[6] = 0x70; //0x50

	spi_send_cmd(bl229x,test_tx,test_rx,7);
	spi_send_cmd(bl229x,test_tx,test_rx,7);
	for(i = 3; i < 7;i++)
	{
		if(test_rx[i] != test_tx[i])
		{
			BTL_DEBUG("esd check find error\n");
			//hw_reset(bl229x);
			recovey_from_esd_failed(bl229x);
			break;
		}
	}
	spi_send_cmd(bl229x,interrupt,rx_data2,1);

	interrupt_mode_flag = 1;
	return 0;
}

//采集传感器图像
static int bl229x_read_image(struct bl229x_data *bl229x,uint32_t timeout)
{
	uint32_t i=0,j=0,w=0;
	uint8_t Frameend = 0;
	uint8_t linehead=0,temp=0;
	int8_t  image_cap_flag = 0;
	
	printk("Linc bl229x_read_image\n");
	if ((NULL == imagetxcmd)||(NULL == imagerxpix))
	{
		return -ENOMEM;
	}
	// 清空接收缓冲区 
	memset(imagerxpix,0x00,READIMAGE_BUF_SIZE);
	imagetxcmd[0] = 0x90;
	memset (&imagetxcmd[1], 0x66, READIMAGE_BUF_SIZE - 1);
	memset(imagebuf, 0x00, READIMAGE_BUF_SIZE);

	mtspi_set_dma_en(1); 

	while ((image_cap_flag == 0)&&(timeout != 0))
	{
		i = 0; 
		//memset(imagerxpix,0x00,12288);//tyy
		//获取图像
		spi_send_cmd(bl229x,imagetxcmd,imagerxpix,READIMAGE_BUF_SIZE);
		//for(i=0;i<12288;i++) BTL_DEBUG("%s imagerxpix[%d] = %d\n",__func__,i,imagerxpix[i]);
		//i=0;
		//提取图像
		while((Frameend == 0))
		{
			linehead=imagerxpix[i];
			//BTL_DEBUG("imagerxpix[%d] = %d\n",i,imagerxpix[i]);
			if (linehead==0xAA)
			{
				//BTL_DEBUG("%s find head\n",__func__);
				i++;
				temp = imagerxpix[i];
				for (j=0;j<112;j++)
				{
					i++;
					imagebuf[w] = imagerxpix[i];
					//BTL_DEBUG("%s imagebuf[%d] = %d\n",__func__,w,imagebuf[w]);
					w++;
				}
				if (temp==95)
				{
					Frameend = 1;
					image_cap_flag = 1;
					//BTL_DEBUG("%s Frameend\n",__func__);

					mtspi_set_dma_en(0); 
					return 0;
				}
			}
			i++;
			if (i >= 12177) // 
			{
				timeout --;
				if (timeout <= 1)
				{
					timeout =0 ;
					image_cap_flag = 1;
					BTL_DEBUG("%s read timeout\n",__func__);

					mtspi_set_dma_en(0); 
					return -1;               //没读到超时
				}
			}
		}
	}
	mtspi_set_dma_en(0); 

	interrupt_mode_flag = 1;
	return 0;

}
// powerdown mode
static void bl229x_powerdown_mode(struct bl229x_data* bl229x)
{
	static uint8_t powerdown_cmd[1] = {0x00};
	uint8_t powerdown_rx [1] = {0x00};
	spi_send_cmd(bl229x,powerdown_cmd,powerdown_rx,1); //发 0x18
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                    //
//                                             手指检测部分代码                                                       //
//                                                                                                                    // 
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static int bl229x_clear_interrupt(struct bl229x_data *bl229x)
{
	return bl229x_dev_interrupt_init(bl229x);
}

//中断处理函数

irqreturn_t bl229x_eint_handler(int irq,void *data)
{
	 if (interrupt_mode_flag == 0x01)
	  {
			//int_count = 0;
			interrupt_mode_flag = 0; //tyy

			input_report_key(bl229x_inputdev,key_interrupt,1);
			input_sync(bl229x_inputdev);

			input_report_key(bl229x_inputdev,key_interrupt,0);
			input_sync(bl229x_inputdev);
			
			bl229x_async_Report();

			printk("%s report power key %d\n",__func__,key_interrupt);
	   }

		msleep(report_delay);
		bl229x_dev_interrupt_init(g_bl229x);
	   return IRQ_HANDLED;
}

/* -------------------------------------------------------------------- */
#ifdef CONFIG_ARCH_MSM
static int bl229x_parse_dt(struct device *dev,
			struct bl229x_data *pdata)
{

	dev_err(dev, "bl229x_parse_dt\n");
	pdata->reset_gpio = of_get_named_gpio_flags(dev->of_node,
				"fingerprint,rst-gpio", 0, NULL);
	if (!gpio_is_valid(pdata->reset_gpio))
		return -EINVAL;
	gpio_direction_output(pdata->reset_gpio, 1);
	
	pdata->irq_gpio = of_get_named_gpio_flags(dev->of_node,
				"fingerprint,touch-int-gpio", 0, NULL);
	if (!gpio_is_valid(pdata->irq_gpio))
		return -EINVAL;
	gpio_direction_input(pdata->irq_gpio);
	
	pdata->power_en_gpio = of_get_named_gpio_flags(dev->of_node,
				"fingerprint,en-gpio", 0, NULL);
	if (!gpio_is_valid(pdata->power_en_gpio))
		return -EINVAL;
	gpio_direction_output(pdata->power_en_gpio, 1);

	dev_err(dev, "bl229x_parse_dt out\n");

	return 0;
}
#else

static int bl229x_parse_dt(struct device *dev,
			struct bl229x_data *pdata)
{
	int ret;
	struct pinctrl *pinctrl1 = pdata->pinctrl1;
	struct pinctrl_state *spi_pins_default = pdata->spi_pins_default;
	struct pinctrl_state *power_en_output0 = pdata->power_en_output0;
	struct pinctrl_state *power_en_output1 = pdata->power_en_output1;
	struct pinctrl_state *rst_output0 = pdata->rst_output0;
	struct pinctrl_state *rst_output1 = pdata->rst_output1;
	struct pinctrl_state *int_default = pdata->int_default;

	BTL_DEBUG("bl229x_pinctrl+++++++++++++++++\n");
	pinctrl1 = devm_pinctrl_get(dev);
	if (IS_ERR(pinctrl1)) {
		ret = PTR_ERR(pinctrl1);
		dev_err(dev, "fwq Cannot find bl229x pinctrl1!\n");
		return ret;
	}
	spi_pins_default = pinctrl_lookup_state(pinctrl1, "spi0_default");
	if (IS_ERR(pdata->spi_pins_default)) {
		ret = PTR_ERR(pdata->spi_pins_default);
		dev_err(dev, "fwq Cannot find bl229x pinctrl default %d!\n", ret);
	}
	rst_output1 = pinctrl_lookup_state(pinctrl1, "rst_output1");
	if (IS_ERR(rst_output1)) {
		ret = PTR_ERR(rst_output1);
		dev_err(dev, "fwq Cannot find bl229x pinctrl rst_output1!\n");
	}
	rst_output0 = pinctrl_lookup_state(pinctrl1, "rst_output0");
	if (IS_ERR(rst_output0)) {
		ret = PTR_ERR(rst_output0);
		dev_err(dev, "fwq Cannot find bl229x pinctrl rst_output0!\n");
	}
	power_en_output1 = pinctrl_lookup_state(pinctrl1, "power_en_output1");
	if (IS_ERR(power_en_output1)) {
		ret = PTR_ERR(power_en_output1);
		dev_err(dev, "fwq Cannot find bl229x pinctrl power_en_output1!\n");
	}
	power_en_output0 = pinctrl_lookup_state(pinctrl1, "power_en_output0");
	if (IS_ERR(power_en_output0)) {
		ret = PTR_ERR(power_en_output0);
		dev_err(dev, "fwq Cannot find bl229x pinctrl power_en_output0!\n");
	}
	int_default = pinctrl_lookup_state(pinctrl1, "int_default");
	if (IS_ERR(int_default)) {
		ret = PTR_ERR(int_default);
		dev_err(dev, "fwq Cannot find bl229x pinctrl int_default!\n");
	}
	
	pdata->reset_gpio = of_get_named_gpio_flags(dev->of_node,
				"fingerprint,rst-gpio", 0, NULL);
	pdata->irq_gpio = of_get_named_gpio_flags(dev->of_node,
				"fingerprint,touch-int-gpio", 0, NULL);
	pdata->power_en_gpio = of_get_named_gpio_flags(dev->of_node,
				"fingerprint,en-gpio", 0, NULL);
	
	pdata->pinctrl1 = pinctrl1;
	pdata->spi_pins_default = spi_pins_default;
	pdata->power_en_output0 = power_en_output0;
	pdata->power_en_output1 = power_en_output1;
	pdata->rst_output0 = rst_output0;
	pdata->rst_output1 = rst_output1;
	pdata->int_default = int_default;	

	pinctrl_select_state(pinctrl1, spi_pins_default);
	pinctrl_select_state(pinctrl1, rst_output1);
	pinctrl_select_state(pinctrl1, power_en_output1);
	pinctrl_select_state(pinctrl1, int_default);

	BTL_DEBUG("bl229x_pinctrl----------\n");
	return 0;
}

#endif

//电源开关AVDD（2.6V-3.6V），DVDD（1.8V），IOVDD（1.8V or 2.8V）,RST/SHUTDOWN pull high
//ESD recovery have to power off, AVDD must under control
static int bl229x_power_on(struct bl229x_data *bl229x,bool enable)
{
	if(enable)
	{
		if (!IS_ERR(bl229x->rst_output1)) {
			pinctrl_select_state(bl229x->pinctrl1, bl229x->rst_output1);
		}
		if (!IS_ERR(bl229x->power_en_output1)) {
			pinctrl_select_state(bl229x->pinctrl1, bl229x->power_en_output1);
		}
	}
	else
	{
		if (!IS_ERR(bl229x->rst_output0)) {
			pinctrl_select_state(bl229x->pinctrl1, bl229x->rst_output0);
		}
		if (!IS_ERR(bl229x->power_en_output0)) {
			pinctrl_select_state(bl229x->pinctrl1, bl229x->power_en_output0);
		}
	}
	return 0;
}

static int bl229x_gpio_select_and_init(struct bl229x_data *bl229x)
{
	int error = 0;
#if defined(ARCH_MTK_BTL)
	bl229x_parse_dt(&bl229x->spi->dev, bl229x);
#elif defined(CONFIG_ARCH_MSM)
	bl229x_parse_dt(&bl229x->spi->dev, bl229x);
	if (gpio_is_valid(bl229x->reset_gpio)) {
		error = gpio_request(bl229x->reset_gpio, "FINGERPRINT_RST");
		gpio_direction_output(bl229x->reset_gpio, 1);
	}
	if (gpio_is_valid(bl229x->power_en_gpio)) {
		error = gpio_request(bl229x->power_en_gpio, "FINGERPRINT_3V3_EN");
		dev_err(&bl229x->spi->dev, "power_en_gpio\n");
	}
	if (gpio_is_valid(bl229x->irq_gpio)) {
		error = gpio_request(bl229x->irq_gpio, "FINGERPRINT-IRQ");
		if (error) {
			dev_err(&bl229x->spi->dev, "unable to request GPIO %d\n",
				bl229x->irq_gpio);
			goto err;
		}
		error = gpio_direction_input(bl229x->irq_gpio);
		if (error) {
			dev_err(&bl229x->spi->dev, "set_direction for irq gpio failed\n");
			goto err;
		}
	}
#endif
	return error;
}



//线程处理函数
static int bl229x_thread_func(void *unused)
{
	//struct sched_param param = {.sched_priority = RTPM_PRIO_TPD};
	//sched_setscheduler(current,SCHED_RR,&param);
	do
	{
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, bl229x_interrupt_flag !=0);

		set_current_state(TASK_RUNNING); 

		if(gpio_get_value(g_bl229x->irq_gpio) != 0)
		{
			if(atomic_read(&suspended) == 1)
			{
#if 0			
				input_report_key(bl229x_inputdev,key_interrupt,1);
				input_sync(bl229x_inputdev);
	
				input_report_key(bl229x_inputdev,key_interrupt,0);
				input_sync(bl229x_inputdev);
#else 
                bl229x_async_Report();
#endif 
			}
			bl229x_interrupt_flag = 0;
			interrupt_mode_flag = 0;
		}
		else
		{
			msleep(100);
			if(atomic_read(&suspended) != 1)
			{
				bl229x_interrupt_flag = 0;
				interrupt_mode_flag = 0;
			}		
		}
		//}
	}while(!kthread_should_stop());

	return 0;
}

// 注册中断设备
static int bl229x_create_inputdev(void)
{
	bl229x_inputdev = input_allocate_device();
	if (!bl229x_inputdev)
	{
		BTL_DEBUG("bl229x_inputdev create faile!\n");
		return -ENOMEM;
	}
	__set_bit(EV_KEY,bl229x_inputdev->evbit);
	__set_bit(KEY_F10,bl229x_inputdev->keybit);		//68
	__set_bit(KEY_F12,bl229x_inputdev->keybit);		//88	
	__set_bit(KEY_CAMERA,bl229x_inputdev->keybit);	//212
	__set_bit(KEY_POWER,bl229x_inputdev->keybit);	//116
	__set_bit(KEY_PHONE,bl229x_inputdev->keybit);  //call 169

	__set_bit(KEY_F1,bl229x_inputdev->keybit);	//69
	__set_bit(KEY_F2,bl229x_inputdev->keybit);	//60
	__set_bit(KEY_F3,bl229x_inputdev->keybit);	//61
	__set_bit(KEY_F4,bl229x_inputdev->keybit);	//62
	__set_bit(KEY_F5,bl229x_inputdev->keybit);	//63
	__set_bit(KEY_F6,bl229x_inputdev->keybit);	//64
	__set_bit(KEY_F7,bl229x_inputdev->keybit);	//65
	__set_bit(KEY_F8,bl229x_inputdev->keybit);	//66
	__set_bit(KEY_F9,bl229x_inputdev->keybit);	//67

	bl229x_inputdev->id.bustype = BUS_HOST;
	bl229x_inputdev->name = "bl229x_inputdev";
	if (input_register_device(bl229x_inputdev))
	{
		BTL_DEBUG("register inputdev failed\n");
		input_free_device(bl229x_inputdev);
		return -ENOMEM;
	}
	//创建并运行线程

	bl229x_thread = kthread_run(bl229x_thread_func,g_bl229x,"bl229x_thread");
	if (IS_ERR(bl229x_thread))
	{
		BTL_DEBUG("kthread_run is faile\n");
		return -(PTR_ERR(bl229x_thread));
	}

	return 0;
}

static int  mt_spi_init(void)
{
	int ret=0;
	BTL_DEBUG("%s",__func__);
	//ret=spi_register_board_info(spi_board_bl229x,ARRAY_SIZE(spi_board_bl229x));
	ret=spi_register_driver(&bl229x_driver);

	return ret; 
}


static void  mt_spi_exit(void)
{
	spi_unregister_driver(&bl229x_driver);
}

module_init(mt_spi_init);
module_exit(mt_spi_exit);

MODULE_DESCRIPTION("BL229X fingerprint driver for mtk & qcom");
MODULE_AUTHOR("Liang Bo");
MODULE_LICENSE("GPL");
MODULE_ALIAS("bl229x");
