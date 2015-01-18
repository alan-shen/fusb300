#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/of_gpio.h>


#define DRIVER_NAME "usb_typec_i2c"

#define REG_DEVICE_ID 	0X01	//RO
#define REG_SWITCHES_0 	0X02	//RW
#define REG_SWITCHES_1 	0X03	//RW
#define REG_MEASURE 	0X04	//RW
#define REG_SLICE 		0X05	//RW
#define REG_CONTROL_0 	0X06	//RW
#define REG_CONTROL_1 	0X07	//RW
#define REG_MASK_0 		0X0A	//RW
#define REG_POWER 		0X0B	//RW
#define REG_SRESET 		0X0C	//WC
#define REG_STATUS_0 	0X40	//RO
#define REG_STATUS_1 	0X41	//RO
#define REG_INTERRUPT 	0X42	//RC
#define REG_FIFOS 		0X43	//RW

/*register status_0*/
#define STS0_COMP		(1<<5)
#define STS0_BC_LVL		0x3

/*register interrupt*/
#define I_COMP		(1<<5)

enum dev_state {
	DEV_UNKNOW = 0,
	DEV_FLOAT,
	DEV_ATTACH,
};

enum cc_state {
	CC_UNKNOW = 0,
	CC_RA,
	CC_RD,
	CC_FLOAT,
};

enum fusb_work_mode {
	MD_UNKNOW = 0,
	MD_UFP,
	MD_DFP,
	MD_DR,
};

extern int typec_set_msm_usb_host_mode(bool on);

struct fusb300_dev_info {
	u8 reg;
	char *name;
	int irq_gpio;
	int irq;
	int int_status;
	enum cc_state cc1_status;
	enum cc_state cc2_status;
	enum dev_state dev_status;
	enum fusb_work_mode fusb_mode;
	struct i2c_client *i2c_client;
	struct regulator *regulator_vdd;
	struct pinctrl *pinctrl;
	struct pinctrl_state *intr_active;
	struct delayed_work check_work;
	struct mutex di_mutex;
};

static int usb_typec_regulator_init(struct fusb300_dev_info *fdi)
{
	return 0;
}
static int usb_typec_power_on(struct fusb300_dev_info *fdi)
{

	return 0;
}
static int usb_typec_gpio_config(struct fusb300_dev_info *fdi)
{
	int ret = 0;

	if(gpio_is_valid(fdi->irq_gpio)) {
		ret = gpio_request(fdi->irq_gpio, "fusb300_irq_gpio");
	}
	if (ret) {
		dev_err(&fdi->i2c_client->dev,
			"unable to request gpio [%d]\n",
			fdi->irq_gpio);
		goto err_irq_gpio_req;
	}
	ret = gpio_direction_input(fdi->irq_gpio);
	/*ret = gpio_direction_output(fdi->irq_gpio, 1);//wyh debug*/
	if (ret) {
		dev_err(&fdi->i2c_client->dev,
			"unable to set direction for gpio " \
			"[%d]\n", fdi->irq_gpio);
		goto err_irq_gpio_dir;
	}

	printk("wyh debug %s:irq gpio config done.\n", __func__);

	return ret;

err_irq_gpio_dir:
	if (gpio_is_valid(fdi->irq_gpio))
		gpio_free(fdi->irq_gpio);
err_irq_gpio_req:
	return ret;
}
static int usb_typec_parse_dt(struct device *dev, struct fusb300_dev_info *fdi)
{
	if(NULL != dev->of_node) {
		fdi->irq_gpio = of_get_named_gpio_flags(dev->of_node,
			"irq-gpio", 0, NULL);
	}

	printk("wyh debug %s: irq_gpio=%d.\n", __func__, fdi->irq_gpio);

	return 0;
}

static int fusb300_write_i2c(struct i2c_client *client, u8 reg, u8 value)
{
	struct i2c_msg msg[1];
	u8 data[2];
	int ret;

	if (!client->adapter)
		return -ENODEV;

	data[0] = reg;
	data[1] = value;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = data;
	msg[0].len = sizeof(data);

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		return ret;

	return 0;
}

static int fusb300_read_i2c(struct i2c_client *client, u8 reg)
{
	struct i2c_msg msg[2];
	u8 data;
	int ret;

	if ((NULL == client) || (NULL == client->adapter))
		return -ENODEV;

	/*printk(KERN_ERR "wyh debug addr=0x%x,reg=0x%x.\n", client->addr, reg);*/

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = &data;
	msg[1].len = sizeof(data);

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		return ret;

	return (int)data;
}

static int fusb300_write_reg(struct i2c_client *client, u8 reg, u8 value)
{
	int ret;

	printk("wyh debug value=0x%hhx.\n", value);
	ret = fusb300_write_i2c(client, reg, value);
	if(ret < 0) {
		dev_err(&client->dev, "%s write err:%d.\n", __func__, ret);
	}
#if 1
//check
	ret = fusb300_read_i2c(client, reg);
	if(ret < 0) {
		dev_err(&client->dev, "%s read err:%d.\n", __func__, ret);
	}
	printk("wyh debug reg0x%x=0x%x.\n", reg, ret);
#endif

	return ret;
}

static int get_dev_id(struct i2c_client *client)
{
	int ret;
	u8 reg = 0x01;

	ret = fusb300_read_i2c(client, reg);
	if(ret < 0) {
		dev_err(&client->dev, "%s read err:%d.\n", __func__, ret);
	}

	dev_info(&client->dev, "%s reg0x%x=0x%x.\n", __func__, reg, ret);

#if 0
	for(reg=0x1;reg<0x8;reg++){
		ret = fusb300_read_i2c(client, reg);
		if(ret < 0) {
			dev_err(&client->dev, "%s read err:%d.\n", __func__, ret);
		}

		dev_info(&client->dev, "%s reg0x%x=0x%x.\n", __func__, reg, ret);
	}

	reg = 0x40;
	ret = fusb300_read_i2c(client, reg);
	if(ret < 0) {
		dev_err(&client->dev, "%s read err:%d.\n", __func__, ret);
	}

	dev_info(&client->dev, "%s reg0x%x=0x%x.\n", __func__, reg, ret);

	reg = 0x41;
	ret = fusb300_read_i2c(client, reg);
	if(ret < 0) {
		dev_err(&client->dev, "%s read err:%d.\n", __func__, ret);
	}

	dev_info(&client->dev, "%s reg0x%x=0x%x.\n", __func__, reg, ret);

	reg = 0x42;
	ret = fusb300_read_i2c(client, reg);
	if(ret < 0) {
		dev_err(&client->dev, "%s read err:%d.\n", __func__, ret);
	}

	dev_info(&client->dev, "%s reg0x%x=0x%x.\n", __func__, reg, ret);
#endif

	return ret;
}

static void fusb300_set_mode(struct fusb300_dev_info *fdi, enum fusb_work_mode fusb_mode)
{
	if(MD_DFP == fusb_mode) {
		fusb300_write_reg(fdi->i2c_client, REG_SWITCHES_0, 0xcc);
		fusb300_write_reg(fdi->i2c_client, REG_MEASURE, 0x26);
		fusb300_write_reg(fdi->i2c_client, REG_CONTROL_0, 0x04);
		fusb300_write_reg(fdi->i2c_client, REG_POWER, 0x07);
	}

	return;
}

static void fusb300_pullup_cc1(struct fusb300_dev_info *fdi)
{
	fusb300_write_reg(fdi->i2c_client, REG_SWITCHES_0, 0x44);
	fusb300_write_reg(fdi->i2c_client, REG_MEASURE, 0x26);
	fusb300_write_reg(fdi->i2c_client, REG_CONTROL_0, 0x04);
	fusb300_write_reg(fdi->i2c_client, REG_POWER, 0x07);

	return;
}

static void fusb300_pullup_cc2(struct fusb300_dev_info *fdi)
{
	fusb300_write_reg(fdi->i2c_client, REG_SWITCHES_0, 0x88);
	fusb300_write_reg(fdi->i2c_client, REG_MEASURE, 0x26);
	fusb300_write_reg(fdi->i2c_client, REG_CONTROL_0, 0x04);
	fusb300_write_reg(fdi->i2c_client, REG_POWER, 0x07);

	return;
}

static void fusb300_enter_low_power_mode(struct fusb300_dev_info *fdi)
{
	fusb300_write_reg(fdi->i2c_client, REG_SWITCHES_0, 0x0c);
	fusb300_write_reg(fdi->i2c_client, REG_POWER, 0x01);
	fusb300_write_reg(fdi->i2c_client, REG_CONTROL_0, 0x00);

	return;
}

static void fusb300_check_status_work(struct work_struct *work)
{
	int reg_sts0;
	struct fusb300_dev_info *fdi =
			container_of(work, struct fusb300_dev_info,
			check_work.work);

	/*check cc1 status*/
	fusb300_pullup_cc1(fdi);
	reg_sts0 = fusb300_read_i2c(fdi->i2c_client,REG_STATUS_0);
	if (reg_sts0 & STS0_COMP)
		fdi->cc1_status = CC_UNKNOW;
	else if (reg_sts0 & STS0_BC_LVL)
		fdi->cc1_status = CC_RD;
	else if (!(reg_sts0 & STS0_BC_LVL))
		fdi->cc1_status = CC_RA;

	/*check cc2 status*/
	fusb300_pullup_cc2(fdi);
	reg_sts0 = fusb300_read_i2c(fdi->i2c_client,REG_STATUS_0);
	if (reg_sts0 & STS0_COMP)
		fdi->cc2_status = CC_UNKNOW;
	else if (reg_sts0 & STS0_BC_LVL)
		fdi->cc2_status = CC_RD;
	else if (!(reg_sts0 & STS0_BC_LVL))
		fdi->cc2_status = CC_RA;

	/*set plug out cable interrupt*/
	if(CC_RD == fdi->cc1_status) {
		fusb300_write_i2c(fdi->i2c_client, REG_SWITCHES_0, 0x64);
	}

	if(CC_RD == fdi->cc1_status) {
		fusb300_write_i2c(fdi->i2c_client, REG_SWITCHES_0, 0x98);
	}

	if ((fdi->cc1_status == CC_RD && fdi->cc2_status == CC_RA) ||
			(fdi->cc1_status == CC_RA && fdi->cc2_status == CC_RD)) {
		/*DFP mode device attach*/
		typec_set_msm_usb_host_mode(true);
		fdi->dev_status = DEV_ATTACH;
	} else if (fdi->cc1_status == CC_RD && fdi->cc2_status == CC_RD) {
		/*it should no need to set host mode,when vbus interrupt*/
		/*occure,usb driver will set to host automaticlly */
		typec_set_msm_usb_host_mode(false);
		fdi->dev_status = DEV_UNKNOW;
	} else {
		typec_set_msm_usb_host_mode(false);
		fdi->dev_status = DEV_FLOAT;
	}

	if (DEV_ATTACH == fdi->dev_status) {
		if ((fdi->int_status & I_COMP) && (reg_sts0 & STS0_COMP)) {
			fusb300_enter_low_power_mode(fdi);
		}
	}

	return;
}

static int fusb300_enable_irq(struct i2c_client *client, bool enable)
{
	int ret;
	u8 val;

	if(!enable) {
		printk("wyh debug disable irq.\n");
	} else {
		printk("wyh debug enable irq.\n");
		ret = fusb300_read_i2c(client, REG_CONTROL_0);
		if(ret < 0) {
			dev_err(&client->dev, "%s read err:%d.\n", __func__, ret);
		}
		ret &= ~(1 << 5);
		val = (u8)ret;
		printk("wyh debug val=0x%hhx.\n", val);
		ret = fusb300_write_i2c(client, REG_CONTROL_0, val);
		if(ret < 0) {
			dev_err(&client->dev, "%s write err:%d.\n", __func__, ret);
		}
#if 1
//check
		ret = fusb300_read_i2c(client, REG_CONTROL_0);
		if(ret < 0) {
			dev_err(&client->dev, "%s read err:%d.\n", __func__, ret);
		}
		printk("wyh debug reg06=0x%x.\n", ret);
#endif
	}

	return ret;
}

static int is_valid_reg(u8 reg)
{
	if(reg >= 0x1 && reg <= 0xc)
		return 0;
	else if(reg >= 0x40 && reg <= 0x43)
		return 0;
	else if(0xff == reg)//for dump regs
		return 0;

	return -1;
}

static int fusb300_dump_regs(struct i2c_client *client)
{
	int ret,i;

	for(i=1;i<=0x7;i++){
		ret = fusb300_read_i2c(client, (u8)i);
		printk(KERN_ERR "REG0x%x=0x%x.\n", i,ret);
	}

	ret = fusb300_read_i2c(client, 0x0A);
	printk(KERN_ERR "REG0x0a=0x%x.\n", ret);
	ret = fusb300_read_i2c(client, 0x0B);
	printk(KERN_ERR "REG0x0b=0x%x.\n", ret);

	for(i=0x40;i<=0x43;i++){
		ret = fusb300_read_i2c(client, (u8)i);
		printk(KERN_ERR "REG0x%x=0x%x.\n", i,ret);
	}

	return ret;
}

static ssize_t fusb300_sysfs_show_votg(struct device *dev,
				struct device_attribute *attr, char *buf)
{

	/*struct i2c_client *client = to_i2c_client(dev);*/
	/*struct fusb300_dev_info *fdi = i2c_get_clientdata(client);*/

	return sprintf(buf, "votg.\n");
}

static ssize_t fusb300_sysfs_set_votg(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret, val;

	ret = kstrtouint(buf, 10, &val);
	if(ret)
		return ret;

	typec_set_msm_usb_host_mode(val);

	return count;
}

static ssize_t fusb300_sysfs_show_mode(struct device *dev,
				struct device_attribute *attr, char *buf)
{

	/*struct i2c_client *client = to_i2c_client(dev);*/
	/*struct fusb300_dev_info *fdi = i2c_get_clientdata(client);*/

	return sprintf(buf, "in kernel log.\n");
}

static ssize_t fusb300_sysfs_set_mode(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{

	return count;
}

static ssize_t fusb300_sysfs_show_regs(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	/*int ret;*/

	struct i2c_client *client = to_i2c_client(dev);
	struct fusb300_dev_info *fdi = i2c_get_clientdata(client);
	/*struct fusb300_dev_info *fdi = container_of(*/
			/*&client, struct fusb300_dev_info, i2c_client);*/

	if(NULL == fdi || NULL == client)
		return 0;

	fusb300_dump_regs(client);
#if 0
	ret = is_valid_reg(fdi->reg);
	if(ret < 0){
		dev_err(&client->dev, "%s invalid reg:%d.\n", __func__, fdi->reg);
		return 0;
	}

	if(0xff == fdi->reg) {
		fusb300_dump_regs(client);
		return 0;
	} else
		ret = fusb300_read_i2c(client, fdi->reg);

	return sprintf(buf, "reg%d=%d.\n", fdi->reg, ret);
#endif

	return sprintf(buf, "reg value in kernel log.\n");
}

static ssize_t fusb300_sysfs_set_regs(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	u8 reg,val;
	int ret;
	struct i2c_client *client = to_i2c_client(dev);
	struct fusb300_dev_info *fdi = i2c_get_clientdata(client);

	if(NULL == fdi || NULL == client)
		return count;

	printk(KERN_ERR "wyh debug count=%ld.\n", count);

	if(sscanf(buf, "%hhx %hhx", &reg, &val) != 2) {
		dev_err(&client->dev,
				"%s: get regs values: Invaild regs\n",
				__func__);
		return -EINVAL;
	}

	printk(KERN_ERR "wyh debug %s reg=0x%hhx,val=0x%hhx\n", __func__,  reg, val);

	ret = is_valid_reg(reg);
	if(ret < 0){
		dev_err(&client->dev, "%s invalid reg:%d.\n", __func__, fdi->reg);
		return count;
	}

	ret = fusb300_write_i2c(client, reg, val);
	if(ret < 0) {
		dev_err(&client->dev, "%s write err:%d.\n", __func__, ret);
		return count;
	}

	return count;
}

static DEVICE_ATTR(regs, S_IRUGO | S_IWUGO,
		   fusb300_sysfs_show_regs, fusb300_sysfs_set_regs);
static DEVICE_ATTR(mode, S_IRUGO | S_IWUGO,
		   fusb300_sysfs_show_mode, fusb300_sysfs_set_mode);
static DEVICE_ATTR(votg, S_IRUGO | S_IWUGO,
		   fusb300_sysfs_show_votg, fusb300_sysfs_set_votg);

static struct attribute *attrs[] = {
	&dev_attr_regs.attr,
	&dev_attr_mode.attr,
	&dev_attr_votg.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

static irqreturn_t fusb300_irq_handler(int irq, void *info)
{
	int reg_sts0;
	struct fusb300_dev_info *fdi = info;

	/*printk(KERN_ERR "wyh debug %s.\n", __func__);*/

	if(NULL == fdi->i2c_client)
		return IRQ_NONE;

	/*must clear interrupt register*/
	fdi->int_status = fusb300_read_i2c(fdi->i2c_client,REG_INTERRUPT);
	reg_sts0 = fusb300_read_i2c(fdi->i2c_client,REG_STATUS_0);

	if(fdi->fusb_mode == MD_DFP && (reg_sts0 & STS0_BC_LVL)) {
		/*delay 200ms for debonce*/
		schedule_delayed_work(&fdi->check_work, 200);
	}

	return IRQ_HANDLED;
}

static int usb_typec_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
	int ret = 0;
	struct fusb300_dev_info *fdi = NULL;

	printk("wyh debug only %s.\n", __func__);

	if(!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev,
				"%s: SMBus byte data commands not supported by host\n",
				__func__);
		return -EIO;
	}


	fdi = kzalloc(sizeof(struct fusb300_dev_info), GFP_KERNEL);
	if (!fdi) {
		dev_err(&client->dev,
				"%s: Failed to alloc mem for fusb300 device info\n",
				__func__);
		return -ENOMEM;
	}

	fdi->i2c_client = client;

	INIT_DELAYED_WORK(&fdi->check_work, fusb300_check_status_work);

	usb_typec_parse_dt(&client->dev, fdi);

	fdi->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(fdi->pinctrl)) {
		pr_err("%s: Unable to get pinctrl handle\n", __func__);
		goto err_gpio_config;
	}

	fdi->intr_active = pinctrl_lookup_state(fdi->pinctrl,
					"fusb300_active");
	if (IS_ERR(fdi->intr_active)) {
		pr_err("%s: could not get intr_active pinstate\n", __func__);
		goto err_gpio_config;
	}

	ret = pinctrl_select_state(fdi->pinctrl,
					fdi->intr_active);
	if (ret != 0) {
		pr_err("%s: Disable TLMM pins failed with %d\n",
			__func__, ret);
		ret = -EIO;
		goto err_gpio_config;
	}

	ret = usb_typec_gpio_config(fdi);
	if(ret < 0) {
		dev_err(&client->dev, "Failed to configure gpios\n");
		goto err_gpio_config;
	}

	fdi->irq = gpio_to_irq(fdi->irq_gpio);
	printk("wyh debug %s irq_gpio is %d, irq=%d.\n", __func__, fdi->irq_gpio, fdi->irq);

	ret = request_threaded_irq(fdi->irq, NULL,
		/*fusb300_irq_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT,*/
		fusb300_irq_handler, IRQF_TRIGGER_LOW | IRQF_ONESHOT,
		DRIVER_NAME, fdi);
	if(ret < 0) {
		dev_err(&client->dev, "Failed to request irq.\n");
		goto err_request_irq;
	}

	/*need request L21A_3P3 LDO*/
	ret = usb_typec_regulator_init(fdi);
	if(ret < 0) {
		dev_err(&client->dev, "Failed to init regulator.\n");
		goto err_request_regulator;
	}
	ret = usb_typec_power_on(fdi);
	if(ret < 0) {
		dev_err(&client->dev, "Failed to power on.\n");
		goto err_power_on;
	}

	i2c_set_clientdata(client, fdi);

	ret = get_dev_id(client);
	if(ret != 0x50) {
		dev_err(&client->dev, "%s unexpect device dev_id=%d,abort!\n", __func__, ret);
		goto err_unknow_dev;
	}
		goto err_unknow_dev; //wyh debug

	fusb300_set_mode(fdi, MD_DFP);

	fusb300_enable_irq(client, true);

	ret = sysfs_create_group(&(&client->dev)->kobj, &attr_group);
	if(ret) {
		dev_err(&client->dev, "%s sysfs err:%d.\n", __func__, ret);
		goto err_creat_sysfs;
	}

	printk("wyh debug %s end.\n", __func__);
	return 0;

err_creat_sysfs:
err_unknow_dev:
err_power_on:
	regulator_put(fdi->regulator_vdd);
err_request_regulator:
	free_irq(fdi->irq, NULL);
err_request_irq:
	if (gpio_is_valid(fdi->irq_gpio))
		gpio_free(fdi->irq_gpio);
err_gpio_config:
	kfree(fdi);
	return 0;
}

static int usb_typec_i2c_remove(struct i2c_client *client)
{
	sysfs_remove_group(&(&client->dev)->kobj, &attr_group);

	return 0;
}


static const struct i2c_device_id usb_typec_id_table[] = {
	{DRIVER_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, usb_typec_id_table);

#ifdef CONFIG_OF
static struct of_device_id typec_match_table[] = {
	{ .compatible = "letv,usb_type_c",},
	{ },
};
#else
#define typec_match_table NULL
#endif

static struct i2c_driver usb_typec_i2c_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = typec_match_table,
	},
	.probe = usb_typec_i2c_probe,
	.remove = usb_typec_i2c_remove,
	.id_table = usb_typec_id_table,
};

int usb_typec_init(void)
{
	return i2c_add_driver(&usb_typec_i2c_driver);
}

void usb_typec_exit(void)
{
	i2c_del_driver(&usb_typec_i2c_driver);

	return;
}

module_init(usb_typec_init);
module_exit(usb_typec_exit);

MODULE_AUTHOR("Letv, Inc.");
MODULE_DESCRIPTION("Letv usb-type-c I2C Bus Support Module");
MODULE_LICENSE("GPL v2");
