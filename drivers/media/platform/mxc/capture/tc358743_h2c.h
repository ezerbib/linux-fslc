/*
 * tc358743_h2c.h
 *
 *  Created on: Jul 24, 2016
 *      Author: ezerbib
 */

#ifndef DRIVERS_MEDIA_PLATFORM_MXC_CAPTURE_TC358743_H2C_H_
#define DRIVERS_MEDIA_PLATFORM_MXC_CAPTURE_TC358743_H2C_H_

enum tc358743_mode {
	tc358743_mode_INIT0, //1080p60
	tc358743_mode_INIT, //cb640x480-108MHz
	tc358743_mode_INIT1,//cb1280x720-2lane@30
	tc358743_mode_INIT2,//cb1280x720-4lane-125MHz
	tc358743_mode_INIT3,//cb1024x720-4lane
	tc358743_mode_INIT4,//cb640x480-174Mhz
	tc358743_mode_INIT5,//cb1280x720-4lane-300MHz
	tc358743_mode_INIT6,//ccb1920x1023
	tc358743_mode_480P_720_480,
	tc358743_mode_720P_60_1280_720,
	tc358743_mode_480P_640_480,
	tc358743_mode_1080P_1920_1080,
//	tc358743_mode_1440x480,
	tc358743_mode_720P_1280_720,
	tc358743_mode_1024x768,
	tc358743_mode_1440x480,
	tc358743_mode_720x576,
	tc358743_mode_2880x480,
	tc358743_mode_800x600,
	tc358743_mode_1280x1024,
	tc358743_mode_848x480,
	tc358743_mode_1600x900,
	tc358743_mode_1440x900,
	tc358743_mode_1600x1200,
	tc358743_mode_1920x1200,
//	tc358743_mode_1080P_1920_1080,
	tc358743_mode_1366x768,
	tc358743_mode_MAX,
};
/*!
 * Maintains the information on the current state of the sensor.
 */
struct tc_data {
	struct sensor_data sensor;
	struct delayed_work det_work;
	struct mutex access_lock;
	int det_work_enable;
	int det_work_timeout;
	int det_changed;
#define REGULATOR_IO		0
#define REGULATOR_CORE		1
#define REGULATOR_GPO		2
#define REGULATOR_ANALOG	3
#define REGULATOR_CNT		4
	struct regulator *regulator[REGULATOR_CNT];
	u32 lock;
	u32 bounce;
	enum tc358743_mode mode;
	u32 fps;
	u32 audio;
	int pwn_gpio;
	int rst_gpio;
	u16 hpd_active;
	int edid_initialized;
	int irq_changed;
#ifdef CONFIG_TC358743_DEV
	struct tc358743_irq_private *tc_irq_priv;
#endif
};


extern struct tc_data *tc358743_get_tc_data(void);
extern s32 tc358743_read_reg(struct sensor_data *sensor, u16 reg, void *rxbuf);

#endif /* DRIVERS_MEDIA_PLATFORM_MXC_CAPTURE_TC358743_H2C_H_ */
