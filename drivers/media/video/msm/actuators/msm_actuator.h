/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef MSM_ACTUATOR_H
#define MSM_ACTUATOR_H

#include <linux/i2c.h>
#include <mach/camera.h>
#include <mach/gpio.h>
#include <media/v4l2-subdev.h>
#include <media/msm_camera.h>
#include "msm_camera_i2c.h"
/* HTC_START harvey 20120730 */
#ifdef CONFIG_MACH_MAGNIDS
#include <mach/TCA6418_ioextender.h>
#endif
/* HTC_END */

#ifdef LERROR
#undef LERROR
#endif

#ifdef LINFO
#undef LINFO
#endif

#define LERROR(fmt, args...) pr_err(fmt, ##args)

#define CONFIG_MSM_CAMERA_ACT_DBG 0

#if CONFIG_MSM_CAMERA_ACT_DBG
#define LINFO(fmt, args...) printk(fmt, ##args)
#else
#define LINFO(fmt, args...) CDBG(fmt, ##args)
#endif

/* HTC_START */
#ifdef CONFIG_RAWCHIP
#define USE_RAWCHIP_AF
#endif
/* HTC_END */

struct msm_actuator_ctrl_t;

/* HTC_START */
struct damping_t {
	struct damping_params_t *ringing_params;
};
/* HTC_END */

struct msm_actuator_func_tbl {
	int32_t (*actuator_i2c_write_b_af)(struct msm_actuator_ctrl_t *,
			uint8_t,
			uint8_t);
	int32_t (*actuator_init_step_table)(struct msm_actuator_ctrl_t *,
		struct msm_actuator_set_info_t *);
	int32_t (*actuator_init_focus)(struct msm_actuator_ctrl_t *,
		uint16_t, enum msm_actuator_data_type, struct reg_settings_t *);
	int32_t (*actuator_set_default_focus) (struct msm_actuator_ctrl_t *,
			struct msm_actuator_move_params_t *);
	/* HTC_START */
	int32_t (*actuator_init_table)(struct msm_actuator_ctrl_t *);
	int32_t (*actuator_set_params)(struct msm_actuator_ctrl_t *);
	/* HTC_END */
	int32_t (*actuator_move_focus) (struct msm_actuator_ctrl_t *,
			struct msm_actuator_move_params_t *);
	int32_t (*actuator_i2c_write)(struct msm_actuator_ctrl_t *,
			int16_t, uint32_t);
	int32_t (*actuator_write_focus)(struct msm_actuator_ctrl_t *,
			uint16_t,
			struct damping_params_t *,
			int8_t,
			int16_t);
};

struct msm_actuator {
	enum actuator_type act_type;
	struct msm_actuator_func_tbl func_tbl;
};

/* HTC_START */
struct msm_actuator_ctrl {
	int (*a_init_table)(void);
	int (*a_power_up)(void *);
	int (*a_power_down)(void *);
	int (*a_create_subdevice)(void *, void *);
	int (*a_config)(void __user *);
};
/* HTC_END */

struct msm_actuator_ctrl_t {
	struct i2c_driver *i2c_driver;
	struct msm_camera_i2c_client i2c_client;
	struct mutex *actuator_mutex;
	struct msm_actuator_func_tbl *func_tbl;
	enum msm_actuator_data_type i2c_data_type;
	struct v4l2_subdev sdev;
	struct v4l2_subdev_ops *act_v4l2_subdev_ops;
	/* HTC_START */
	uint32_t i2c_addr;
	struct msm_actuator_ctrl actuator_ext_ctrl;
	struct msm_actuator_set_info_t set_info;
	struct msm_actuator_get_info_t get_info;
	uint16_t *ringing_scenario[2];
	uint16_t scenario_size[2];
	struct damping_t *damping[2];
	/*HTC_END */

	int16_t curr_step_pos;
	uint16_t curr_region_index;
	uint16_t *step_position_table;
	struct region_params_t region_params[MAX_ACTUATOR_REGION];
	uint16_t reg_tbl_size;
	struct msm_actuator_reg_params_t reg_tbl[MAX_ACTUATOR_REG_TBL_SIZE];
	uint16_t region_size;
	void *user_data;
	uint32_t vcm_pwd;
	uint32_t vcm_enable;
	uint32_t total_steps;
	uint16_t pwd_step;
	uint16_t initial_code;
	int use_rawchip_af; /* HTC Angie 20120615 */
};

struct msm_actuator_ctrl_t *get_actrl(struct v4l2_subdev *sd);
int32_t msm_actuator_i2c_write(struct msm_actuator_ctrl_t *a_ctrl,
		int16_t next_lens_position, uint32_t hw_params);
int32_t msm_actuator_init_focus(struct msm_actuator_ctrl_t *a_ctrl,
		uint16_t size, enum msm_actuator_data_type type,
		struct reg_settings_t *settings);
int32_t msm_actuator_i2c_write_b_af(struct msm_actuator_ctrl_t *a_ctrl,
		uint8_t msb,
		uint8_t lsb);
int32_t msm_actuator_move_focus(struct msm_actuator_ctrl_t *a_ctrl,
		struct msm_actuator_move_params_t *move_params);
int32_t msm_actuator_piezo_move_focus(
		struct msm_actuator_ctrl_t *a_ctrl,
		struct msm_actuator_move_params_t *move_params);
int32_t msm_actuator_init_step_table(struct msm_actuator_ctrl_t *a_ctrl,
		struct msm_actuator_set_info_t *set_info);
int32_t msm_actuator_set_default_focus(struct msm_actuator_ctrl_t *a_ctrl,
		struct msm_actuator_move_params_t *move_params);
/* HTC_START */
int32_t msm_actuator_config(struct msm_actuator_ctrl_t *a_ctrl,
		void __user *cfg_data);
int32_t msm_actuator_af_power_down(struct msm_actuator_ctrl_t *a_ctrl);
int32_t msm_actuator_create_subdevice(struct msm_actuator_ctrl_t *a_ctrl,
		struct i2c_board_info const *board_info,
		struct v4l2_subdev *sdev);
/* HTC_END */
int32_t msm_actuator_piezo_set_default_focus(
		struct msm_actuator_ctrl_t *a_ctrl,
		struct msm_actuator_move_params_t *move_params);
int32_t msm_actuator_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id);
int32_t msm_actuator_write_focus(struct msm_actuator_ctrl_t *a_ctrl,
		uint16_t curr_lens_pos, struct damping_params_t *damping_params,
		int8_t sign_direction, int16_t code_boundary);
int32_t msm_actuator_write_focus2(struct msm_actuator_ctrl_t *a_ctrl,
		uint16_t curr_lens_pos, struct damping_params_t *damping_params,
		int8_t sign_direction, int16_t code_boundary);
long msm_actuator_subdev_ioctl(struct v4l2_subdev *sd,
			unsigned int cmd, void *arg);
int32_t msm_actuator_power(struct v4l2_subdev *sd, int on);

#define VIDIOC_MSM_ACTUATOR_CFG \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 11, void __user *)

#endif
