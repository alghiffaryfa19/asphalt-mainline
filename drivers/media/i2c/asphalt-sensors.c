// SPDX-License-Identifier: GPL-2.0-only
/* Minimal RAW10 capture for the TB320FC's HI1339, S5K4H7 and SC202CS.
 * Registers/timing: stock 24 MHz sensor modules; see patch provenance.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

struct asphalt_reg {
	u32 reg;
	u32 value;
	u32 delay_us;
};
struct asphalt_table {
	const struct asphalt_reg *regs;
	unsigned int count;
};
#include "asphalt-sensor-regs.h"
#define TABLE(a) { a, ARRAY_SIZE(a) }
enum asphalt_model { HI1339, S5K4H7, SC202CS };
struct asphalt_info {
	const char *name;
	enum asphalt_model model;
	u32 id_reg, id, width, height, code, hts, vts, pixel_rate;
	s64 link_freq;
	u32 lanes, margin, min_exposure, gain_min, gain_max;
	struct asphalt_table init, mode, start, stop, hold, release;
};
#define TABLES(n)                                                     \
	.init = TABLE(n##_init), .mode = TABLE(n##_mode),             \
	.start = TABLE(n##_stream_on), .stop = TABLE(n##_stream_off), \
	.hold = TABLE(n##_group_hold_on), .release = TABLE(n##_group_hold_off)
static const struct asphalt_info hi1339_info = {
	.name = "hi1339",
	.model = HI1339,
	.id_reg = CCI_REG16(0x0716),
	.id = 0x1339,
	.width = 4208,
	.height = 3120,
	.code = MEDIA_BUS_FMT_SGBRG10_1X10,
	/* 72 MHz / 720 clocks per line, expressed in pixel units. */
	.hts = 5760,
	.vts = 3332,
	.pixel_rate = 576000000,
	.link_freq = 608000000,
	.lanes = 4,
	.margin = 4,
	.min_exposure = 4,
	.gain_min = 0,
	.gain_max = 240,
	TABLES(hi1339),
};
static const struct asphalt_info s5k4h7_info = {
	.name = "s5k4h7",
	.model = S5K4H7,
	.id_reg = CCI_REG16(0x0000),
	.id = 0x487b,
	.width = 3264,
	.height = 2448,
	.code = MEDIA_BUS_FMT_SGRBG10_1X10,
	.hts = 3688,
	.vts = 2530,
	.pixel_rate = 280000000,
	.link_freq = 340000000,
	.lanes = 4,
	.margin = 5,
	.min_exposure = 2,
	.gain_min = 32,
	.gain_max = 512,
	TABLES(s5k4h7),
};
static const struct asphalt_info sc202cs_info = {
	.name = "sc202cs",
	.model = SC202CS,
	.id_reg = CCI_REG16(0x3107),
	.id = 0xeb52,
	.width = 1600,
	.height = 1200,
	.code = MEDIA_BUS_FMT_SBGGR10_1X10,
	.hts = 1920,
	.vts = 1250,
	.pixel_rate = 72000000,
	.link_freq = 360000000,
	.lanes = 1,
	.margin = 6,
	.min_exposure = 2,
	.gain_min = 1024,
	.gain_max = 16384,
	TABLES(sc202cs),
};
struct asphalt_sensor {
	struct device *dev;
	struct regmap *regmap;
	const struct asphalt_info *info;
	struct clk *clock;
	struct gpio_desc *reset;
	struct regulator_bulk_data supplies[4];
	unsigned int num_supplies;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler controls;
	struct v4l2_ctrl *exposure;
	struct mutex lock;
	bool streaming;
};
static struct asphalt_sensor *to_sensor(struct v4l2_subdev *sd)
{
	return container_of(sd, struct asphalt_sensor, sd);
}
static int write_table(struct asphalt_sensor *s, const struct asphalt_table *t)
{
	unsigned int i;
	int ret;
	for (i = 0; i < t->count; i++) {
		ret = cci_write(s->regmap, t->regs[i].reg, t->regs[i].value,
				NULL);
		if (ret)
			return ret;
		if (t->regs[i].delay_us)
			fsleep(t->regs[i].delay_us);
	}
	return 0;
}
static int sensor_power_on(struct device *dev)
{
	struct asphalt_sensor *s = to_sensor(dev_get_drvdata(dev));
	bool front = s->info->model == S5K4H7;
	int ret, i;

	if (front) {
		ret = clk_prepare_enable(s->clock);
		if (ret)
			return ret;
		msleep(10);
	}
	gpiod_set_value_cansleep(s->reset, 1);
	fsleep(1000);
	for (i = 0; i < s->num_supplies; i++) {
		ret = regulator_enable(s->supplies[i].consumer);
		if (ret)
			goto disable_supplies;
		fsleep(1000);
	}
	if (!front) {
		ret = clk_prepare_enable(s->clock);
		if (ret)
			goto disable_supplies;
		fsleep(s->info->model == SC202CS ? 5000 : 1000);
	}
	gpiod_set_value_cansleep(s->reset, 0);
	fsleep(s->info->model == HI1339 ? 10000 : 5000);
	return 0;

disable_supplies:
	while (i--)
		regulator_disable(s->supplies[i].consumer);
	if (front)
		clk_disable_unprepare(s->clock);
	return ret;
}
static int sensor_power_off(struct device *dev)
{
	struct asphalt_sensor *s = to_sensor(dev_get_drvdata(dev));
	int i;
	gpiod_set_value_cansleep(s->reset, 1);
	fsleep(1000);
	clk_disable_unprepare(s->clock);
	for (i = s->num_supplies - 1; i >= 0; i--) {
		regulator_disable(s->supplies[i].consumer);
		fsleep(1000);
	}
	return 0;
}
static int sensor_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct asphalt_sensor *s =
		container_of(ctrl->handler, struct asphalt_sensor, controls);
	const struct asphalt_info *p = s->info;
	u32 v = ctrl->val;
	int ret = 0, released;

	if (ctrl->id == V4L2_CID_VBLANK)
		__v4l2_ctrl_modify_range(
			s->exposure, p->min_exposure, p->height + v - p->margin,
			1, min_t(u32, 1000, p->height + v - p->margin));
	if (!pm_runtime_get_if_in_use(s->dev))
		return 0;
	ret = write_table(s, &p->hold);
	if (ret)
		goto out;
	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		if (p->model == HI1339)
			cci_write(s->regmap, CCI_REG16(0x020a), v, &ret);
		else if (p->model == S5K4H7)
			cci_write(s->regmap, CCI_REG16(0x0202), v, &ret);
		else {
			cci_write(s->regmap, CCI_REG8(0x3e00), (v >> 12) & 0xf,
				  &ret);
			cci_write(s->regmap, CCI_REG8(0x3e01), (v >> 4) & 0xff,
				  &ret);
			cci_write(s->regmap, CCI_REG8(0x3e02), (v & 0xf) << 4,
				  &ret);
		}
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		if (p->model == HI1339)
			cci_write(s->regmap, CCI_REG16(0x0212), v, &ret);
		else if (p->model == S5K4H7)
			cci_write(s->regmap, CCI_REG16(0x0204), v, &ret);
		else {
			unsigned int stage =
				min_t(unsigned int, ilog2(v / 1024), 4);
			cci_write(s->regmap, CCI_REG8(0x3e09), BIT(stage) - 1,
				  &ret);
			cci_write(s->regmap, CCI_REG8(0x3e07),
				  v * 128 / (1024 << stage), &ret);
		}
		break;
	case V4L2_CID_VBLANK:
		v += p->height;
		cci_write(s->regmap,
			  p->model == HI1339 ? CCI_REG16(0x020e) :
			  p->model == S5K4H7 ? CCI_REG16(0x0340) :
					       CCI_REG16(0x320e),
			  v, &ret);
		break;
	default:
		ret = -EINVAL;
	}
	released = write_table(s, &p->release);
	if (!ret)
		ret = released;
out:
	pm_runtime_put(s->dev);
	return ret;
}
static const struct v4l2_ctrl_ops sensor_ctrl_ops = { .s_ctrl =
							      sensor_set_ctrl };
static int sensor_stream(struct v4l2_subdev *sd, int enable)
{
	struct asphalt_sensor *s = to_sensor(sd);
	int ret = 0;
	mutex_lock(&s->lock);
	if (s->streaming == !!enable)
		goto unlock;
	if (!enable) {
		ret = write_table(s, &s->info->stop);
		s->streaming = false;
		pm_runtime_put(s->dev);
		goto unlock;
	}
	ret = pm_runtime_resume_and_get(s->dev);
	if (ret < 0)
		goto unlock;
	ret = write_table(s, &s->info->init);
	if (!ret)
		ret = write_table(s, &s->info->mode);
	if (!ret)
		ret = __v4l2_ctrl_handler_setup(&s->controls);
	if (!ret)
		ret = write_table(s, &s->info->start);
	if (ret) {
		write_table(s, &s->info->stop);
		pm_runtime_put(s->dev);
	} else {
		s->streaming = true;
	}
unlock:
	mutex_unlock(&s->lock);
	return ret;
}
static int sensor_enum_code(struct v4l2_subdev *sd,
			    struct v4l2_subdev_state *state,
			    struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index || code->pad)
		return -EINVAL;
	code->code = to_sensor(sd)->info->code;
	return 0;
}
static int sensor_enum_size(struct v4l2_subdev *sd,
			    struct v4l2_subdev_state *state,
			    struct v4l2_subdev_frame_size_enum *size)
{
	const struct asphalt_info *p = to_sensor(sd)->info;
	if (size->index || size->pad || size->code != p->code)
		return -EINVAL;
	size->min_width = size->max_width = p->width;
	size->min_height = size->max_height = p->height;
	return 0;
}
static int sensor_format(struct v4l2_subdev *sd,
			 struct v4l2_subdev_state *state,
			 struct v4l2_subdev_format *fmt)
{
	const struct asphalt_info *p = to_sensor(sd)->info;
	if (fmt->pad)
		return -EINVAL;
	fmt->format = (struct v4l2_mbus_framefmt){
		.width = p->width,
		.height = p->height,
		.code = p->code,
		.field = V4L2_FIELD_NONE,
		.colorspace = V4L2_COLORSPACE_RAW,
		.quantization = V4L2_QUANTIZATION_FULL_RANGE,
		.xfer_func = V4L2_XFER_FUNC_NONE,
	};
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;
	return 0;
}
static int sensor_selection(struct v4l2_subdev *sd,
			    struct v4l2_subdev_state *state,
			    struct v4l2_subdev_selection *sel)
{
	const struct asphalt_info *p = to_sensor(sd)->info;
	if (sel->pad)
		return -EINVAL;
	switch (sel->target) {
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r = (struct v4l2_rect){ .width = p->width,
					     .height = p->height };
		return 0;
	default:
		return -EINVAL;
	}
}
static int sensor_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format fmt = { .which = V4L2_SUBDEV_FORMAT_TRY };
	return sensor_format(sd, state, &fmt);
}
static const struct v4l2_subdev_video_ops sensor_video_ops = {
	.s_stream = sensor_stream
};
static const struct v4l2_subdev_pad_ops sensor_pad_ops = {
	.enum_mbus_code = sensor_enum_code,
	.enum_frame_size = sensor_enum_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = sensor_format,
	.get_selection = sensor_selection,
};
static const struct v4l2_subdev_ops sensor_ops = { .video = &sensor_video_ops,
						   .pad = &sensor_pad_ops };
static const struct v4l2_subdev_internal_ops sensor_internal_ops = {
	.init_state = sensor_init_state
};
static int sensor_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct v4l2_fwnode_endpoint ep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
	struct v4l2_fwnode_device_properties props;
	struct fwnode_handle *endpoint;
	struct v4l2_ctrl *ctrl;
	struct asphalt_sensor *s;
	const struct asphalt_info *p;
	u64 id = 0;
	unsigned int i;
	int ret;

	s = devm_kzalloc(dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;
	s->dev = dev;
	s->info = p = device_get_match_data(dev);
	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint)
		return -EINVAL;
	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep);
	fwnode_handle_put(endpoint);
	if (ret)
		return ret;
	ret = -EINVAL;
	if (ep.bus.mipi_csi2.num_data_lanes == p->lanes)
		for (i = 0; i < ep.nr_of_link_frequencies; i++)
			if (ep.link_frequencies[i] == p->link_freq)
				ret = 0;
	v4l2_fwnode_endpoint_free(&ep);
	if (ret)
		return dev_err_probe(dev, ret,
				     "unsupported CSI lanes/frequency\n");
	s->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(s->regmap))
		return PTR_ERR(s->regmap);
	s->clock = devm_clk_get(dev, NULL);
	if (IS_ERR(s->clock))
		return dev_err_probe(dev, PTR_ERR(s->clock), "clock\n");
	ret = clk_set_rate(s->clock, 24000000);
	if (ret || clk_get_rate(s->clock) != 24000000)
		return dev_err_probe(dev, ret ?: -EINVAL,
				     "24 MHz clock required\n");
	s->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(s->reset))
		return dev_err_probe(dev, PTR_ERR(s->reset), "reset GPIO\n");
	s->supplies[0].supply = "vddio";
	s->supplies[1].supply = "vdd";
	s->supplies[2].supply = "avdd";
	s->supplies[3].supply = "dvdd";
	s->num_supplies = p->model == SC202CS ? 3 : 4;
	ret = devm_regulator_bulk_get(dev, s->num_supplies, s->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "supplies\n");
	mutex_init(&s->lock);
	v4l2_i2c_subdev_init(&s->sd, client, &sensor_ops);
	v4l2_i2c_subdev_set_name(&s->sd, client, p->name, NULL);
	s->sd.internal_ops = &sensor_internal_ops;
	s->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	s->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	s->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = sensor_power_on(dev);
	if (ret)
		goto destroy_mutex;
	ret = cci_read(s->regmap, p->id_reg, &id, NULL);
	if (ret || id != p->id) {
		ret = dev_err_probe(dev, ret ?: -ENODEV,
				    "chip ID %#llx (expected %#x)\n", id,
				    p->id);
		goto power_off;
	}
	v4l2_ctrl_handler_init(&s->controls, 8);
	s->controls.lock = &s->lock;
	ctrl = v4l2_ctrl_new_int_menu(&s->controls, NULL, V4L2_CID_LINK_FREQ, 0,
				      0, &p->link_freq);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	v4l2_ctrl_new_std(&s->controls, NULL, V4L2_CID_PIXEL_RATE,
			  p->pixel_rate, p->pixel_rate, 1, p->pixel_rate);
	ctrl = v4l2_ctrl_new_std(&s->controls, NULL, V4L2_CID_HBLANK,
				 p->hts - p->width, p->hts - p->width, 1,
				 p->hts - p->width);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	s->exposure = v4l2_ctrl_new_std(&s->controls, &sensor_ctrl_ops,
					V4L2_CID_EXPOSURE, p->min_exposure,
					p->vts - p->margin, 1, 1000);
	v4l2_ctrl_new_std(&s->controls, &sensor_ctrl_ops, V4L2_CID_VBLANK,
			  p->vts - p->height,
			  (p->model == SC202CS ? 3750 : 65535) - p->height, 1,
			  p->vts - p->height);
	v4l2_ctrl_new_std(&s->controls, &sensor_ctrl_ops,
			  V4L2_CID_ANALOGUE_GAIN, p->gain_min, p->gain_max, 1,
			  p->gain_min);
	ret = v4l2_fwnode_device_parse(dev, &props);
	if (ret)
		goto free_controls;
	ret = v4l2_ctrl_new_fwnode_properties(&s->controls, &sensor_ctrl_ops,
					      &props);
	if (ret)
		goto free_controls;
	if (s->controls.error) {
		ret = s->controls.error;
		goto free_controls;
	}
	s->sd.ctrl_handler = &s->controls;
	ret = media_entity_pads_init(&s->sd.entity, 1, &s->pad);
	if (ret)
		goto free_controls;
	s->sd.state_lock = &s->lock;
	ret = v4l2_subdev_init_finalize(&s->sd);
	if (ret)
		goto clean_entity;
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	ret = v4l2_async_register_subdev_sensor(&s->sd);
	if (ret)
		goto disable_pm;
	pm_runtime_idle(dev);
	dev_info(dev, "%s chip %#llx, %ux%u RAW10\n", p->name, id, p->width,
		 p->height);
	return 0;
disable_pm:
	pm_runtime_disable(dev);
	v4l2_subdev_cleanup(&s->sd);
clean_entity:
	media_entity_cleanup(&s->sd.entity);
free_controls:
	v4l2_ctrl_handler_free(&s->controls);
power_off:
	sensor_power_off(dev);
destroy_mutex:
	mutex_destroy(&s->lock);
	return ret;
}
static void sensor_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct asphalt_sensor *s = to_sensor(sd);
	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&s->controls);
	pm_runtime_disable(s->dev);
	if (!pm_runtime_status_suspended(s->dev))
		sensor_power_off(s->dev);
	pm_runtime_set_suspended(s->dev);
	mutex_destroy(&s->lock);
}
static const struct dev_pm_ops sensor_pm_ops = { SET_RUNTIME_PM_OPS(
	sensor_power_off, sensor_power_on, NULL) };
static const struct of_device_id sensor_of_match[] = {
	{ .compatible = "hynix,hi1339", .data = &hi1339_info },
	{ .compatible = "samsung,s5k4h7", .data = &s5k4h7_info },
	{ .compatible = "smartsens,sc202cs", .data = &sc202cs_info },
	{}
};
MODULE_DEVICE_TABLE(of, sensor_of_match);
static struct i2c_driver sensor_driver = {
	.driver = { .name = "asphalt-sensors",
		    .of_match_table = sensor_of_match,
		    .pm = &sensor_pm_ops },
	.probe = sensor_probe,
	.remove = sensor_remove,
};
module_i2c_driver(sensor_driver);
MODULE_DESCRIPTION("Lenovo TB320FC RAW10 camera sensors");
MODULE_LICENSE("GPL");
