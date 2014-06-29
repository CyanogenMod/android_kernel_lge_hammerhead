/*
 * Author: Paul Reioux aka Faux123 <reioux@gmail.com>
 *
 * WCD93xx sound control module
 * Copyright 2013 Paul Reioux
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/kallsyms.h>
#include <linux/mfd/wcd9xxx/wcd9320_registers.h>

#define SOUND_CONTROL_MAJOR_VERSION	3
#define SOUND_CONTROL_MINOR_VERSION	1

extern struct snd_soc_codec *fauxsound_codec_ptr;

static int snd_ctrl_locked = 0;

unsigned int taiko_read(struct snd_soc_codec *codec, unsigned int reg);
int taiko_write(struct snd_soc_codec *codec, unsigned int reg,
		unsigned int value);

int reg_access(unsigned int reg)
{
	int ret = 1;

	switch (reg) {
		case TAIKO_A_RX_HPH_L_GAIN:
		case TAIKO_A_RX_HPH_R_GAIN:
		case TAIKO_A_RX_HPH_L_STATUS:
		case TAIKO_A_RX_HPH_R_STATUS:
		case TAIKO_A_CDC_RX1_VOL_CTL_B2_CTL:
		case TAIKO_A_CDC_RX2_VOL_CTL_B2_CTL:
		case TAIKO_A_CDC_RX3_VOL_CTL_B2_CTL:
		case TAIKO_A_CDC_RX4_VOL_CTL_B2_CTL:
		case TAIKO_A_CDC_RX5_VOL_CTL_B2_CTL:
		case TAIKO_A_CDC_RX6_VOL_CTL_B2_CTL:
		case TAIKO_A_CDC_RX7_VOL_CTL_B2_CTL:
		case TAIKO_A_CDC_TX1_VOL_CTL_GAIN:
		case TAIKO_A_CDC_TX2_VOL_CTL_GAIN:
		case TAIKO_A_CDC_TX3_VOL_CTL_GAIN:
		case TAIKO_A_CDC_TX4_VOL_CTL_GAIN:
		case TAIKO_A_CDC_TX5_VOL_CTL_GAIN:
		case TAIKO_A_CDC_TX6_VOL_CTL_GAIN:
		case TAIKO_A_CDC_TX7_VOL_CTL_GAIN:
		case TAIKO_A_CDC_TX8_VOL_CTL_GAIN:
		case TAIKO_A_CDC_TX9_VOL_CTL_GAIN:
		case TAIKO_A_CDC_TX10_VOL_CTL_GAIN:
			if (snd_ctrl_locked)
				ret = 0;
			break;
		default:
			break;
	}
	return ret;
}
EXPORT_SYMBOL(reg_access);

static bool calc_checksum(unsigned int a, unsigned int b, unsigned int c)
{
	unsigned char chksum = 0;

	chksum = ~((a & 0xff) + (b & 0xff));

	if (chksum == (c & 0xff)) {
		return true;
	} else {
		return false;
	}
}

static ssize_t cam_mic_gain_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
        return sprintf(buf, "%u\n",
		taiko_read(fauxsound_codec_ptr,
			TAIKO_A_CDC_TX6_VOL_CTL_GAIN));

}

static ssize_t cam_mic_gain_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int lval, chksum;

	sscanf(buf, "%u %u", &lval, &chksum);

	if (calc_checksum(lval, 0, chksum)) {
		taiko_write(fauxsound_codec_ptr,
			TAIKO_A_CDC_TX6_VOL_CTL_GAIN, lval);
	}
	return count;
}

static ssize_t mic_gain_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n",
		taiko_read(fauxsound_codec_ptr,
			TAIKO_A_CDC_TX7_VOL_CTL_GAIN));
}

static ssize_t mic_gain_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int lval, chksum;

	sscanf(buf, "%u %u", &lval, &chksum);

	if (calc_checksum(lval, 0, chksum)) {
		taiko_write(fauxsound_codec_ptr,
			TAIKO_A_CDC_TX7_VOL_CTL_GAIN, lval);
	}
	return count;

}

static ssize_t speaker_gain_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
        return sprintf(buf, "%u %u\n",
			taiko_read(fauxsound_codec_ptr,
				TAIKO_A_CDC_RX7_VOL_CTL_B2_CTL),
			taiko_read(fauxsound_codec_ptr,
				TAIKO_A_CDC_RX7_VOL_CTL_B2_CTL));

}

static ssize_t speaker_gain_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int lval, rval, chksum;

	sscanf(buf, "%u %u %u", &lval, &rval, &chksum);

	if (calc_checksum(lval, rval, chksum)) {
		taiko_write(fauxsound_codec_ptr,
			TAIKO_A_CDC_RX7_VOL_CTL_B2_CTL, lval);
		taiko_write(fauxsound_codec_ptr,
			TAIKO_A_CDC_RX7_VOL_CTL_B2_CTL, rval);
	}
	return count;
}

static ssize_t headphone_gain_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u %u\n",
			taiko_read(fauxsound_codec_ptr,
				TAIKO_A_CDC_RX1_VOL_CTL_B2_CTL),
			taiko_read(fauxsound_codec_ptr,
				TAIKO_A_CDC_RX2_VOL_CTL_B2_CTL));
}

static ssize_t headphone_gain_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int lval, rval, chksum;

	sscanf(buf, "%u %u %u", &lval, &rval, &chksum);

	if (calc_checksum(lval, rval, chksum)) {
		taiko_write(fauxsound_codec_ptr,
			TAIKO_A_CDC_RX1_VOL_CTL_B2_CTL, lval);
		taiko_write(fauxsound_codec_ptr,
			TAIKO_A_CDC_RX2_VOL_CTL_B2_CTL, rval);
	}
	return count;
}

static ssize_t headphone_pa_gain_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u %u\n",
		taiko_read(fauxsound_codec_ptr, TAIKO_A_RX_HPH_L_GAIN),
		taiko_read(fauxsound_codec_ptr, TAIKO_A_RX_HPH_R_GAIN));
}

static ssize_t headphone_pa_gain_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int lval, rval, chksum;
	unsigned int gain, status;
	unsigned int out;

	sscanf(buf, "%u %u %u", &lval, &rval, &chksum);

	if (calc_checksum(lval, rval, chksum)) {
	gain = taiko_read(fauxsound_codec_ptr, TAIKO_A_RX_HPH_L_GAIN);
	out = (gain & 0xf0) | lval;
	taiko_write(fauxsound_codec_ptr, TAIKO_A_RX_HPH_L_GAIN, out);

	status = taiko_read(fauxsound_codec_ptr, TAIKO_A_RX_HPH_L_STATUS);
	out = (status & 0x0f) | (lval << 4);
	taiko_write(fauxsound_codec_ptr, TAIKO_A_RX_HPH_L_STATUS, out);

	gain = taiko_read(fauxsound_codec_ptr, TAIKO_A_RX_HPH_R_GAIN);
	out = (gain & 0xf0) | rval;
	taiko_write(fauxsound_codec_ptr, TAIKO_A_RX_HPH_R_GAIN, out);

	status = taiko_read(fauxsound_codec_ptr, TAIKO_A_RX_HPH_R_STATUS);
	out = (status & 0x0f) | (rval << 4);
	taiko_write(fauxsound_codec_ptr, TAIKO_A_RX_HPH_R_STATUS, out);
	}
	return count;
}

static ssize_t sound_control_version_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "version: %u.%u\n",
			SOUND_CONTROL_MAJOR_VERSION,
			SOUND_CONTROL_MINOR_VERSION);
}

static ssize_t sound_control_locked_store(struct kobject *kobj,
                struct kobj_attribute *attr, const char *buf, size_t count)
{
	int inp;

	sscanf(buf, "%d", &inp);

	if (inp == 0)
		snd_ctrl_locked = 0;
	else
		snd_ctrl_locked = 1;

	return count;
}

static ssize_t sound_control_locked_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
        return sprintf(buf, "%d\n", snd_ctrl_locked);
}

static struct kobj_attribute cam_mic_gain_attribute =
	__ATTR(gpl_cam_mic_gain,
		0666,
		cam_mic_gain_show,
		cam_mic_gain_store);

static struct kobj_attribute mic_gain_attribute =
	__ATTR(gpl_mic_gain,
		0666,
		mic_gain_show,
		mic_gain_store);

static struct kobj_attribute speaker_gain_attribute =
	__ATTR(gpl_speaker_gain,
		0666,
		speaker_gain_show,
		speaker_gain_store);

static struct kobj_attribute headphone_gain_attribute =
	__ATTR(gpl_headphone_gain,
		0666,
		headphone_gain_show,
		headphone_gain_store);

static struct kobj_attribute headphone_pa_gain_attribute =
	__ATTR(gpl_headphone_pa_gain,
		0666,
		headphone_pa_gain_show,
		headphone_pa_gain_store);

static struct kobj_attribute sound_control_locked_attribute =
	__ATTR(gpl_sound_control_locked,
		0666,
		sound_control_locked_show,
		sound_control_locked_store);

static struct kobj_attribute sound_control_version_attribute =
	__ATTR(gpl_sound_control_version,
		0444,
		sound_control_version_show, NULL);

static struct attribute *sound_control_attrs[] =
	{
		&cam_mic_gain_attribute.attr,
		&mic_gain_attribute.attr,
		&speaker_gain_attribute.attr,
		&headphone_gain_attribute.attr,
		&headphone_pa_gain_attribute.attr,
		&sound_control_locked_attribute.attr,
		&sound_control_version_attribute.attr,
		NULL,
	};

static struct attribute_group sound_control_attr_group =
	{
		.attrs = sound_control_attrs,
	};

static struct kobject *sound_control_kobj;

static int sound_control_init(void)
{
	int sysfs_result;

	sound_control_kobj =
		kobject_create_and_add("sound_control_3", kernel_kobj);

	if (!sound_control_kobj) {
		pr_err("%s sound_control_kobj create failed!\n",
			__FUNCTION__);
		return -ENOMEM;
        }

	sysfs_result = sysfs_create_group(sound_control_kobj,
			&sound_control_attr_group);

	if (sysfs_result) {
		pr_info("%s sysfs create failed!\n", __FUNCTION__);
		kobject_put(sound_control_kobj);
	}
	return sysfs_result;
}

static void sound_control_exit(void)
{
	if (sound_control_kobj != NULL)
		kobject_put(sound_control_kobj);
}

module_init(sound_control_init);
module_exit(sound_control_exit);
MODULE_LICENSE("GPL and additional rights");
MODULE_AUTHOR("Paul Reioux <reioux@gmail.com>");
MODULE_DESCRIPTION("Sound Control Module 3.x");

