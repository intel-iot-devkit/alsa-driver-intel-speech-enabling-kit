#ifndef __SOUND_SOC_S1000_H
#define __SOUND_SOC_S1000_H

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <sound/pcm.h>
#include <sound/soc.h>

#define S1000_HOST_RST_N 22
#define S1000_HOST_IRQ 27
#define S1000_GPIO_VAL (gpio_get_value(S1000_HOST_IRQ))

#define S1000_SHORT_MSG_SIZE	64
#define S1000_LARGE_MSG_SIZE	4096
#define S1000_MAX_STREAMS	2

enum s1000_state {
	S1000_STATE_IDLE = 0,
	S1000_STATE_ACTIVE,
};

enum s1000_dsp_state {
	S1000_DSP_RUNNING,
	S1000_DSP_UPDATING,
};

enum s1000_stream_cmd {
	S1000_CMD_STOP = 0x001,
	S1000_CMD_START = 0x002,
	S1000_CMD_CLOUD_STOP = 0x003,
	S1000_CMD_CLOUD_START = 0x004,
	S1000_CMD_SOFT_RESET = 0x005,
	S1000_CMD_TPLG_STOP = 0x008,
	S1000_CMD_TPLG_START = 0x009,
	S1000_CMD_FW_UPDATE_START = 0xc,
};

enum s1000_streaming_mode {
	S1000_MODE_WOV = 0,
	S1000_MODE_STREAMING,
};

struct s1000_stream {
	struct snd_soc_dai *dai;
	u32 id;
	u32 res_id;
	bool low_thres;
	bool high_thres;
	bool wov_stream;
	bool listening_mode;

	int channels;
	int params_rate;
	int pcm_format_width;

	unsigned int pos; /*in bytes */
	unsigned int period_pos; /*in bytes */
	unsigned int size;
	unsigned int buffer_size;
	unsigned int period_size;
	struct snd_pcm_substream *substream;
	bool started;
	bool is_prepared;

	wait_queue_head_t wr_waitq;
	struct work_struct data_kwork;
	bool kp_detected;
	u32 kp_data;
	struct mutex lock;
	int streaming_mode;
};

struct s1000_gen_msg {
	struct list_head node;
	u32 *tx_data;
	u32 *rx_data;
	u32 tx_size;
	u32 rx_size;
	bool wait;
	wait_queue_head_t waitq;
	bool recv;
	bool tx_done;
	int err_code;
	bool raw_msg;
	bool mode_switch;
};

struct s1000 {
	struct spi_device *spi;
	struct device *dev;
	int irq;
	const struct firmware *fw;
	const char *fw_name;
	int rx_size;
	int tx_size;
	enum s1000_state state;
	bool xfer_raw_mode;
	bool large_msg;
	bool expect_notif;
	wait_queue_head_t boot_wait;
	wait_queue_head_t rom_ready_wait;
	wait_queue_head_t dfu_ready_wait;
	bool boot_complete;
	bool rom_ready;
	bool dfu_ready;
	bool msg_send;
	struct mutex mutex;
	struct mutex thread_lock;
	struct s1000_stream streams[2];
	u32 mgm_id;
	int count_rising_irq;
	int count_rising_miss;
	int count_falling_irq;
	int count_falling_miss;
	int count_thread;
	int count_req;
	unsigned int flags;
	u16 sv_score;
	bool boot_mode;

	struct platform_device *i2s_dev;
	struct snd_soc_platform *platform;
	//TODO: Add into sep struct
	struct s1000_gen_msg *msg;
	struct list_head tx_list;
	struct list_head rx_list;
	struct list_head msg_pool_list;

	struct work_struct kwork;
	bool tplg_loaded;
	enum s1000_dsp_state dsp_state;
	struct mutex lock;
};

int s1000_dsp_init(struct s1000 *ctx);
void s1000_dsp_cleanup(struct s1000 *ctx);
int s1000_spi_transfer(struct spi_device *spi, void *tx_buf,
		  void *rx_buf, int size, bool swap);
void s1000_set_state(struct s1000 *ctx, int state);
int s1000_platform_register(struct device *dev);
int s1000_platform_unregister(struct device *dev);
int s1000_stream_control_op(struct s1000 *ctx, int cmd, struct s1000_stream *stream);
int s1000_write_data(struct s1000 *ctx, u32 res_id, void *data, size_t size);
int s1000_read_data(struct s1000 *ctx, u32 res_id, void *data,
				    size_t size, u32 *sz_left);
void s1000_stream_low_threshold(struct s1000 *ctx, u32 res_id);
void s1000_stream_high_threshold(struct s1000 *ctx, u32 res_id);
void s1000_stream_kp_detected(struct s1000 *ctx, void *data);
#endif /* __SOUND_SOC_S1000_H */
