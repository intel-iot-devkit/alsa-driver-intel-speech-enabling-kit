#ifndef __S1000_DSP_MSG_H
#define __S1000_DSP_MSG_H

/* ROM control CMD */
#define S1000_ROM_CONTROL_LOAD            0x02
#define S1000_ROM_CONTROL_ROM_READY       0x20
#define S1000_ROM_CONTROL_MEM_WRITE       0x11

#define S1000_BOOT_MSECS		5000
#define S1000_SHORT_MSG_SIZE	64
#define S1000_LARGE_MSG_SIZE	4096

#define S1000_CLOCK_SELECT_DEFAULT 0x0
#define S1000_CLOCK_SELECT_SPI_SLAVE 0x1

#define S1000_PARAM_GENERIC 0xFE
#define S1000_PARAM_SPECFIC 0xFF

#define S1000_GTW_CMD_GET_DATA 0x1
#define S1000_GTW_CMD_SET_DATA 0x2

struct s1000_msg {
	u32 primary;
	u32 extension;
};

struct s1000_rom_mem_info {
	u32 addr;
	u32 value;
};

union s1000_fw_load_flags {
	u32 val;
	struct {
		u32 count:9;
		u32 rsvd_1:12;
		u32 clock_select:4;
		u32 no_sha:1;
		u32 no_exec:1;
		u32 no_tlb:1;
		u32 no_sram:1;
		u32 no_l1_cache:1;
		u32 rsvd_2:2;
	} flags;
};

struct s1000_fw_load_data {
	u32 mem_addr;
	u32 img_offset;
	u32 img_size;
	u32 img_sha[8];
};

union s1000_rom_info {
	u32 val;
	struct {
		u32 fuse_values:8;
		u32 load_method:1;
		u32 rsvd1:3;
		u32 revision_min:4;
		u32 revision_maj:4;
		u32 version_min:4;
		u32 versiom_maj:4;
		u32 rsvd2:4;
	} rom;
};

struct s1000_resource_event {
	u32 type;
	u32 id;
	u32 event_type;
	u32 event_data[6];
};

struct s1000_large_param_cfg {
	u32 param_id;
	u32 length;
	u32 value[0];
};

struct s1000_gtw_payload {
	u32 node_id;
	u8 data[0];
};

struct s1000_gtw_payload_resp {
	u32 len;
	u8 data[0];
};

struct s1000_kp_payload {
	u16 phrase_start;
	u16 phrase_end;
};

struct s1000_mega_cfg_payload {
	u32 offset;
	u8 data[0];
};

irqreturn_t s1000_dsp_trigger_rising_handler(int irq, void *dev_id);
irqreturn_t s1000_dsp_trigger_falling_handler(int irq, void *dev_id);

int s1000_rom_cntrl_msg(struct s1000 *ctx, u32 val,
			u32 *param, u32 cmd, bool wait);

int s1000_set_large_cfg_msg(struct s1000 *ctx, u32 mod_id, u32 param_id,
				u32 param_size, u32 *param);

int s1000_get_large_cfg_msg(struct s1000 *ctx, u32 mod_id, u32 param_id,
				u32 param_size, u32 *param);

int s1000_set_mod_cfg_msg(struct s1000 *ctx, u32 mod_id, u32 param_id,
				u32 *param);

int s1000_set_megalarge_cfg_msg(struct s1000 *ctx, u32 mod_id, u32 cmd_id,
				u32 size, u32 *blob);

int s1000_send_null_msg(struct s1000 *ctx);

int s1000_msg_init(struct s1000 *ctx);

void s1000_msg_cleanup(struct s1000 *ctx);

int push_request(struct s1000 *ctx, u32 *data, u32 data_size,
			u32 *rx_data, u32 rx_size, int wait, bool raw_msg);
int s1000_set_ipcgateway_msg(struct s1000 *ctx, u32 param_size, u32 *param);

int s1000_get_ipcgateway_msg(struct s1000 *ctx, u32 param_size,
				u32 *param, u32 *data, u32 *sz_left);
#endif /* __S1000_DSP_MSG_H */
