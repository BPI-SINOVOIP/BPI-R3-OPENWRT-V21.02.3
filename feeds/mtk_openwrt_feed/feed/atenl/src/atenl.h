/* Copyright (C) 2021-2022 Mediatek Inc. */
#ifndef __ATENL_H
#define __ATENL_H

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <linux/nl80211.h>
#include <net/if.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "nl.h"
#include "util.h"
#include "debug.h"

#define BRIDGE_NAME	"br-lan"
#define ETH_P_RACFG	0x2880
#define RACFG_PKT_MAX_SIZE	1600
#define RACFG_HLEN	12
#define RACFG_MAGIC_NO	0x18142880

#define RACFG_CMD_TYPE_MASK	GENMASK(14, 0)
#define RACFG_CMD_TYPE_ETHREQ	BIT(3)
#define RACFG_CMD_TYPE_PLATFORM_MODULE	GENMASK(4, 3)

#define set_band_val(_an, _band, _field, _val)	\
	_an->anb[_band]._field = (_val)
#define get_band_val(_an, _band, _field)	\
	(_an->anb[_band]._field)

enum atenl_rf_mode {
	ATENL_RF_MODE_NORMAL,
	ATENL_RF_MODE_TEST,
	ATENL_RF_MODE_ICAP,
	ATENL_RF_MODE_ICAP_OVERLAP,

	__ATENL_RF_MODE_MAX,
};

struct atenl_rx_stat {
	u64 total;
	u64 ok_cnt;
	u64 err_cnt;
	u64 len_mismatch;
};

struct atenl_band {
	bool valid;
	u8 phy_idx;
	u8 cap;
	u8 chainmask;

	enum mt76_testmode_state cur_state;
	s8 tx_power;
	enum atenl_rf_mode rf_mode;

	bool use_tx_time;
	u32 tx_time;
	u32 tx_mpdu_len;

	bool reset_tx_cnt;
	bool reset_rx_cnt;

	/* history */
	struct atenl_rx_stat rx_stat;
};

#define MAX_BAND_NUM	3

struct atenl {
	struct atenl_band anb[MAX_BAND_NUM];
	u16 chip_id;
	u16 adie_id;
	u8 sub_chip_id;
	u8 cur_band;

	u8 mac_addr[ETH_ALEN];
	bool unicast;
	int sock_eth;

	const char *mtd_part;
	u32 mtd_offset;
	u8 *eeprom_data;
	int eeprom_fd;
	u16 eeprom_size;

	bool cmd_mode;

	bool ibf_cal;
	/* intermediate data */
	u8 ibf_mcs;
	u8 ibf_ant;
};

struct atenl_cmd_hdr {
	__be32 magic_no;
	__be16 cmd_type;
	__be16 cmd_id;
	__be16 len;
	__be16 seq;
	u8 data[2048];
} __attribute__((packed));

enum atenl_cmd {
	HQA_CMD_UNKNOWN,
	HQA_CMD_LEGACY,	/* legacy or deprecated */

	HQA_CMD_OPEN_ADAPTER,
	HQA_CMD_CLOSE_ADAPTER,
	HQA_CMD_GET_CHIP_ID,
	HQA_CMD_GET_SUB_CHIP_ID,
	HQA_CMD_SET_TX_BW,
	HQA_CMD_SET_TX_PKT_BW,
	HQA_CMD_SET_TX_PRI_BW,
	HQA_CMD_GET_TX_INFO,
	HQA_CMD_SET_TX_PATH,
	HQA_CMD_SET_TX_POWER,
	HQA_CMD_SET_TX_POWER_MANUAL,
	HQA_CMD_SET_RF_MODE,
	HQA_CMD_SET_RX_PATH,
	HQA_CMD_SET_RX_PKT_LEN,
	HQA_CMD_SET_FREQ_OFFSET,
	HQA_CMD_SET_TSSI,
	HQA_CMD_SET_CFG,
	HQA_CMD_SET_RU,
	HQA_CMD_SET_BAND,
	HQA_CMD_SET_EEPROM_TO_FW,
	HQA_CMD_READ_MAC_BBP_REG,
	HQA_CMD_READ_MAC_BBP_REG_QA,
	HQA_CMD_READ_RF_REG,
	HQA_CMD_READ_EEPROM_BULK,
	HQA_CMD_READ_TEMPERATURE,
	HQA_CMD_WRITE_MAC_BBP_REG,
	HQA_CMD_WRITE_RF_REG,
	HQA_CMD_WRITE_EEPROM_BULK,
	HQA_CMD_WRITE_BUFFER_DONE,
	HQA_CMD_GET_BAND,
	HQA_CMD_GET_CFG,
	HQA_CMD_GET_TX_POWER,
	HQA_CMD_GET_TX_TONE_POWER,
	HQA_CMD_GET_EFUSE_FREE_BLOCK,
	HQA_CMD_GET_FREQ_OFFSET,
	HQA_CMD_GET_FW_INFO,
	HQA_CMD_GET_RX_INFO,
	HQA_CMD_GET_RF_CAP,
	HQA_CMD_CHECK_EFUSE_MODE,
	HQA_CMD_CHECK_EFUSE_MODE_TYPE,
	HQA_CMD_CHECK_EFUSE_MODE_NATIVE,
	HQA_CMD_ANT_SWAP_CAP,
	HQA_CMD_RESET_TX_RX_COUNTER,
	HQA_CMD_CONTINUOUS_TX,

	HQA_CMD_EXT,
	HQA_CMD_ERR,

	__HQA_CMD_MAX_NUM,
};

enum atenl_ext_cmd {
	HQA_EXT_CMD_UNSPEC,

	HQA_EXT_CMD_SET_CHANNEL,
	HQA_EXT_CMD_SET_TX,
	HQA_EXT_CMD_START_TX,
	HQA_EXT_CMD_START_RX,
	HQA_EXT_CMD_STOP_TX,
	HQA_EXT_CMD_STOP_RX,
	HQA_EXT_CMD_SET_TX_TIME_OPT,

	HQA_EXT_CMD_OFF_CH_SCAN,

	HQA_EXT_CMD_IBF_SET_VAL,
	HQA_EXT_CMD_IBF_GET_STATUS,
	HQA_EXT_CMD_IBF_PROF_UPDATE_ALL,

	HQA_EXT_CMD_ERR,

	__HQA_EXT_CMD_MAX_NUM,
};

struct atenl_data {
	u8 buf[RACFG_PKT_MAX_SIZE];
	int len;
	u16 cmd_id;
	u8 ext_id;
	enum atenl_cmd cmd;
	enum atenl_ext_cmd ext_cmd;
};

struct atenl_ops {
	int (*ops)(struct atenl *an, struct atenl_data *data);
	u8 cmd;
	u8 flags;
	u16 cmd_id;
	u16 resp_len;
};

#define ATENL_OPS_FLAG_EXT_CMD	BIT(0)
#define ATENL_OPS_FLAG_LEGACY	BIT(1)
#define ATENL_OPS_FLAG_SKIP	BIT(2)

static inline struct atenl_cmd_hdr * atenl_hdr(struct atenl_data *data)
{
	u8 *hqa_data = (u8 *)data->buf + ETH_HLEN;

	return (struct atenl_cmd_hdr *)hqa_data;
}

enum atenl_phy_type {
	ATENL_PHY_TYPE_CCK,
	ATENL_PHY_TYPE_OFDM,
	ATENL_PHY_TYPE_HT,
	ATENL_PHY_TYPE_HT_GF,
	ATENL_PHY_TYPE_VHT,
	ATENL_PHY_TYPE_HE_SU = 8,
	ATENL_PHY_TYPE_HE_EXT_SU,
	ATENL_PHY_TYPE_HE_TB,
	ATENL_PHY_TYPE_HE_MU,
};

enum atenl_e2p_mode {
	E2P_EFUSE_MODE = 1,
	E2P_FLASH_MODE,
	E2P_EEPROM_MODE,
	E2P_BIN_MODE,
};

enum atenl_band_type {
	BAND_TYPE_UNUSE,
	BAND_TYPE_2G,
	BAND_TYPE_5G,
	BAND_TYPE_2G_5G,
	BAND_TYPE_6G,
	BAND_TYPE_2G_6G,
	BAND_TYPE_5G_6G,
	BAND_TYPE_2G_5G_6G,
};

enum atenl_ch_band {
	CH_BAND_2GHZ,
	CH_BAND_5GHZ,
	CH_BAND_6GHZ,
};

/* for mt7915 */
enum {
	MT_EE_BAND_SEL_DEFAULT,
	MT_EE_BAND_SEL_5GHZ,
	MT_EE_BAND_SEL_2GHZ,
	MT_EE_BAND_SEL_DUAL,
};

/* for mt7916/mt7986 */
enum {
	MT_EE_BAND_SEL_2G,
	MT_EE_BAND_SEL_5G,
	MT_EE_BAND_SEL_6G,
	MT_EE_BAND_SEL_5G_6G,
};

#define MT_EE_WIFI_CONF				0x190
#define MT_EE_WIFI_CONF0_BAND_SEL		GENMASK(7, 6)

enum {
	MT7976_ONE_ADIE_DBDC		= 0x7,
	MT7975_ONE_ADIE_SINGLE_BAND	= 0x8, /* AX7800 */
	MT7976_ONE_ADIE_SINGLE_BAND	= 0xa, /* AX7800 */
	MT7975_DUAL_ADIE_DBDC		= 0xd, /* AX6000 */
	MT7976_DUAL_ADIE_DBDC		= 0xf, /* AX6000 */
};

enum {
	TEST_CBW_20MHZ,
	TEST_CBW_40MHZ,
	TEST_CBW_80MHZ,
	TEST_CBW_10MHZ,
	TEST_CBW_5MHZ,
	TEST_CBW_160MHZ,
	TEST_CBW_8080MHZ,

	TEST_CBW_MAX = TEST_CBW_8080MHZ - 1,
};

struct atenl_rx_info_hdr {
	__be32 type;
	__be32 ver;
	__be32 val;
	__be32 len;
} __attribute__((packed));

struct atenl_rx_info_band {
	__be32 mac_rx_fcs_err_cnt;
	__be32 mac_rx_mdrdy_cnt;
	__be32 mac_rx_len_mismatch;
	__be32 mac_rx_fcs_ok_cnt;
	__be32 phy_rx_fcs_err_cnt_cck;
	__be32 phy_rx_fcs_err_cnt_ofdm;
	__be32 phy_rx_pd_cck;
	__be32 phy_rx_pd_ofdm;
	__be32 phy_rx_sig_err_cck;
	__be32 phy_rx_sfd_err_cck;
	__be32 phy_rx_sig_err_ofdm;
	__be32 phy_rx_tag_err_ofdm;
	__be32 phy_rx_mdrdy_cnt_cck;
	__be32 phy_rx_mdrdy_cnt_ofdm;
} __attribute__((packed));

struct atenl_rx_info_path {
	__be32 rcpi;
	__be32 rssi;
	__be32 fagc_ib_rssi;
	__be32 fagc_wb_rssi;
	__be32 inst_ib_rssi;
	__be32 inst_wb_rssi;
} __attribute__((packed));

struct atenl_rx_info_user {
	__be32 freq_offset;
	__be32 snr;
	__be32 fcs_error_cnt;
} __attribute__((packed));

struct atenl_rx_info_comm {
	__be32 rx_fifo_full;
	__be32 aci_hit_low;
	__be32 aci_hit_high;
	__be32 mu_pkt_count;
	__be32 sig_mcs;
	__be32 sinr;
	__be32 driver_rx_count;
} __attribute__((packed));

enum atenl_ibf_action {
	TXBF_ACT_INIT = 1,
	TXBF_ACT_CHANNEL,
	TXBF_ACT_MCS,
	TXBF_ACT_POWER,
	TXBF_ACT_TX_ANT,
	TXBF_ACT_RX_START,
	TXBF_ACT_RX_ANT,
	TXBF_ACT_LNA_GAIN,
	TXBF_ACT_IBF_PHASE_COMP,
	TXBF_ACT_TX_PKT,
	TXBF_ACT_IBF_PROF_UPDATE,
	TXBF_ACT_EBF_PROF_UPDATE,
	TXBF_ACT_IBF_PHASE_CAL,
	TXBF_ACT_IBF_PHASE_E2P_UPDATE = 16,
};

static inline bool is_mt7915(struct atenl *an)
{
	return an->chip_id == 0x7915;
}

static inline bool is_mt7916(struct atenl *an)
{
	return (an->chip_id == 0x7916) || (an->chip_id == 0x7906);
}

static inline bool is_mt7986(struct atenl *an)
{
	return an->chip_id == 0x7986;
}

int atenl_eth_init(struct atenl *an);
int atenl_eth_recv(struct atenl *an, struct atenl_data *data);
int atenl_eth_send(struct atenl *an, struct atenl_data *data);
int atenl_hqa_proc_cmd(struct atenl *an);
int atenl_nl_process(struct atenl *an, struct atenl_data *data);
int atenl_nl_process_many(struct atenl *an, struct atenl_data *data);
int atenl_nl_check_mtd(struct atenl *an);
int atenl_nl_write_eeprom(struct atenl *an, u32 offset, u8 *val, int len);
int atenl_nl_write_efuse_all(struct atenl *an);
int atenl_nl_update_buffer_mode(struct atenl *an);
int atenl_nl_set_state(struct atenl *an, u8 band,
		       enum mt76_testmode_state state);
int atenl_nl_set_aid(struct atenl *an, u8 band, u8 aid);
int atenl_eeprom_init(struct atenl *an, u8 phy_idx);
void atenl_eeprom_close(struct atenl *an);
int atenl_eeprom_write_mtd(struct atenl *an);
int atenl_eeprom_read_from_driver(struct atenl *an, u32 offset, int len);
void atenl_eeprom_cmd_handler(struct atenl *an, u8 phy_idx, char *cmd);
u16 atenl_get_center_channel(u8 bw, u8 ch_band, u16 ctrl_ch);
int atenl_reg_read(struct atenl *an, u32 offset, u32 *res);
int atenl_reg_write(struct atenl *an, u32 offset, u32 val);
int atenl_rf_read(struct atenl *an, u32 wf_sel, u32 offset, u32 *res);
int atenl_rf_write(struct atenl *an, u32 wf_sel, u32 offset, u32 val);

#endif
