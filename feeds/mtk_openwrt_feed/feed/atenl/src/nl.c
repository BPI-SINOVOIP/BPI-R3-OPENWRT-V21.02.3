/* Copyright (C) 2021-2022 Mediatek Inc. */
#define _GNU_SOURCE

#include <unl.h>

#include "atenl.h"

#define to_rssi(_rcpi)	((_rcpi - 220) / 2)

struct atenl_nl_priv {
	struct atenl *an;
	struct unl unl;
	struct nl_msg *msg;
	int attr;
	void *res;
};

struct atenl_nl_ops {
	int set;
	int dump;
	int (*ops)(struct atenl *an, struct atenl_data *data,
		   struct atenl_nl_priv *nl_priv);
};

static struct nla_policy testdata_policy[NUM_MT76_TM_ATTRS] = {
	[MT76_TM_ATTR_STATE] = { .type = NLA_U8 },
	[MT76_TM_ATTR_MTD_PART] = { .type = NLA_STRING },
	[MT76_TM_ATTR_MTD_OFFSET] = { .type = NLA_U32 },
	[MT76_TM_ATTR_TX_COUNT] = { .type = NLA_U32 },
	[MT76_TM_ATTR_TX_LENGTH] = { .type = NLA_U32 },
	[MT76_TM_ATTR_TX_RATE_MODE] = { .type = NLA_U8 },
	[MT76_TM_ATTR_TX_RATE_NSS] = { .type = NLA_U8 },
	[MT76_TM_ATTR_TX_RATE_IDX] = { .type = NLA_U8 },
	[MT76_TM_ATTR_TX_RATE_SGI] = { .type = NLA_U8 },
	[MT76_TM_ATTR_TX_RATE_LDPC] = { .type = NLA_U8 },
	[MT76_TM_ATTR_TX_RATE_STBC] = { .type = NLA_U8 },
	[MT76_TM_ATTR_TX_LTF] = { .type = NLA_U8 },
	[MT76_TM_ATTR_TX_POWER_CONTROL] = { .type = NLA_U8 },
	[MT76_TM_ATTR_TX_ANTENNA] = { .type = NLA_U8 },
	[MT76_TM_ATTR_FREQ_OFFSET] = { .type = NLA_U32 },
	[MT76_TM_ATTR_STATS] = { .type = NLA_NESTED },
};

static struct nla_policy stats_policy[NUM_MT76_TM_STATS_ATTRS] = {
	[MT76_TM_STATS_ATTR_TX_PENDING] = { .type = NLA_U32 },
	[MT76_TM_STATS_ATTR_TX_QUEUED] = { .type = NLA_U32 },
	[MT76_TM_STATS_ATTR_TX_DONE] = { .type = NLA_U32 },
	[MT76_TM_STATS_ATTR_RX_PACKETS] = { .type = NLA_U64 },
	[MT76_TM_STATS_ATTR_RX_FCS_ERROR] = { .type = NLA_U64 },
};

static struct nla_policy rx_policy[NUM_MT76_TM_RX_ATTRS] = {
	[MT76_TM_RX_ATTR_FREQ_OFFSET] = { .type = NLA_U32 },
	[MT76_TM_RX_ATTR_RCPI] = { .type = NLA_NESTED },
	[MT76_TM_RX_ATTR_IB_RSSI] = { .type = NLA_NESTED },
	[MT76_TM_RX_ATTR_WB_RSSI] = { .type = NLA_NESTED },
	[MT76_TM_RX_ATTR_SNR] = { .type = NLA_U8 },
};

struct he_sgi {
	enum mt76_testmode_tx_mode tx_mode;
	u8 sgi;
	u8 tx_ltf;
};

#define HE_SGI_GROUP(_tx_mode, _sgi, _tx_ltf)	\
	{ .tx_mode = MT76_TM_TX_MODE_##_tx_mode, .sgi = _sgi, .tx_ltf = _tx_ltf }
static const struct he_sgi he_sgi_groups[] = {
	HE_SGI_GROUP(HE_SU, 0, 0),
	HE_SGI_GROUP(HE_SU, 0, 1),
	HE_SGI_GROUP(HE_SU, 1, 1),
	HE_SGI_GROUP(HE_SU, 2, 2),
	HE_SGI_GROUP(HE_SU, 0, 2),
	HE_SGI_GROUP(HE_EXT_SU, 0, 0),
	HE_SGI_GROUP(HE_EXT_SU, 0, 1),
	HE_SGI_GROUP(HE_EXT_SU, 1, 1),
	HE_SGI_GROUP(HE_EXT_SU, 2, 2),
	HE_SGI_GROUP(HE_EXT_SU, 0, 2),
	HE_SGI_GROUP(HE_TB, 1, 0),
	HE_SGI_GROUP(HE_TB, 1, 1),
	HE_SGI_GROUP(HE_TB, 2, 2),
	HE_SGI_GROUP(HE_MU, 0, 2),
	HE_SGI_GROUP(HE_MU, 0, 1),
	HE_SGI_GROUP(HE_MU, 1, 1),
	HE_SGI_GROUP(HE_MU, 2, 2),
};
#undef HE_SGI_LTF_GROUP

static u8 phy_type_to_attr(u8 phy_type)
{
	static const u8 phy_type_to_attr[] = {
		[ATENL_PHY_TYPE_CCK] = MT76_TM_TX_MODE_CCK,
		[ATENL_PHY_TYPE_OFDM] = MT76_TM_TX_MODE_OFDM,
		[ATENL_PHY_TYPE_HT] = MT76_TM_TX_MODE_HT,
		[ATENL_PHY_TYPE_HT_GF] = MT76_TM_TX_MODE_HT,
		[ATENL_PHY_TYPE_VHT] = MT76_TM_TX_MODE_VHT,
		[ATENL_PHY_TYPE_HE_SU] = MT76_TM_TX_MODE_HE_SU,
		[ATENL_PHY_TYPE_HE_EXT_SU] = MT76_TM_TX_MODE_HE_EXT_SU,
		[ATENL_PHY_TYPE_HE_TB] = MT76_TM_TX_MODE_HE_TB,
		[ATENL_PHY_TYPE_HE_MU] = MT76_TM_TX_MODE_HE_MU,
	};

	if (phy_type >= ARRAY_SIZE(phy_type_to_attr))
		return 0;

	return phy_type_to_attr[phy_type];
}

static void
atenl_set_attr_state(struct atenl *an, struct nl_msg *msg,
		     u8 band, enum mt76_testmode_state state)
{
	if (get_band_val(an, band, cur_state) == state)
		return;

	nla_put_u8(msg, MT76_TM_ATTR_STATE, state);
	set_band_val(an, band, cur_state, state);
}

static void
atenl_set_attr_tx_length(struct atenl *an, struct nl_msg *msg,
			 u32 tx_length, bool is_mu)
{
#define IEEE80211_MAX_FRAME_LEN		2352
	int hdr_len = 24;

	if (tx_length > IEEE80211_MAX_FRAME_LEN || is_mu)
		hdr_len += 2;

	if (an->ibf_cal)
		hdr_len = 0;

	/* The definition of MT76_TM_ATTR_TX_LENGTH is MPDU length,
	 * so add ieee80211_hdr length.
	 */
	nla_put_u32(msg, MT76_TM_ATTR_TX_LENGTH, tx_length + hdr_len);
}

static void
atenl_set_attr_antenna(struct atenl *an, struct nl_msg *msg, u8 tx_antenna)
{
	if (!tx_antenna)
		return;

	if (is_mt7915(an))
		nla_put_u8(msg, MT76_TM_ATTR_TX_ANTENNA,
			   tx_antenna << (2 * an->cur_band));
	else if (is_mt7916(an) || is_mt7986(an))
		nla_put_u8(msg, MT76_TM_ATTR_TX_ANTENNA, tx_antenna);
}

static int
atenl_nl_set_attr(struct atenl *an, struct atenl_data *data,
		  struct atenl_nl_priv *nl_priv)
{
	struct atenl_cmd_hdr *hdr = atenl_hdr(data);
	struct nl_msg *msg = nl_priv->msg;
	u32 val = ntohl(*(u32 *)hdr->data);
	int attr = nl_priv->attr;
	void *ptr, *a;

	ptr = nla_nest_start(msg, NL80211_ATTR_TESTDATA);
	if (!ptr)
		return -ENOMEM;

	switch (attr) {
	case MT76_TM_ATTR_TX_ANTENNA:
		atenl_set_attr_antenna(an, msg, val);
		break;
	case MT76_TM_ATTR_FREQ_OFFSET:
		nla_put_u32(msg, attr, val);
		break;
	case MT76_TM_ATTR_TX_POWER:
		a = nla_nest_start(msg, MT76_TM_ATTR_TX_POWER);
		if (!a)
			return -ENOMEM;
		nla_put_u8(msg, 0, val);
		nla_nest_end(msg, a);
		break;
	default:
		nla_put_u8(msg, attr, val);
		break;
	}

	nla_nest_end(msg, ptr);

	return unl_genl_request(&nl_priv->unl, msg, NULL, NULL);
}

static int
atenl_nl_set_cfg(struct atenl *an, struct atenl_data *data,
		 struct atenl_nl_priv *nl_priv)
{
	struct atenl_cmd_hdr *hdr = atenl_hdr(data);
	struct nl_msg *msg = nl_priv->msg;
	enum atenl_cmd cmd = data->cmd;
	u32 *v = (u32 *)hdr->data;
	u8 type = ntohl(v[0]);
	u8 enable = ntohl(v[1]);
	void *ptr, *cfg;

	if (cmd == HQA_CMD_SET_TSSI) {
		type = 0;
		enable = 1;
	}

	ptr = nla_nest_start(msg, NL80211_ATTR_TESTDATA);
	if (!ptr)
		return -ENOMEM;

	cfg = nla_nest_start(msg, MT76_TM_ATTR_CFG);
	if (!cfg)
		return -ENOMEM;

	if (nla_put_u8(msg, 0, type) ||
	    nla_put_u8(msg, 1, enable))
		return -EINVAL;

	nla_nest_end(msg, cfg);

	nla_nest_end(msg, ptr);

	return unl_genl_request(&nl_priv->unl, msg, NULL, NULL);
}

static int
atenl_nl_set_tx(struct atenl *an, struct atenl_data *data,
		struct atenl_nl_priv *nl_priv)
{
	struct atenl_cmd_hdr *hdr = atenl_hdr(data);
	struct nl_msg *msg = nl_priv->msg;
	u32 *v = (u32 *)hdr->data;
	u8 *addr1 = hdr->data + 36;
	u8 *addr2 = addr1 + ETH_ALEN;
	u8 *addr3 = addr2 + ETH_ALEN;
	u8 def_mac[ETH_ALEN] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
	void *ptr, *a;

	if (get_band_val(an, an->cur_band, use_tx_time))
		set_band_val(an, an->cur_band, tx_time, ntohl(v[7]));
	else
		set_band_val(an, an->cur_band, tx_mpdu_len, ntohl(v[7]));

	ptr = nla_nest_start(msg, NL80211_ATTR_TESTDATA);
	if (!ptr)
		return -ENOMEM;

	a = nla_nest_start(msg, MT76_TM_ATTR_MAC_ADDRS);
	if (!a)
		return -ENOMEM;

	nla_put(msg, 0, ETH_ALEN, use_default_addr(addr1) ? def_mac : addr1);
	nla_put(msg, 1, ETH_ALEN, use_default_addr(addr2) ? def_mac : addr2);
	nla_put(msg, 2, ETH_ALEN, use_default_addr(addr3) ? def_mac : addr3);

	nla_nest_end(msg, a);

	nla_nest_end(msg, ptr);

	*(u32 *)(hdr->data + 2) = data->ext_id;

	return unl_genl_request(&nl_priv->unl, msg, NULL, NULL);
}

static int
atenl_nl_tx(struct atenl *an, struct atenl_data *data, struct atenl_nl_priv *nl_priv)
{
#define USE_SPE_IDX	BIT(31)
	struct atenl_cmd_hdr *hdr = atenl_hdr(data);
	struct nl_msg *msg = nl_priv->msg;
	u32 *v = (u32 *)hdr->data;
	u8 band = ntohl(v[2]);
	void *ptr;
	int ret;

	if (band >= MAX_BAND_NUM)
		return -EINVAL;

	ptr = nla_nest_start(msg, NL80211_ATTR_TESTDATA);
	if (!ptr)
		return -ENOMEM;

	if (data->ext_cmd == HQA_EXT_CMD_STOP_TX) {
		atenl_set_attr_state(an, msg, band, MT76_TM_STATE_IDLE);
	} else {
		u32 tx_count = ntohl(v[3]);
		u8 tx_rate_mode = phy_type_to_attr(ntohl(v[4]));
		u8 aid = ntohl(v[11]);
		u8 sgi = ntohl(v[13]);
		u32 tx_antenna = ntohl(v[14]);
		void *a;

		if (sgi > 5)
			return -EINVAL;

		if (!tx_count)
			tx_count = 10000000;

		nla_put_u32(msg, MT76_TM_ATTR_TX_COUNT, tx_count);
		nla_put_u32(msg, MT76_TM_ATTR_TX_IPG, ntohl(v[12]));
		nla_put_u8(msg, MT76_TM_ATTR_TX_RATE_MODE, tx_rate_mode);
		nla_put_u8(msg, MT76_TM_ATTR_TX_RATE_IDX, ntohl(v[5]));
		nla_put_u8(msg, MT76_TM_ATTR_TX_RATE_STBC, ntohl(v[7]));
		nla_put_u8(msg, MT76_TM_ATTR_TX_RATE_LDPC, ntohl(v[8]));
		nla_put_u8(msg, MT76_TM_ATTR_TX_RATE_NSS, ntohl(v[15]));

		if (get_band_val(an, band, use_tx_time))
			nla_put_u32(msg, MT76_TM_ATTR_TX_TIME,
				    get_band_val(an, band, tx_time));
		else
			atenl_set_attr_tx_length(an, msg,
					get_band_val(an, band, tx_mpdu_len),
					tx_rate_mode == MT76_TM_TX_MODE_HE_MU);

		/* for chips after 7915, tx need to use at least wcid = 1 */
		if (!is_mt7915(an) && !aid)
			aid = 1;
		nla_put_u8(msg, MT76_TM_ATTR_AID, aid);

		if (tx_antenna & USE_SPE_IDX) {
			nla_put_u8(msg, MT76_TM_ATTR_TX_SPE_IDX,
				   tx_antenna & ~USE_SPE_IDX);
		} else {
			nla_put_u8(msg, MT76_TM_ATTR_TX_SPE_IDX, 0);
			atenl_set_attr_antenna(an, msg, tx_antenna);
		}

		if (tx_rate_mode >= MT76_TM_TX_MODE_HE_SU) {
			u8 ofs = sgi;
			size_t i;

			for (i = 0; i < ARRAY_SIZE(he_sgi_groups); i++)
				if (he_sgi_groups[i].tx_mode == tx_rate_mode)
					break;

			if ((i + ofs) >= ARRAY_SIZE(he_sgi_groups))
				return -EINVAL;

			sgi = he_sgi_groups[i + ofs].sgi;
			nla_put_u8(msg, MT76_TM_ATTR_TX_LTF,
				   he_sgi_groups[i + ofs].tx_ltf);
		}
		nla_put_u8(msg, MT76_TM_ATTR_TX_RATE_SGI, sgi);

		a = nla_nest_start(msg, MT76_TM_ATTR_TX_POWER);
		if (!a)
			return -ENOMEM;
		nla_put_u8(msg, 0, ntohl(v[6]));
		nla_nest_end(msg, a);

		atenl_set_attr_state(an, msg, band, MT76_TM_STATE_TX_FRAMES);
	}

	nla_nest_end(msg, ptr);

	ret = unl_genl_request(&nl_priv->unl, msg, NULL, NULL);
	if (ret)
		return ret;

	*(u32 *)(hdr->data + 2) = data->ext_id;

	return 0;
}

static int
atenl_nl_rx(struct atenl *an, struct atenl_data *data, struct atenl_nl_priv *nl_priv)
{
	struct atenl_cmd_hdr *hdr = atenl_hdr(data);
	struct atenl_band *anb = &an->anb[an->cur_band];
	struct nl_msg *msg = nl_priv->msg;
	u32 *v = (u32 *)hdr->data;
	u8 band = ntohl(v[2]);
	void *ptr;

	if (band >= MAX_BAND_NUM)
		return -EINVAL;

	ptr = nla_nest_start(msg, NL80211_ATTR_TESTDATA);
	if (!ptr)
		return -ENOMEM;

	if (data->ext_cmd == HQA_EXT_CMD_STOP_RX) {
		atenl_set_attr_state(an, msg, band, MT76_TM_STATE_IDLE);
	} else {
		v = (u32 *)(hdr->data + 18);

		atenl_set_attr_antenna(an, msg, ntohl(v[0]));
		nla_put_u8(msg, MT76_TM_ATTR_AID, ntohl(v[1]));
		atenl_set_attr_state(an, msg, band, MT76_TM_STATE_RX_FRAMES);

		anb->reset_rx_cnt = false;

		/* clear history buffer */
		memset(&anb->rx_stat, 0, sizeof(anb->rx_stat));
	}

	nla_nest_end(msg, ptr);

	*(u32 *)(hdr->data + 2) = data->ext_id;

	return unl_genl_request(&nl_priv->unl, msg, NULL, NULL);
}

static int
atenl_off_ch_scan(struct atenl *an, struct atenl_data *data,
		  struct atenl_nl_priv *nl_priv)
{
	struct atenl_cmd_hdr *hdr = atenl_hdr(data);
	struct nl_msg *msg = nl_priv->msg;
	u32 *v = (u32 *)hdr->data;
	u8 ch = ntohl(v[2]);
	u8 bw = ntohl(v[4]);
	u8 tx_path = ntohl(v[5]);
	u8 status = ntohl(v[6]);
	void *ptr;

	if (!status)
		ch = 0; /* stop */

	ptr = nla_nest_start(msg, NL80211_ATTR_TESTDATA);
	if (!ptr)
		return -ENOMEM;

	nla_put_u8(msg, MT76_TM_ATTR_OFF_CH_SCAN_CH, ch);
	nla_put_u8(msg, MT76_TM_ATTR_OFF_CH_SCAN_CENTER_CH,
		   atenl_get_center_channel(bw, CH_BAND_5GHZ, ch));
	nla_put_u8(msg, MT76_TM_ATTR_OFF_CH_SCAN_BW, bw);
	nla_put_u8(msg, MT76_TM_ATTR_OFF_CH_SCAN_PATH, tx_path);

	nla_nest_end(msg, ptr);

	*(u32 *)(hdr->data + 2) = data->ext_id;

	return 0;
}

static int atenl_nl_dump_cb(struct nl_msg *msg, void *arg)
{
	struct atenl_nl_priv *nl_priv = (struct atenl_nl_priv *)arg;
	struct nlattr *tb1[NUM_MT76_TM_ATTRS];
	struct nlattr *tb2[NUM_MT76_TM_STATS_ATTRS];
	struct nlattr *nl_attr;
	int attr = nl_priv->attr;
	u64 *res = nl_priv->res;

	nl_attr = unl_find_attr(&nl_priv->unl, msg, NL80211_ATTR_TESTDATA);
	if (!nl_attr) {
		atenl_err("Testdata attribute not found\n");
		return NL_SKIP;
	}

	nla_parse_nested(tb1, MT76_TM_ATTR_MAX, nl_attr, testdata_policy);
	nla_parse_nested(tb2, MT76_TM_STATS_ATTR_MAX,
			 tb1[MT76_TM_ATTR_STATS], stats_policy);

	if (attr == MT76_TM_STATS_ATTR_TX_DONE)
		*res = nla_get_u32(tb2[MT76_TM_STATS_ATTR_TX_DONE]);

	return NL_SKIP;
}

static int
atenl_nl_dump_attr(struct atenl *an, struct atenl_data *data,
		   struct atenl_nl_priv *nl_priv)
{
	struct atenl_cmd_hdr *hdr = atenl_hdr(data);
	struct nl_msg *msg = nl_priv->msg;
	void *ptr;
	u64 res = 0;

	nl_priv->res = (void *)&res;

	ptr = nla_nest_start(msg, NL80211_ATTR_TESTDATA);
	if (!ptr)
		return -ENOMEM;
	nla_put_flag(msg, MT76_TM_ATTR_STATS);
	nla_nest_end(msg, ptr);

	unl_genl_request(&nl_priv->unl, msg, atenl_nl_dump_cb, (void *)nl_priv);

	if (nl_priv->attr == MT76_TM_STATS_ATTR_TX_DONE)
		*(u32 *)(hdr->data + 2 + 4 * an->cur_band) = htonl(res);

	return 0;
}

static int atenl_nl_continuous_tx(struct atenl *an,
				  struct atenl_data *data,
				  struct atenl_nl_priv *nl_priv)
{
	struct atenl_cmd_hdr *hdr = atenl_hdr(data);
	struct nl_msg *msg = nl_priv->msg;
	u32 *v = (u32 *)hdr->data;
	u8 band = ntohl(v[0]);
	bool enable = ntohl(v[1]);
	void *ptr;

	ptr = nla_nest_start(msg, NL80211_ATTR_TESTDATA);
	if (!ptr)
		return -ENOMEM;

	if (band >= MAX_BAND_NUM)
		return -EINVAL;

	if (!enable) {
		int phy = get_band_val(an, band, phy_idx);
		char cmd[64];

		atenl_set_attr_state(an, msg, band, MT76_TM_STATE_IDLE);
		nla_nest_end(msg, ptr);
		unl_genl_request(&nl_priv->unl, msg, NULL, NULL);

		sprintf(cmd, "iw dev mon%d del", phy);
		system(cmd);
		sprintf(cmd, "iw phy phy%d interface add mon%d type monitor", phy, phy);
		system(cmd);
		sprintf(cmd, "ifconfig mon%d up", phy);
		system(cmd);

		return 0;
	}

	if (get_band_val(an, band, rf_mode) != ATENL_RF_MODE_TEST)
		return 0;

	nla_put_u8(msg, MT76_TM_ATTR_TX_ANTENNA, ntohl(v[2]));
	nla_put_u8(msg, MT76_TM_ATTR_TX_RATE_MODE, phy_type_to_attr(ntohl(v[3])));
	nla_put_u8(msg, MT76_TM_ATTR_TX_RATE_IDX, ntohl(v[6]));

	atenl_dbg("%s: enable = %d, ant=%u, tx_rate_mode=%u, rate_idx=%u\n",
		  __func__, enable, ntohl(v[2]), ntohl(v[3]), ntohl(v[6]));

	atenl_set_attr_state(an, msg, band, MT76_TM_STATE_TX_CONT);

	nla_nest_end(msg, ptr);

	return unl_genl_request(&nl_priv->unl, msg, NULL, NULL);
}

static int atenl_nl_get_rx_info_cb(struct nl_msg *msg, void *arg)
{
	struct atenl_nl_priv *nl_priv = (struct atenl_nl_priv *)arg;
	struct atenl *an = nl_priv->an;
	struct atenl_band *anb = &an->anb[an->cur_band];
	struct atenl_data *data = nl_priv->res;
	struct atenl_cmd_hdr *hdr = atenl_hdr(data);
	struct atenl_rx_info_hdr *rx_hdr;
	struct atenl_rx_info_band *rx_band;
	struct atenl_rx_info_user *rx_user;
	struct atenl_rx_info_path *rx_path;
	struct atenl_rx_info_comm *rx_comm;
	struct nlattr *tb1[NUM_MT76_TM_ATTRS];
	struct nlattr *tb2[NUM_MT76_TM_STATS_ATTRS];
	struct nlattr *tb3[NUM_MT76_TM_RX_ATTRS];
	struct nlattr *nl_attr, *cur;
	struct atenl_rx_stat rx_cur, rx_diff = {};
	u32 rcpi[4] = {};
	u32 type_num = htonl(4);
	s32 ib_rssi[4] = {}, wb_rssi[4] = {};
	u8 path = an->anb[an->cur_band].chainmask;
	u8 path_num = __builtin_popcount(path);
	u8 *buf = hdr->data + 2;
	int i, rem;

	*(u32 *)buf = type_num;
	buf += sizeof(type_num);

#define RX_PUT_HDR(_hdr, _type, _val, _size) do {	\
		_hdr->type = htonl(_type);		\
		_hdr->val = htonl(_val);		\
		_hdr->len = htonl(_size);		\
		buf += sizeof(*_hdr);			\
	} while (0)

	rx_hdr = (struct atenl_rx_info_hdr *)buf;
	RX_PUT_HDR(rx_hdr, 0, BIT(an->cur_band), sizeof(*rx_band));
	rx_band = (struct atenl_rx_info_band *)buf;
	buf += sizeof(*rx_band);

	rx_hdr = (struct atenl_rx_info_hdr *)buf;
	RX_PUT_HDR(rx_hdr, 1, path, path_num * sizeof(*rx_path));
	rx_path = (struct atenl_rx_info_path *)buf;
	buf += path_num * sizeof(*rx_path);

	rx_hdr = (struct atenl_rx_info_hdr *)buf;
	RX_PUT_HDR(rx_hdr, 2, GENMASK(15, 0), 16 * sizeof(*rx_user));
	rx_user = (struct atenl_rx_info_user *)buf;
	buf += 16 * sizeof(*rx_user);

	rx_hdr = (struct atenl_rx_info_hdr *)buf;
	RX_PUT_HDR(rx_hdr, 3, BIT(0), sizeof(*rx_comm));
	rx_comm = (struct atenl_rx_info_comm *)buf;
	buf += sizeof(*rx_comm);

	hdr->len = htons(buf - hdr->data);

	nl_attr = unl_find_attr(&nl_priv->unl, msg, NL80211_ATTR_TESTDATA);
	if (!nl_attr) {
		atenl_err("Testdata attribute not found\n");
		return NL_SKIP;
	}

	nla_parse_nested(tb1, MT76_TM_ATTR_MAX, nl_attr, testdata_policy);
	nla_parse_nested(tb2, MT76_TM_STATS_ATTR_MAX,
			 tb1[MT76_TM_ATTR_STATS], stats_policy);

	rx_cur.total = nla_get_u64(tb2[MT76_TM_STATS_ATTR_RX_PACKETS]);
	rx_cur.err_cnt = nla_get_u64(tb2[MT76_TM_STATS_ATTR_RX_FCS_ERROR]);
	rx_cur.len_mismatch = nla_get_u64(tb2[MT76_TM_STATS_ATTR_RX_LEN_MISMATCH]);
	rx_cur.ok_cnt = rx_cur.total - rx_cur.err_cnt - rx_cur.len_mismatch;

	if (!anb->reset_rx_cnt ||
	    get_band_val(an, an->cur_band, cur_state) == MT76_TM_STATE_RX_FRAMES) {
#define RX_COUNT_DIFF(_field)	\
	rx_diff._field = (rx_cur._field) - (anb->rx_stat._field);
		RX_COUNT_DIFF(total);
		RX_COUNT_DIFF(err_cnt);
		RX_COUNT_DIFF(len_mismatch);
		RX_COUNT_DIFF(ok_cnt);
#undef RX_COUNT_DIFF

		memcpy(&anb->rx_stat, &rx_cur, sizeof(anb->rx_stat));
	}

	rx_band->mac_rx_mdrdy_cnt = htonl((u32)rx_diff.total);
	rx_band->mac_rx_fcs_err_cnt = htonl((u32)rx_diff.err_cnt);
	rx_band->mac_rx_fcs_ok_cnt = htonl((u32)rx_diff.ok_cnt);
	rx_band->mac_rx_len_mismatch = htonl((u32)rx_diff.len_mismatch);
	rx_user->fcs_error_cnt = htonl((u32)rx_diff.err_cnt);

	nla_parse_nested(tb3, MT76_TM_RX_ATTR_MAX,
			 tb2[MT76_TM_STATS_ATTR_LAST_RX], rx_policy);

	rx_user->freq_offset = htonl(nla_get_u32(tb3[MT76_TM_RX_ATTR_FREQ_OFFSET]));
	rx_user->snr = htonl(nla_get_u8(tb3[MT76_TM_RX_ATTR_SNR]));

	i = 0;
	nla_for_each_nested(cur, tb3[MT76_TM_RX_ATTR_RCPI], rem) {
		if (nla_len(cur) != 1 || i >= 4)
			break;

		rcpi[i++] = nla_get_u8(cur);
	}

	i = 0;
	nla_for_each_nested(cur, tb3[MT76_TM_RX_ATTR_IB_RSSI], rem) {
		if (nla_len(cur) != 1 || i >= 4)
			break;

		ib_rssi[i++] = (s8)nla_get_u8(cur);
	}

	i = 0;
	nla_for_each_nested(cur, tb3[MT76_TM_RX_ATTR_WB_RSSI], rem) {
		if (nla_len(cur) != 1 || i >= 4)
			break;

		wb_rssi[i++] = (s8)nla_get_u8(cur);
	}

	for (i = 0; i < 4; i++) {
		struct atenl_rx_info_path *path = &rx_path[i];

		path->rcpi = htonl(rcpi[i]);
		path->rssi = htonl(to_rssi((u8)rcpi[i]));
		path->fagc_ib_rssi = htonl(ib_rssi[i]);
		path->fagc_wb_rssi = htonl(wb_rssi[i]);
	}

	return NL_SKIP;
}

static int atenl_nl_get_rx_info(struct atenl *an, struct atenl_data *data,
				struct atenl_nl_priv *nl_priv)
{
	struct nl_msg *msg = nl_priv->msg;
	void *ptr;

	nl_priv->an = an;
	nl_priv->res = (void *)data;

	ptr = nla_nest_start(msg, NL80211_ATTR_TESTDATA);
	if (!ptr)
		return -ENOMEM;

	nla_put_flag(msg, MT76_TM_ATTR_STATS);

	nla_nest_end(msg, ptr);

	return unl_genl_request(&nl_priv->unl, msg, atenl_nl_get_rx_info_cb,
				(void *)nl_priv);
}

static int
atenl_nl_set_ru(struct atenl *an, struct atenl_data *data,
		struct atenl_nl_priv *nl_priv)
{
	struct atenl_cmd_hdr *hdr = atenl_hdr(data);
	struct nl_msg *msg;
	u32 *v = (u32 *)(hdr->data + 4);
	u32 seg0_num = ntohl(v[0]);	/* v[1] seg1_num unused */
	void *ptr;
	int i, ret;

	if (seg0_num > 8)
		return -EINVAL;

	for (i = 0, v = &v[2]; i < seg0_num; i++, v += 11) {
		u32 ru_alloc = ntohl(v[1]);
		u32 aid = ntohl(v[2]);
		u32 ru_idx = ntohl(v[3]);
		u32 mcs = ntohl(v[4]);
		u32 ldpc = ntohl(v[5]);
		u32 nss = ntohl(v[6]);
		u32 tx_length = ntohl(v[8]);
		char buf[10];

		if (unl_genl_init(&nl_priv->unl, "nl80211") < 0) {
			atenl_err("Failed to connect to nl80211\n");
			return 2;
		}

		msg = unl_genl_msg(&nl_priv->unl, NL80211_CMD_TESTMODE, false);
		nla_put_u32(msg, NL80211_ATTR_WIPHY, get_band_val(an, an->cur_band, phy_idx));

		ptr = nla_nest_start(msg, NL80211_ATTR_TESTDATA);
		if (!ptr)
			return -ENOMEM;

		if (i == 0)
			atenl_set_attr_state(an, msg, an->cur_band, MT76_TM_STATE_IDLE);

		nla_put_u8(msg, MT76_TM_ATTR_AID, aid);
		nla_put_u8(msg, MT76_TM_ATTR_RU_IDX, ru_idx);
		nla_put_u8(msg, MT76_TM_ATTR_TX_RATE_IDX, mcs);
		nla_put_u8(msg, MT76_TM_ATTR_TX_RATE_LDPC, ldpc);
		nla_put_u8(msg, MT76_TM_ATTR_TX_RATE_NSS, nss);
		atenl_set_attr_tx_length(an, msg, tx_length, true);

		ret = snprintf(buf, sizeof(buf), "%x", ru_alloc);
		if (snprintf_error(sizeof(buf), ret))
			return -EINVAL;

		nla_put_u8(msg, MT76_TM_ATTR_RU_ALLOC, strtol(buf, NULL, 2));

		nla_nest_end(msg, ptr);

		unl_genl_request(&nl_priv->unl, msg, NULL, NULL);

		unl_free(&nl_priv->unl);
	}

	return 0;
}

static int
atenl_nl_ibf_init(struct atenl *an, u8 band)
{
	struct atenl_nl_priv nl_priv = {};
	struct nl_msg *msg;
	void *ptr, *a;
	int ret;

	if (unl_genl_init(&nl_priv.unl, "nl80211") < 0) {
		atenl_err("Failed to connect to nl80211\n");
		return 2;
	}

	msg = unl_genl_msg(&nl_priv.unl, NL80211_CMD_TESTMODE, false);
	nla_put_u32(msg, NL80211_ATTR_WIPHY, get_band_val(an, band, phy_idx));

	ptr = nla_nest_start(msg, NL80211_ATTR_TESTDATA);
	if (!ptr) {
		ret = -ENOMEM;
		goto out;
	}

	nla_put_u8(msg, MT76_TM_ATTR_TX_RATE_MODE, MT76_TM_TX_MODE_HT);
	nla_put_u8(msg, MT76_TM_ATTR_TX_RATE_IDX, an->ibf_mcs);
	nla_put_u8(msg, MT76_TM_ATTR_TX_ANTENNA, an->ibf_ant);
	nla_put_u8(msg, MT76_TM_ATTR_TXBF_ACT, MT76_TM_TXBF_ACT_INIT);

	a = nla_nest_start(msg, MT76_TM_ATTR_TXBF_PARAM);
	if (!a) {
		ret = -ENOMEM;
		goto out;
	}
	nla_put_u16(msg, 0, 1);
	nla_nest_end(msg, a);

	nla_nest_end(msg, ptr);

	ret = unl_genl_request(&nl_priv.unl, msg, NULL, NULL);

out:
	unl_free(&nl_priv.unl);
	return ret;
}

static int
atenl_nl_ibf_e2p_update(struct atenl *an)
{
	struct atenl_nl_priv nl_priv = {};
	struct nl_msg *msg;
	void *ptr, *a;
	int ret;

	if (unl_genl_init(&nl_priv.unl, "nl80211") < 0) {
		atenl_err("Failed to connect to nl80211\n");
		return 2;
	}

	msg = unl_genl_msg(&nl_priv.unl, NL80211_CMD_TESTMODE, false);
	nla_put_u32(msg, NL80211_ATTR_WIPHY, get_band_val(an, an->cur_band, phy_idx));

	ptr = nla_nest_start(msg, NL80211_ATTR_TESTDATA);
	if (!ptr) {
		ret = -ENOMEM;
		goto out;
	}

	nla_put_u8(msg, MT76_TM_ATTR_TXBF_ACT, MT76_TM_TXBF_ACT_E2P_UPDATE);
	a = nla_nest_start(msg, MT76_TM_ATTR_TXBF_PARAM);
	if (!a) {
		ret = -ENOMEM;
		goto out;
	}
	nla_put_u16(msg, 0, 0);
	nla_nest_end(msg, a);

	nla_nest_end(msg, ptr);

	ret = unl_genl_request(&nl_priv.unl, msg, NULL, NULL);

out:
	unl_free(&nl_priv.unl);
	return ret;
}

static void
atenl_get_ibf_cal_result(struct atenl *an)
{
	u16 offset;

	if (an->adie_id == 0x7975)
		offset = 0x651;
	else if (an->adie_id == 0x7976)
		offset = 0x60a;

	/* per group size = 40, for group 0-8 */
	atenl_eeprom_read_from_driver(an, offset, 40 * 9);
}

static int
atenl_nl_ibf_set_val(struct atenl *an, struct atenl_data *data,
		     struct atenl_nl_priv *nl_priv)
{
#define MT_IBF(_act)	MT76_TM_TXBF_ACT_##_act
	static const u8 bf_act_map[] = {
		[TXBF_ACT_IBF_PHASE_COMP] = MT_IBF(PHASE_COMP),
		[TXBF_ACT_IBF_PROF_UPDATE] = MT_IBF(IBF_PROF_UPDATE),
		[TXBF_ACT_EBF_PROF_UPDATE] = MT_IBF(EBF_PROF_UPDATE),
		[TXBF_ACT_IBF_PHASE_CAL] = MT_IBF(PHASE_CAL),
	};
#undef MT_IBF
	struct atenl_cmd_hdr *hdr = atenl_hdr(data);
	struct nl_msg *msg = nl_priv->msg;
	u32 *v = (u32 *)(hdr->data + 4);
	u32 action = ntohl(v[0]);
	u16 val[8];
	u8 tmp_ant;
	void *ptr, *a;
	char cmd[64];
	int i;

	for (i = 0; i < 8; i++)
		val[i] = ntohl(v[i + 1]);

	atenl_dbg("%s: action = %u, val = %u, %u, %u, %u, %u\n",
		  __func__, action, val[0], val[1], val[2], val[3], val[4]);

	ptr = nla_nest_start(msg, NL80211_ATTR_TESTDATA);
	if (!ptr)
		return -ENOMEM;

	switch (action) {
	case TXBF_ACT_CHANNEL:
		an->cur_band = val[1];
		/* a sanity to prevent script band idx error */
		if (val[0] > 14)
			an->cur_band = 1;
		atenl_nl_ibf_init(an, an->cur_band);
		atenl_set_channel(an, 0, an->cur_band, val[0], 0, 0);

		nla_put_u8(msg, MT76_TM_ATTR_AID, 0);
		nla_put_u8(msg, MT76_TM_ATTR_TXBF_ACT, MT76_TM_TXBF_ACT_UPDATE_CH);
		a = nla_nest_start(msg, MT76_TM_ATTR_TXBF_PARAM);
		if (!a)
			return -ENOMEM;
		nla_put_u16(msg, 0, 0);
		nla_nest_end(msg, a);

		an->ibf_cal = true;
		break;
	case TXBF_ACT_MCS:
		tmp_ant = (1 << DIV_ROUND_UP(val[0], 8)) - 1 ?: 1;
		/* sometimes the correct band idx will be set after this action,
		 * so maintain a temp variable to allow mcs update in anthor action.
		 */
		an->ibf_mcs = val[0];
		an->ibf_ant = tmp_ant;
		nla_put_u8(msg, MT76_TM_ATTR_TX_RATE_IDX, an->ibf_mcs);
		nla_put_u8(msg, MT76_TM_ATTR_TX_ANTENNA, an->ibf_ant);
		break;
	case TXBF_ACT_TX_ANT:
		nla_put_u8(msg, MT76_TM_ATTR_TX_ANTENNA, val[0]);
		break;
	case TXBF_ACT_RX_START:
		atenl_set_attr_state(an, msg, an->cur_band, MT76_TM_STATE_RX_FRAMES);
		break;
	case TXBF_ACT_RX_ANT:
		nla_put_u8(msg, MT76_TM_ATTR_TX_ANTENNA, val[0]);
		break;
	case TXBF_ACT_TX_PKT:
		nla_put_u8(msg, MT76_TM_ATTR_AID, val[1]);
		nla_put_u8(msg, MT76_TM_ATTR_TXBF_ACT, MT76_TM_TXBF_ACT_TX_PREP);
		nla_put_u32(msg, MT76_TM_ATTR_TX_COUNT, 10000000);
		atenl_set_attr_tx_length(an, msg, 1024, false);
		a = nla_nest_start(msg, MT76_TM_ATTR_TXBF_PARAM);
		if (!a)
			return -ENOMEM;

		for (i = 0; i < 5; i++)
			nla_put_u16(msg, i, val[i]);
		nla_nest_end(msg, a);

		atenl_set_attr_state(an, msg, an->cur_band, MT76_TM_STATE_TX_FRAMES);
		break;
	case TXBF_ACT_IBF_PHASE_COMP:
		nla_put_u8(msg, MT76_TM_ATTR_AID, 1);
	case TXBF_ACT_IBF_PROF_UPDATE:
	case TXBF_ACT_EBF_PROF_UPDATE:
	case TXBF_ACT_IBF_PHASE_CAL:
		nla_put_u8(msg, MT76_TM_ATTR_TXBF_ACT, bf_act_map[action]);
		a = nla_nest_start(msg, MT76_TM_ATTR_TXBF_PARAM);
		if (!a)
			return -ENOMEM;

		for (i = 0; i < 5; i++)
			nla_put_u16(msg, i, val[i]);
		nla_nest_end(msg, a);
		break;
	case TXBF_ACT_IBF_PHASE_E2P_UPDATE:
		atenl_nl_ibf_e2p_update(an);
		atenl_get_ibf_cal_result(an);

		nla_put_u8(msg, MT76_TM_ATTR_AID, 0);
		nla_put_u8(msg, MT76_TM_ATTR_TXBF_ACT, MT76_TM_TXBF_ACT_INIT);

		a = nla_nest_start(msg, MT76_TM_ATTR_TXBF_PARAM);
		if (!a)
			return -ENOMEM;
		nla_put_u16(msg, 0, 0);
		nla_nest_end(msg, a);

		an->ibf_cal = false;
		break;
	case TXBF_ACT_INIT:
	case TXBF_ACT_POWER:
	default:
		break;
	}

	nla_nest_end(msg, ptr);

	*(u32 *)(hdr->data + 2) = data->ext_id;

	return unl_genl_request(&nl_priv->unl, msg, NULL, NULL);
}

static int
atenl_nl_ibf_get_status(struct atenl *an, struct atenl_data *data,
			struct atenl_nl_priv *nl_priv)
{
	struct atenl_cmd_hdr *hdr = atenl_hdr(data);
	u32 status = htonl(1);

	*(u32 *)(hdr->data + 2) = data->ext_id;
	memcpy(hdr->data + 6, &status, 4);

	return 0;
}

static int
atenl_nl_ibf_profile_update_all(struct atenl *an, struct atenl_data *data,
				struct atenl_nl_priv *nl_priv)
{
	struct atenl_cmd_hdr *hdr = atenl_hdr(data);
	struct nl_msg *msg;
	void *ptr, *a;
	u32 *v = (u32 *)(hdr->data + 4);
	u16 pfmu_idx = ntohl(v[0]);
	int i;

	for (i = 0, v = &v[5]; i < 64; i++, v += 5) {
		int j;

		if (unl_genl_init(&nl_priv->unl, "nl80211") < 0) {
			atenl_err("Failed to connect to nl80211\n");
			return 2;
		}

		msg = unl_genl_msg(&nl_priv->unl, NL80211_CMD_TESTMODE, false);
		nla_put_u32(msg, NL80211_ATTR_WIPHY,
			    get_band_val(an, an->cur_band, phy_idx));

		ptr = nla_nest_start(msg, NL80211_ATTR_TESTDATA);
		if (!ptr)
			return -ENOMEM;

		nla_put_u8(msg, MT76_TM_ATTR_TXBF_ACT, MT76_TM_TXBF_ACT_PROF_UPDATE_ALL);
		a = nla_nest_start(msg, MT76_TM_ATTR_TXBF_PARAM);
		if (!a)
			return -ENOMEM;
		nla_put_u16(msg, 0, pfmu_idx);

		for (j = 0; j < 5; j++)
			nla_put_u16(msg, j + 1, ntohl(v[j]));
		nla_nest_end(msg, a);

		nla_nest_end(msg, ptr);

		unl_genl_request(&nl_priv->unl, msg, NULL, NULL);

		unl_free(&nl_priv->unl);
	}

	*(u32 *)(hdr->data + 2) = data->ext_id;

	return 0;
}

#define NL_OPS_GROUP(cmd, ...)	[HQA_CMD_##cmd] = { __VA_ARGS__ }
static const struct atenl_nl_ops nl_ops[] = {
	NL_OPS_GROUP(SET_TX_PATH, .set=MT76_TM_ATTR_TX_ANTENNA),
	NL_OPS_GROUP(SET_TX_POWER, .set=MT76_TM_ATTR_TX_POWER),
	NL_OPS_GROUP(SET_RX_PATH, .set=MT76_TM_ATTR_TX_ANTENNA),
	NL_OPS_GROUP(SET_FREQ_OFFSET, .set=MT76_TM_ATTR_FREQ_OFFSET),
	NL_OPS_GROUP(SET_CFG, .ops=atenl_nl_set_cfg),
	NL_OPS_GROUP(SET_TSSI, .ops=atenl_nl_set_cfg),
	NL_OPS_GROUP(CONTINUOUS_TX, .ops=atenl_nl_continuous_tx),
	NL_OPS_GROUP(GET_TX_INFO, .dump=MT76_TM_STATS_ATTR_TX_DONE),
	NL_OPS_GROUP(GET_RX_INFO, .ops=atenl_nl_get_rx_info, .dump=true),
	NL_OPS_GROUP(SET_RU, .ops=atenl_nl_set_ru),
};
#undef NL_OPS_GROUP

#define NL_OPS_EXT(cmd, ...)	[HQA_EXT_CMD_##cmd] = { __VA_ARGS__ }
static const struct atenl_nl_ops nl_ops_ext[] = {
	NL_OPS_EXT(SET_TX, .ops=atenl_nl_set_tx),
	NL_OPS_EXT(START_TX, .ops=atenl_nl_tx),
	NL_OPS_EXT(STOP_TX, .ops=atenl_nl_tx),
	NL_OPS_EXT(START_RX, .ops=atenl_nl_rx),
	NL_OPS_EXT(STOP_RX, .ops=atenl_nl_rx),
	NL_OPS_EXT(OFF_CH_SCAN, .ops=atenl_off_ch_scan),
	NL_OPS_EXT(IBF_SET_VAL, .ops=atenl_nl_ibf_set_val),
	NL_OPS_EXT(IBF_GET_STATUS, .ops=atenl_nl_ibf_get_status),
	NL_OPS_EXT(IBF_PROF_UPDATE_ALL, .ops=atenl_nl_ibf_profile_update_all),
};
#undef NL_OPS_EXT

int atenl_nl_process(struct atenl *an, struct atenl_data *data)
{
	struct atenl_nl_priv nl_priv = {};
	const struct atenl_nl_ops *ops;
	struct nl_msg *msg;
	int ret = 0;

	if (data->ext_cmd != 0)
		ops = &nl_ops_ext[data->ext_cmd];
	else
		ops = &nl_ops[data->cmd];

	if (unl_genl_init(&nl_priv.unl, "nl80211") < 0) {
		atenl_err("Failed to connect to nl80211\n");
		return -1;
	}

	msg = unl_genl_msg(&nl_priv.unl, NL80211_CMD_TESTMODE, !!ops->dump);
	nla_put_u32(msg, NL80211_ATTR_WIPHY, get_band_val(an, an->cur_band, phy_idx));
	nl_priv.msg = msg;

	if (ops->ops) {
		ret = ops->ops(an, data, &nl_priv);
	} else if (ops->dump) {
		nl_priv.attr = ops->dump;
		ret = atenl_nl_dump_attr(an, data, &nl_priv);
	} else {
		nl_priv.attr = ops->set;
		ret = atenl_nl_set_attr(an, data, &nl_priv);
	}

	if (ret)
		atenl_err("command process error: 0x%x (0x%x)\n", data->cmd_id, data->ext_id);

	unl_free(&nl_priv.unl);

	return ret;
}

int atenl_nl_process_many(struct atenl *an, struct atenl_data *data)
{
	struct atenl_nl_priv nl_priv = {};
	const struct atenl_nl_ops *ops;
	int ret = 0;

	if (data->ext_cmd != 0)
		ops = &nl_ops_ext[data->ext_cmd];
	else
		ops = &nl_ops[data->cmd];

	if (ops->ops)
		ret = ops->ops(an, data, &nl_priv);

	return ret;
}

int atenl_nl_set_state(struct atenl *an, u8 band,
		       enum mt76_testmode_state state)
{
	struct atenl_nl_priv nl_priv = {};
	struct nl_msg *msg;
	void *ptr;

	if (unl_genl_init(&nl_priv.unl, "nl80211") < 0) {
		atenl_err("Failed to connect to nl80211\n");
		return 2;
	}

	msg = unl_genl_msg(&nl_priv.unl, NL80211_CMD_TESTMODE, false);
	nla_put_u32(msg, NL80211_ATTR_WIPHY, get_band_val(an, band, phy_idx));

	ptr = nla_nest_start(msg, NL80211_ATTR_TESTDATA);
	if (!ptr)
		return -ENOMEM;

	atenl_set_attr_state(an, msg, band, state);

	nla_nest_end(msg, ptr);

	unl_genl_request(&nl_priv.unl, msg, NULL, NULL);

	unl_free(&nl_priv.unl);

	return 0;
}

int atenl_nl_set_aid(struct atenl *an, u8 band, u8 aid)
{
	struct atenl_nl_priv nl_priv = {};
	struct nl_msg *msg;
	void *ptr;

	if (unl_genl_init(&nl_priv.unl, "nl80211") < 0) {
		atenl_err("Failed to connect to nl80211\n");
		return 2;
	}

	msg = unl_genl_msg(&nl_priv.unl, NL80211_CMD_TESTMODE, false);
	nla_put_u32(msg, NL80211_ATTR_WIPHY, get_band_val(an, band, phy_idx));

	ptr = nla_nest_start(msg, NL80211_ATTR_TESTDATA);
	if (!ptr)
		return -ENOMEM;

	nla_put_u8(msg, MT76_TM_ATTR_AID, aid);

	nla_nest_end(msg, ptr);

	unl_genl_request(&nl_priv.unl, msg, NULL, NULL);

	unl_free(&nl_priv.unl);

	return 0;
}

static int atenl_nl_check_mtd_cb(struct nl_msg *msg, void *arg)
{
	struct atenl_nl_priv *nl_priv = (struct atenl_nl_priv *)arg;
	struct atenl *an = nl_priv->an;
	struct nlattr *tb[NUM_MT76_TM_ATTRS];
	struct nlattr *attr;

	attr = unl_find_attr(&nl_priv->unl, msg, NL80211_ATTR_TESTDATA);
	if (!attr)
		return NL_SKIP;

	nla_parse_nested(tb, MT76_TM_ATTR_MAX, attr, testdata_policy);
	if (!tb[MT76_TM_ATTR_MTD_PART] || !tb[MT76_TM_ATTR_MTD_OFFSET])
		return NL_SKIP;

	an->mtd_part = strdup(nla_get_string(tb[MT76_TM_ATTR_MTD_PART]));
	an->mtd_offset = nla_get_u32(tb[MT76_TM_ATTR_MTD_OFFSET]);

	return NL_SKIP;
}

int atenl_nl_check_mtd(struct atenl *an)
{
	struct atenl_nl_priv nl_priv = { .an = an };
	struct nl_msg *msg;

	if (unl_genl_init(&nl_priv.unl, "nl80211") < 0) {
		atenl_err("Failed to connect to nl80211\n");
		return 2;
	}

	msg = unl_genl_msg(&nl_priv.unl, NL80211_CMD_TESTMODE, true);
	nla_put_u32(msg, NL80211_ATTR_WIPHY, get_band_val(an, 0, phy_idx));
	unl_genl_request(&nl_priv.unl, msg, atenl_nl_check_mtd_cb, (void *)&nl_priv);

	unl_free(&nl_priv.unl);

	return 0;
}

int atenl_nl_write_eeprom(struct atenl *an, u32 offset, u8 *val, int len)
{
	struct atenl_nl_priv nl_priv = {};
	struct nl_msg *msg;
	void *ptr, *a;
	int i;

	if (unl_genl_init(&nl_priv.unl, "nl80211") < 0) {
		atenl_err("Failed to connect to nl80211\n");
		return 2;
	}

	if (len > 16)
		return -EINVAL;

	msg = unl_genl_msg(&nl_priv.unl, NL80211_CMD_TESTMODE, false);
	nla_put_u32(msg, NL80211_ATTR_WIPHY, get_band_val(an, 0, phy_idx));

	ptr = nla_nest_start(msg, NL80211_ATTR_TESTDATA);
	if (!ptr)
		return -ENOMEM;

	nla_put_u8(msg, MT76_TM_ATTR_EEPROM_ACTION,
		   MT76_TM_EEPROM_ACTION_UPDATE_DATA);
	nla_put_u32(msg, MT76_TM_ATTR_EEPROM_OFFSET, offset);

	a = nla_nest_start(msg, MT76_TM_ATTR_EEPROM_VAL);
	if (!a)
		return -ENOMEM;

	for (i = 0; i < len; i++)
		if (nla_put_u8(msg, i, val[i]))
			goto out;

	nla_nest_end(msg, a);

	nla_nest_end(msg, ptr);

	unl_genl_request(&nl_priv.unl, msg, NULL, NULL);

	unl_free(&nl_priv.unl);

out:
	return 0;
}

int atenl_nl_write_efuse_all(struct atenl *an)
{
	struct atenl_nl_priv nl_priv = {};
	struct nl_msg *msg;
	void *ptr;

	if (unl_genl_init(&nl_priv.unl, "nl80211") < 0) {
		atenl_err("Failed to connect to nl80211\n");
		return 2;
	}

	msg = unl_genl_msg(&nl_priv.unl, NL80211_CMD_TESTMODE, false);
	nla_put_u32(msg, NL80211_ATTR_WIPHY, get_band_val(an, 0, phy_idx));

	ptr = nla_nest_start(msg, NL80211_ATTR_TESTDATA);
	if (!ptr)
		return -ENOMEM;

	nla_put_u8(msg, MT76_TM_ATTR_EEPROM_ACTION,
		   MT76_TM_EEPROM_ACTION_WRITE_TO_EFUSE);

	nla_nest_end(msg, ptr);

	unl_genl_request(&nl_priv.unl, msg, NULL, NULL);

	unl_free(&nl_priv.unl);

	return 0;
}

int atenl_nl_update_buffer_mode(struct atenl *an)
{
	struct atenl_nl_priv nl_priv = {};
	struct nl_msg *msg;
	void *ptr;

	if (unl_genl_init(&nl_priv.unl, "nl80211") < 0) {
		atenl_err("Failed to connect to nl80211\n");
		return 2;
	}

	msg = unl_genl_msg(&nl_priv.unl, NL80211_CMD_TESTMODE, false);
	nla_put_u32(msg, NL80211_ATTR_WIPHY, get_band_val(an, 0, phy_idx));

	ptr = nla_nest_start(msg, NL80211_ATTR_TESTDATA);
	if (!ptr)
		return -ENOMEM;

	nla_put_u8(msg, MT76_TM_ATTR_EEPROM_ACTION,
		   MT76_TM_EEPROM_ACTION_UPDATE_BUFFER_MODE);

	nla_nest_end(msg, ptr);

	unl_genl_request(&nl_priv.unl, msg, NULL, NULL);

	unl_free(&nl_priv.unl);

	return 0;
}

