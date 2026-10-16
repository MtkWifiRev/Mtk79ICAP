/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * mt7921 ICAP (RBIST internal capture) support.
 *
 * Everything here was recovered from the MT7961 firmware (build 20260224) and
 * cross-checked against the working mt7915 implementation. Firmware
 * addresses below refer to WIFI_RAM_CODE_MT7961_1 build 20260224.
 *
 * Command path:
 *   MCU_EXT_CMD(RF_TEST) 0x04 -> rlmTestExtCmdFsmEventTestCtrl @ 0x00933d86
 *     -> icap_rf_test_dispatch @ 0x00933dea
 *          action byte at +0x40 must be 1, subcommand at +0x44,
 *          payload at +0x48
 *
 * Data path:
 *   subcmd 0x11 -> rlmTestGetIcapData @ 0x00964d7c
 *     -> icapSpectrumDumpDataEvent @ 0x0095c7a0 (mode 0)
 *          -> ext event 0x04, func_index 0x11
 */

#ifndef __MT7921_SPECTRUM_H
#define __MT7921_SPECTRUM_H

/*
 * Not present in upstream mt76_connac_mcu.h, which only carries
 * MCU_EXT_CMD_RF_TEST. Defined here so the shared header stays untouched.
 */
#define MT7921_EXT_EVENT_RF_TEST	0x04
#define MT7921_EXT_EVENT_SPECTRUM_CTRL	0x50	/* wifiSpectrumExtCmdHandler */
#define MT7921_EXT_CMD_SPECTRUM_CTRL	0x56	/* wifiSpectrumExtCmdHandler @ 0x0095c8a0 */

/*
 * MCU_EXT_CMD(SPECTRUM_CTRL) built by hand: upstream mt76_connac_mcu.h has no
 * MCU_EXT_CMD_SPECTRUM_CTRL enumerator, and this composes the identical value
 * from the upstream macros rather than adding one there.
 */
#define MT7921_MCU_CMD_SPECTRUM_CTRL					\
	(MCU_CMD(EXT_CID) |						\
	 FIELD_PREP(__MCU_CMD_FIELD_EXT_ID, MT7921_EXT_CMD_SPECTRUM_CTRL))

/* Subcommands, at +0x44. Confirmed at icap_rf_test_dispatch @ 0x00933dea. */
enum mt7921_icap_subcmd {
	MT7921_ICAP_SET_PARAMETER	= 0x0b,	/* -> rlmTestSetIcapParam */
	MT7921_ICAP_GET_CAPTURE_STATUS	= 0x0c,	/* -> rlmTestSetIcapQueryStatusInfo */
	MT7921_ICAP_DUMP_DATA		= 0x11,	/* -> rlmTestGetIcapData */
};

/*
 * RBIST_CAP_START_T, the SET_PARAMETER payload at +0x48.
 *
 * Layout confirmed identical to mt7915 by reading icapSpectrumSetParam
 * @ 0x0095c678 and icapSpectrumSetParam_hal @ 0x0095c630: 11 of these fields
 * are indexed at exactly these offsets and none contradicts.
 *
 * The fields marked DEAD are never read on mt7961. The payload reaches
 * rlmTestSetIcapParam -> icapSpectrumSetParam -> icapSpectrumSetParam_hal,
 * plus icapSpectrumSetParam_rbist @ 0x0095c58a (missed by an earlier pass
 * because Ghidra truncated that function).
 * The DEAD fields are not copied into the global ICAP context at
 * DAT_ram_0202e3d0 either. They are kept so the struct keeps its wire size
 * and stays diffable against mt7915.
 *
 * Field names and semantics are from MediaTek's own
 * mt_wifi/embedded/include/icap.h (_RBIST_CAP_START_T), not guessed.
 *
 * Two fields long thought DEAD are in fact read by
 * icapSpectrumSetParam_rbist (§4.26):
 *   cap_source     - read, masked to a byte, passed as ucICapSrcSel.
 *   cap_stop_cycle - read, halved for sel 0x11/0x14/0x21, programmed into
 *                    0x800210a4 before the trigger fires. On mt7915,
 *                    cap_stop_cycle == 0 caused the never-completing capture
 *                    and the stale-read loop; 0 has been this driver's
 *                    default too, and arming with a non-zero value in
 *                    sequence has never been tested.
 */
struct mt7921_rbist_cap_start {
	__le32 trigger;			/*  0 0x00 fgTrigger, 0 => stop */
	__le32 ring_cap_en;		/*  1 0x04 fgRingCapEn */
	__le32 trigger_event;		/*  2 0x08 u4TriggerEvent */
	__le32 capture_node;		/*  3 0x0c u4CaptureNode */
	__le32 capture_len;		/*  4 0x10 u4CaptureLen */
	__le32 cap_stop_cycle;		/*  5 0x14 READ: -> 0x800210a4, see above */
	__le32 pd_enable;		/*  6 0x18 u4PdEnable, DEAD */
	__le32 fix_rx_gain;		/*  7 0x1c u4FixRxGain, DEAD (0 = AGC) */
	__le32 wifi_path;		/*  8 0x20 u4WifiPath, DEAD (0 = WF0) */
	__le32 band_idx;		/*  9 0x24 u4BandIdx */
	__le32 bw;			/* 10 0x28 u4BW */
	__le32 bit_width;		/* 11 0x2c u4EnBitWidth - READ, see below */
	__le32 arch;			/* 12 0x30 u4Architech, read as byte */
	__le32 phy_idx;			/* 13 0x34 u4PhyIdx */
	__le32 emi_start_addr;		/* 14 0x38 u4EmiStartAddress */
	__le32 emi_end_addr;		/* 15 0x3c u4EmiEndAddress */
	__le32 emi_msb_addr;		/* 16 0x40 DEAD */
	__le32 cap_source;		/* 17 0x44 READ: ucICapSrcSel, see above */
	__le32 rsv[2];			/* 18,19 */
} __packed;

/*
 * RBIST_DUMP_RAW_DATA_T, the DUMP_DATA payload at +0x48.
 *
 * Mirrors mt7915. Note that icapSpectrumDumpDataEvent sources its capture
 * geometry from the global context stored by SET_PARAMETER (band_idx +0x20,
 * phy_idx +0x24, capture_node +0x28, capture_len +0x2c) rather than from this
 * request, so wf_num/iq_type may not take effect on mt7961. Unconfirmed on
 * hardware; sent anyway because it is harmless if ignored.
 */
struct mt7921_rbist_dump_req {
	__le32 address;
	__le32 addr_offset;
	__le32 bank;
	__le32 bank_size;
	__le32 wf_num;
	__le32 iq_type;
	__le32 rsv[6];
} __packed;

/*
 * The request as it appears on the wire. mt76_connac2_mcu_txd is exactly 64
 * bytes, so `action` lands at +0x40 and `param` at +0x48 as the firmware
 * expects - the same accident mt7915 relies on.
 */
struct mt7921_icap_ctrl {
	u8 action;		/* must be 1; checked at 0x00933dee */
	u8 icap_len;
	u8 rsv[2];
	__le32 func_idx;	/* enum mt7921_icap_subcmd */
	union {
		struct mt7921_rbist_cap_start start;
		struct mt7921_rbist_dump_req dump;
		__le32 raw[20];
	} param;
} __packed;

/*
 * SPECTRUM_CTRL (EXT cmd 0x56) request - the second, ungated entry point.
 *
 * wifiSpectrumExtCmdHandler @ 0x0095c8a0 has NO state gate and NO FWOpMode gate,
 * unlike the RF_TEST path, which dies at 0x00933dd8 unless FWOpMode == ICAP.
 * That matters for more than convenience: entering ICAP opmode takes the radio
 * offline, and the capture engine has never started in that state (the hardware
 * write pointer at 0x800210b4 stays zero for every capture_node - README §1.2).
 * This path can arm a capture with the radio still running in NORMAL mode, which
 * is the property that made ICAP_OVERLAP the only working mode on mt7915.
 *
 * The handler reads a control word at +0x40 and dispatches:
 *
 *   0 -> icapSpectrumReset_maybe(); icapSpectrumSetParam(cmd + 0x44, 1)
 *   1 -> FUN_ram_0095c6e6(cmd, 1)                    (status)
 *   2 -> icapSpectrumDumpDataEvent(cmd, cmd + 0x94, 1)
 *
 * Note the payload offsets differ from RF_TEST: RBIST_CAP_START_T sits at +0x44
 * rather than +0x48, and the dump request at +0x94. Those two are contiguous -
 * sizeof(RBIST_CAP_START_T) is 0x50, and 0x44 + 0x50 == 0x94 - so one request
 * carries both structures rather than a union of them.
 *
 * The trailing argument is 1 in every call, versus 0 on the RF_TEST path. That
 * is the transport selector in icapSpectrumDumpDataEvent: mode 1 emits ext event
 * 0x50 with func_index 0x02, not 0x04 / 0x11 (§4.4).
 */
enum mt7921_spectrum_ctrl_op {
	MT7921_SPECTRUM_CTRL_SET_PARAM	= 0,
	MT7921_SPECTRUM_CTRL_STATUS	= 1,
	MT7921_SPECTRUM_CTRL_DUMP	= 2,
};

/* func_index carried by mode-1 events; the RF_TEST path uses the subcmd values
 * from enum mt7921_icap_subcmd instead.
 */
#define MT7921_SPECTRUM_FUNC_DUMP	0x02

struct mt7921_spectrum_ctrl_req {
	__le32 control;				/* +0x40 */
	struct mt7921_rbist_cap_start start;	/* +0x44, 0x50 bytes */
	struct mt7921_rbist_dump_req dump;	/* +0x94 */
} __packed;

/* The offsets the firmware indexes, relative to the 64-byte mt76_connac2_mcu_txd
 * that precedes this struct on the wire: control at +0x40, RBIST_CAP_START_T at
 * +0x44, RBIST_DUMP_RAW_DATA_T at +0x94. If any of these trip, the firmware is
 * reading the wrong words and the failure is silent.
 */
static_assert(sizeof(struct mt7921_rbist_cap_start) == 0x50);
static_assert(offsetof(struct mt7921_spectrum_ctrl_req, control) == 0x00);
static_assert(offsetof(struct mt7921_spectrum_ctrl_req, start) == 0x04);
static_assert(offsetof(struct mt7921_spectrum_ctrl_req, dump) == 0x54);
/* RF_TEST path, for the same reason: action +0x40, subcmd +0x44, payload +0x48 */
static_assert(offsetof(struct mt7921_icap_ctrl, func_idx) == 0x04);
static_assert(offsetof(struct mt7921_icap_ctrl, param) == 0x08);

/* Which command path start/stop/status/dump use; the `path` debugfs knob. */
enum mt7921_spectrum_path {
	MT7921_SPECTRUM_PATH_RF_TEST	= 0,
	MT7921_SPECTRUM_PATH_CTRL	= 1,
};

/* Event payloads, immediately after struct mt76_connac2_mcu_rxd. */
struct mt7921_icap_evt_status {
	__le32 func_index;
	__le32 cap_done;
	__le32 rsv[15];
} __packed;

/*
 * Offsets confirmed from the pointers icapSpectrumDumpDataEvent hands to
 * halRbistQueryIQData @ 0x0096b9d4: bank +0x2c, data_len +0x30, wf_cnt +0x34,
 * smpl_cnt +0x38 and data +0x54, relative to a func_index stored at +0x24.
 */
struct mt7921_icap_evt_dump {
	__le32 func_index;
	__le32 pkt_num;
	__le32 bank;
	__le32 data_len;
	__le32 wf_cnt;
	__le32 smpl_cnt;
	__le32 rsv[6];
	__le32 data[];
} __packed;

#define MT7921_SPECTRUM_DATA_MAX	512	/* v58: 128 records, 0x860-byte event */
#define MT7921_SPECTRUM_PKT_HISTORY	32
/* Identity-compared, not strcmp'd, to tell a clean end-of-capture from a
 * runaway stream; see mt7921_spectrum_dump().
 */
#define MT7921_SPECTRUM_END_MARKER	"end marker (data_len=0)"

#define MT7921_SPECTRUM_DUMP_TIMEOUT	(HZ / 2)
/* Settle time between the two TM_SWITCH_MODE steps; see the "enter" handler. */
#define MT7921_SPECTRUM_MODE_SETTLE_MS	200

/*
 * Capture node encoding, read out of the mt7961 decoder FUN_ram_0096c61c
 * (reached from icapSpectrumSetParam_hal), which is the authority for THIS chip:
 *
 *   id    = node & 0xff          bits [7:0]   -> hw register bits [15:8]
 *   sel   = (node >> 16) & 0xff  bits [23:16]
 *   group = (node >> 24) & 0xf   bits [27:24]
 *   type  = node >> 28           bits [31:28]
 *
 * Bits [15:8] of the node value are never read.
 *
 * The type nibble matches the vendor's WF_COMM_CR_CAP_NODE_TYPE (icap.h):
 * 0 RX, 1 TX, 2 DFS, 3 THERMAL, 4 INPHYSNIFFER, 5 SPECTRUM. Only 0, 1, 2 and 4
 * are implemented here; every other type returns 1 without touching hardware.
 *
 * Per-type constraints from the dispatch:
 *
 *   type 0  group 0..3   body sits in an ex9 region Ghidra cannot decompile
 *   type 1  group 4..6 or 7..8,  sel 0x13 / 0x14 recognised
 *   type 2  group 0..1,          sel 0x20 / 0x21 recognised
 *   type 4  forces 0x8c0000 and sets _DAT_ram_0000d440 |= 0x13 - this is
 *           mt7915's 0x40000000, and it is what wedges mt7961.
 *
 * NOTE: the (subtype << 12) | node_id scheme in MediaTek's icap.h - WF0 = 0x0b,
 * FIIQ = 0x200b and so on - is connac1 (MT7615/7622/7663). It does NOT apply
 * here: those values differ only in bits [15:12], which this decoder ignores,
 * so 0x000b, 0x200b and 0x300b are all the same node to mt7961.
 */
#define MT7921_CAP_NODE_TYPE_RX		0
#define MT7921_CAP_NODE_TYPE_TX		1
#define MT7921_CAP_NODE_TYPE_DFS	2
#define MT7921_CAP_NODE_TYPE_SNIFFER	4	/* wedges the chip */

#define MT7921_CAP_NODE(type, group, sel, id) \
	((u32)(type) << 28 | (u32)(group) << 24 | (u32)(sel) << 16 | (u32)(id))

/*
 * Defaults. capture_len is 1024 samples rather than 0x4000 so a stuck dump
 * produces four events' worth of work instead of sixteen thousand.
 *
 * capture_node stays at 8 (type 0, group 0, id 8) because it is the only value
 * proven not to wedge the chip. It produces all-zero captures; the id and group
 * that actually select a live tap are still unknown. Sweep with icap-capture.sh.
 */
#define MT7921_SPECTRUM_DEF_CAPTURE_NODE	8
#define MT7921_SPECTRUM_DEF_CAPTURE_LEN		1024
#define MT7921_SPECTRUM_DEF_TRIGGER_EVENT	1	/* 0 = MAC trig, 1 = free run */
/*
 * u4EnBitWidth is an ENUM, not a width: 0 = 32bit, 1 = 96bit, 2 = 128bit
 * (MediaTek icap.h). The CAP_32BITS/CAP_96BITS defines (32/96) in that header
 * belong to something else - do not put them here.
 *
 * It must be 0. mt7915's icapSpectrumSetParam only calls
 * whICapSpectrumEnRbist96b128b() when the field is non-zero, and mt7961 has the
 * identical guard in FUN_ram_0095c58a (`if (*(payload + 0x2c) != 0)`), so any
 * non-zero value enables wide-sample mode with a bogus width.
 *
 * This field is therefore NOT dead, contrary to README §4.3 - that analysis
 * followed only rlmTestSetIcapParam -> icapSpectrumSetParam -> _hal and missed
 * the FUN_ram_0095c58a call.
 */
#define MT7921_SPECTRUM_DEF_BIT_WIDTH		0

struct mt7921_spectrum_pkt {
	bool valid;
	u32 func_index;
	u32 pkt_num;
	u32 bank;
	u32 data_len;
	u32 wf_cnt;
	u32 smpl_cnt;
	__le32 data[MT7921_SPECTRUM_DATA_MAX];
};

struct mt7921_spectrum {
	spinlock_t lock;	/* protects the packet ring and event state */
	wait_queue_head_t wait;

	/* capture parameters, exported as debugfs u32 knobs */
	u32 trigger_event;
	u32 ring_cap_en;
	u32 capture_node;
	u32 capture_len;
	u32 band_idx;
	u32 bw;
	u32 arch;
	u32 phy_idx;
	u32 emi_start_addr;
	u32 emi_end_addr;
	/* ignored by mt7961 firmware; kept for parity with the vendor struct */
	u32 cap_stop_cycle;
	u32 cap_src;
	u32 bit_width;
	u32 pd_enable;
	u32 fix_rx_gain;
	u32 wifi_path;
	/* dump request */
	u32 wf_num;
	u32 iq_type;
	u32 bank;
	u32 bank_size;

	/* runtime state */
	bool running;
	bool truncated;
	u32 event_count;
	u32 pkt_count;
	/* Monotonic until driver reload; draining never resets these. */
	u32 drain_dropped;
	u32 drain_end_markers;
	bool drain_active;
	/* why the last dump stream stopped; static string, NULL if never */
	const char *end_reason;
	/* Last mode we *asked* for, not what the firmware did - TM_SWITCH_MODE
	 * goes out with wait=false and the firmware can silently decline. Only
	 * good enough to catch a forgotten mode entry, which is the common
	 * mistake after a module reload resets this to NORMAL.
	 */
	u32 fw_mode;
	/* enum mt7921_spectrum_path */
	u32 path;

	struct {
		bool valid;
		u32 func_index;
		u32 cap_done;
	} status;

	struct {
		bool valid;
		u32 skb_len;
		u32 payload_len;
		u16 len;
		u16 pkt_type_id;
		u8 eid;
		u8 ext_eid;
		u8 seq;
		u8 option;
		u32 words[8];
	} last_event;

	struct mt7921_spectrum_pkt pkt[MT7921_SPECTRUM_PKT_HISTORY];

	/* IQB2 DMA block stream, see iqstream.h; NULL when disabled */
	struct mt7921_iqs *iqs;
};

int mt7921_spectrum_init(struct mt792x_dev *dev, struct dentry *dir);
int mt7921_spectrum_iq_arm(struct mt792x_dev *dev);
int mt7921_spectrum_iq_dump_begin(struct mt792x_dev *dev);
int mt7921_spectrum_iq_disarm(struct mt792x_dev *dev);
void mt7921_mcu_rx_spectrum_event(struct mt792x_dev *dev, struct sk_buff *skb);

#endif /* __MT7921_SPECTRUM_H */
