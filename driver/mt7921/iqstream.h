/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * IQB2 block stream: I/Q blocks DMA'd by the MCU-side WFDMA TX0 ring.
 *
 * The firmware (or, for bring-up, the host through regidx/regval) points a
 * TX0 descriptor at a small header in MCU DLM (segment 0) and at the sample
 * memory itself (segment 1, e.g. the ICAP window 0xe0200000). The WFDMA
 * copies both to the host without a CPU copy. On the wire each block looks
 * like an ordinary MCU event, so it arrives on the main bulk-IN queue:
 *
 *   struct mt76_connac2_mcu_rxd   0x24 bytes, eid 0xed, ext_eid 0x50
 *   struct mt7921_iqb2_hdr        hdr_len bytes (32 in version 1)
 *   payload                       payload_len bytes
 *
 * mt7921_iqs_rx() recognizes the magic before any linearize or MCU event
 * processing, appends header + payload to a byte ring and wakes the reader of
 * /dev/mt7921_iq_<phy> (or debugfs spectrum/iq_stream). A full ring drops the
 * whole block and counts it.
 *
 * Opening /dev/mt7921_iq_<phy> starts the stream (monitor mode required),
 * closing it stops the stream; read() returns whole IQB2 records as above,
 * without the MCU RXD, and 0 once the stream has ended (chip reset,
 * disconnect, or a failed restart after a reconfiguration).
 */
#ifndef __MT7921_IQSTREAM_H
#define __MT7921_IQSTREAM_H

#include <linux/types.h>

#define MT7921_IQB2_MAGIC	0x32425149	/* "IQB2" little endian */
#define MT7921_IQB2_VERSION	1
#define MT7921_IQB2_EXT_EID	0x50		/* shares SPECTRUM_CTRL's id */

struct mt7921_iqb2_hdr {
	__le32 magic;
	__le16 version;
	__le16 hdr_len;		/* bytes, including this struct */
	__le32 payload_len;	/* bytes following the header */
	__le32 seq;		/* block sequence, +1 per block */
	__le32 cursor;		/* source position of the first payload byte */
	__le32 flags;		/* format / source id, producer-defined */
	__le32 generation;	/* start/stop cycle */
	__le32 tick;		/* MCU free-running counter at submission */
} __packed;

struct mt792x_dev;
struct sk_buff;
struct dentry;

bool mt7921_iqs_rx(struct mt792x_dev *dev, struct sk_buff *skb);
int mt7921_iqs_init(struct mt792x_dev *dev, struct dentry *dir);
void mt7921_iqs_mcu_guard(struct mt792x_dev *dev, int cmd);
void mt7921_iqs_chip_reset(struct mt792x_dev *dev);
void mt7921_iqs_shutdown(struct mt792x_dev *dev);

/* RF SPI bridge transaction (bus-specific; mt7921u registers it). port:
 * physical SPI port (1 = RX chain 0, 0 = companion bank, 5 = TOP); addr:
 * 12-bit analog register offset. Returns 0 or a negative errno.
 */
typedef int (*mt7921_rf_xfer_t)(struct mt792x_dev *dev, u32 port, u32 addr,
				u32 *val, bool read);
void mt7921_iqs_set_rf_xfer(mt7921_rf_xfer_t xfer);

#endif /* __MT7921_IQSTREAM_H */
