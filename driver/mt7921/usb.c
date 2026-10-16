// SPDX-License-Identifier: BSD-3-Clause-Clear
/* Copyright (C) 2022 MediaTek Inc.
 *
 * Author: Lorenzo Bianconi <lorenzo@kernel.org>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/usb.h>

#include "mt7921.h"
#include "iqstream.h"
#include "mcu.h"
#include "../mt76_connac2_mac.h"

static const struct usb_device_id mt7921u_device_table[] = {
	{ USB_DEVICE_AND_INTERFACE_INFO(0x0e8d, 0x7961, 0xff, 0xff, 0xff),
		.driver_info = (kernel_ulong_t)MT7921_FIRMWARE_WM },
	/* Comfast CF-952AX */
	{ USB_DEVICE_AND_INTERFACE_INFO(0x3574, 0x6211, 0xff, 0xff, 0xff),
		.driver_info = (kernel_ulong_t)MT7921_FIRMWARE_WM },
	/* Netgear, Inc. [A8000,AXE3000] */
	{ USB_DEVICE_AND_INTERFACE_INFO(0x0846, 0x9060, 0xff, 0xff, 0xff),
		.driver_info = (kernel_ulong_t)MT7921_FIRMWARE_WM },
	/* Netgear, Inc. A7500 */
	{ USB_DEVICE_AND_INTERFACE_INFO(0x0846, 0x9065, 0xff, 0xff, 0xff),
		.driver_info = (kernel_ulong_t)MT7921_FIRMWARE_WM },
	/* TP-Link TXE50UH */
	{ USB_DEVICE_AND_INTERFACE_INFO(0x35bc, 0x0107, 0xff, 0xff, 0xff),
		.driver_info = (kernel_ulong_t)MT7921_FIRMWARE_WM },
	{ },
};

static int
mt7921u_mcu_send_message(struct mt76_dev *mdev, struct sk_buff *skb,
			 int cmd, int *seq)
{
	struct mt792x_dev *dev = container_of(mdev, struct mt792x_dev, mt76);
	u32 pad, ep;
	int ret;

	ret = mt76_connac2_mcu_fill_message(mdev, skb, cmd, seq);
	if (ret)
		return ret;

	/* A running I/Q stream owns the MCU; stop it first. */
	mt7921_iqs_mcu_guard(dev, cmd);

	mdev->mcu.timeout = 3 * HZ;

	if (cmd != MCU_CMD(FW_SCATTER))
		ep = MT_EP_OUT_INBAND_CMD;
	else
		ep = MT_EP_OUT_AC_BE;

	mt792x_skb_add_usb_sdio_hdr(dev, skb, 0);
	pad = round_up(skb->len, 4) + 4 - skb->len;
	__skb_put_zero(skb, pad);

	ret = mt76u_bulk_msg(&dev->mt76, skb->data, skb->len, NULL,
			     1000, ep);
	dev_kfree_skb(skb);

	return ret;
}

static int mt7921u_mcu_init(struct mt792x_dev *dev)
{
	static const struct mt76_mcu_ops mcu_ops = {
		.headroom = MT_SDIO_HDR_SIZE +
			    sizeof(struct mt76_connac2_mcu_txd),
		.tailroom = MT_USB_TAIL_SIZE,
		.mcu_skb_send_msg = mt7921u_mcu_send_message,
		.mcu_parse_response = mt7921_mcu_parse_response,
	};
	int ret;

	dev->mt76.mcu_ops = &mcu_ops;

	mt76_set(dev, MT_UDMA_TX_QSEL, MT_FW_DL_EN);
	ret = mt7921_run_firmware(dev);
	if (ret)
		return ret;

	set_bit(MT76_STATE_MCU_RUNNING, &dev->mphy.state);
	mt76_clear(dev, MT_UDMA_TX_QSEL, MT_FW_DL_EN);

	return 0;
}

static int mt7921u_mac_reset(struct mt792x_dev *dev)
{
	int err;

	mt7921_iqs_chip_reset(dev);
	mt792xu_reset_on_bus_error(dev);
	if (atomic_read(&dev->mt76.bus_hung))
		return 0;

	mt76_txq_schedule_all(&dev->mphy);
	mt76_worker_disable(&dev->mt76.tx_worker);

	set_bit(MT76_RESET, &dev->mphy.state);
	set_bit(MT76_MCU_RESET, &dev->mphy.state);

	wake_up(&dev->mt76.mcu.wait);
	skb_queue_purge(&dev->mt76.mcu.res_q);

	mt76u_stop_rx(&dev->mt76);
	mt76u_stop_tx(&dev->mt76);

	mt792xu_wfsys_reset(dev);

	clear_bit(MT76_MCU_RESET, &dev->mphy.state);
	err = mt76u_resume_rx(&dev->mt76);
	if (err)
		goto out;

	err = mt792xu_mcu_power_on(dev);
	if (err)
		goto out;

	err = mt792xu_dma_init(dev, false);
	if (err)
		goto out;

	mt76_wr(dev, MT_SWDEF_MODE, MT_SWDEF_NORMAL_MODE);
	mt76_set(dev, MT_UDMA_TX_QSEL, MT_FW_DL_EN);

	err = mt7921_run_firmware(dev);
	if (err)
		goto out;

	mt76_clear(dev, MT_UDMA_TX_QSEL, MT_FW_DL_EN);

	err = mt7921_mcu_set_eeprom(dev);
	if (err)
		goto out;

	err = mt7921_mac_init(dev);
	if (err)
		goto out;

	err = __mt7921_start(&dev->phy);
out:
	clear_bit(MT76_RESET, &dev->mphy.state);

	mt76_worker_enable(&dev->mt76.tx_worker);

	return err;
}

/*
 * Analog (RF) registers sit behind the RF SPI bridge at 0x7c004000, reached
 * with plain vendor requests (USB_TYPE_VENDOR, 0x63/0x66), not the
 * mt792xu_rr/wr request type. Wi-Fi ports 0/1 use the command/data/result
 * triplet +0x10/+0x14/+0x18 with busy bit 1, TOP (port 5) +0x50.. with busy
 * bit 5. Writing DATA launches the transfer. Sequence from BirriaSDR
 * (src/chips/mt7921_rf.c). The vendor firmware uses the same bridge; the
 * busy poll serialises single transactions, not read-modify-write pairs.
 */
#define MT7921U_RF_BRIDGE	0x7c004000
#define MT7921U_RF_POLLS	1000

static u32 mt7921u_br_rr(struct mt792x_dev *dev, u32 addr)
{
	return ___mt76u_rr(&dev->mt76, MT_VEND_READ_EXT,
			   USB_DIR_IN | USB_TYPE_VENDOR, addr);
}

static void mt7921u_br_wr(struct mt792x_dev *dev, u32 addr, u32 val)
{
	___mt76u_wr(&dev->mt76, MT_VEND_WRITE_EXT,
		    USB_DIR_OUT | USB_TYPE_VENDOR, addr, val);
}

static int mt7921u_br_idle(struct mt792x_dev *dev, u32 busy)
{
	int i;

	for (i = 0; i < MT7921U_RF_POLLS; i++)
		if (!(mt7921u_br_rr(dev, MT7921U_RF_BRIDGE) & busy))
			return 0;
	return -ETIMEDOUT;
}

static int mt7921u_rf_xfer(struct mt792x_dev *dev, u32 port, u32 addr,
			   u32 *val, bool read)
{
	u32 cmd, busy, ctl;
	int ret;

	if ((port != 0 && port != 1 && port != 5) || addr > 0xfff || (addr & 3))
		return -EINVAL;
	cmd = MT7921U_RF_BRIDGE + (port == 5 ? 0x50 : 0x10);
	busy = BIT(port == 5 ? 5 : 1);

	mutex_lock(&dev->mt76.usb.usb_ctrl_mtx);
	ret = mt7921u_br_idle(dev, busy);
	if (ret)
		goto out;
	ctl = mt7921u_br_rr(dev, cmd);
	ctl = (ctl & 0xffff0000) | port << 13 | addr | (read ? 0x1000 : 0);
	mt7921u_br_wr(dev, cmd, ctl);
	mt7921u_br_wr(dev, cmd + 4, read ? 0 : *val);
	ret = mt7921u_br_idle(dev, busy);
	if (!ret && read)
		*val = mt7921u_br_rr(dev, cmd + 8);
out:
	mutex_unlock(&dev->mt76.usb.usb_ctrl_mtx);
	return ret;
}

static int mt7921u_probe(struct usb_interface *usb_intf,
			 const struct usb_device_id *id)
{
	static const struct mt76_driver_ops drv_ops = {
		.txwi_size = MT_SDIO_TXD_SIZE,
		.drv_flags = MT_DRV_RX_DMA_HDR | MT_DRV_HW_MGMT_TXQ |
			     MT_DRV_AMSDU_OFFLOAD,
		.survey_flags = SURVEY_INFO_TIME_TX |
				SURVEY_INFO_TIME_RX |
				SURVEY_INFO_TIME_BSS_RX,
		.tx_prepare_skb = mt7921_usb_sdio_tx_prepare_skb,
		.tx_complete_skb = mt7921_usb_sdio_tx_complete_skb,
		.tx_status_data = mt7921_usb_sdio_tx_status_data,
		.rx_skb = mt7921_queue_rx_skb,
		.rx_check = mt7921_rx_check,
		.sta_add = mt7921_mac_sta_add,
		.sta_event = mt7921_mac_sta_event,
		.sta_remove = mt7921_mac_sta_remove,
		.update_survey = mt792x_update_channel,
		.set_channel = mt7921_set_channel,
	};
	static const struct mt792x_hif_ops hif_ops = {
		.mcu_init = mt7921u_mcu_init,
		.init_reset = mt792xu_init_reset,
		.reset = mt7921u_mac_reset,
	};
	static struct mt76_bus_ops bus_ops = {
		.rr = mt792xu_rr,
		.wr = mt792xu_wr,
		.rmw = mt792xu_rmw,
		.read_copy = mt76u_read_copy,
		.write_copy = mt792xu_copy,
		.type = MT76_BUS_USB,
	};
	struct usb_device *udev = interface_to_usbdev(usb_intf);
	struct ieee80211_ops *ops;
	struct ieee80211_hw *hw;
	struct mt792x_dev *dev;
	struct mt76_dev *mdev;
	u8 features;
	int ret;

	ops = mt792x_get_mac80211_ops(&usb_intf->dev, &mt7921_ops,
				      (void *)id->driver_info, &features);
	if (!ops)
		return -ENOMEM;

	ops->stop = mt792xu_stop;
	mdev = mt76_alloc_device(&usb_intf->dev, sizeof(*dev), ops, &drv_ops);
	if (!mdev)
		return -ENOMEM;

	dev = container_of(mdev, struct mt792x_dev, mt76);
	dev->fw_features = features;
	dev->hif_ops = &hif_ops;
	atomic_set(&dev->mt76.bus_hung, false);
	mt792xu_reset_work_init(dev);

	usb_reset_device(udev);

	usb_set_intfdata(usb_intf, dev);

	ret = __mt76u_init(mdev, usb_intf, &bus_ops);
	if (ret < 0)
		goto error;

	mdev->rev = (mt76_rr(dev, MT_HW_CHIPID) << 16) |
		    (mt76_rr(dev, MT_HW_REV) & 0xff);
	dev_dbg(mdev->dev, "ASIC revision: %04x\n", mdev->rev);

	if (mt76_get_field(dev, MT_CONN_ON_MISC, MT_TOP_MISC2_FW_N9_RDY)) {
		ret = mt792xu_wfsys_reset(dev);
		if (ret)
			goto error;
	}

	ret = mt792xu_mcu_power_on(dev);
	if (ret)
		goto error;

	ret = mt76u_alloc_mcu_queue(&dev->mt76);
	if (ret)
		goto error;

	ret = mt76u_alloc_queues(&dev->mt76);
	if (ret)
		goto error;

	ret = mt792xu_dma_init(dev, false);
	if (ret)
		goto error;

	hw = mt76_hw(dev);
	/* check hw sg support in order to enable AMSDU */
	hw->max_tx_fragments = mdev->usb.sg_en ? MT_HW_TXP_MAX_BUF_NUM : 1;

	mt7921_iqs_set_rf_xfer(mt7921u_rf_xfer);
	ret = mt7921_register_device(dev);
	if (ret)
		goto error;

	return 0;

error:
	mt76u_queues_deinit(&dev->mt76);
	mt792xu_reset_work_cleanup(dev);

	usb_set_intfdata(usb_intf, NULL);

	mt76_free_device(&dev->mt76);

	return ret;
}

#ifdef CONFIG_PM
static int mt7921u_suspend(struct usb_interface *intf, pm_message_t state)
{
	struct mt792x_dev *dev = usb_get_intfdata(intf);
	struct mt76_connac_pm *pm = &dev->pm;
	int err;

	pm->suspended = true;
	flush_work(&dev->reset_work);

	err = mt76_connac_mcu_set_hif_suspend(&dev->mt76, true, true);
	if (err)
		goto failed;

	mt76u_stop_rx(&dev->mt76);
	mt76u_stop_tx(&dev->mt76);

	return 0;

failed:
	pm->suspended = false;

	if (err < 0)
		mt792x_reset(&dev->mt76);

	return err;
}

static int mt7921u_resume(struct usb_interface *intf)
{
	struct mt792x_dev *dev = usb_get_intfdata(intf);
	struct mt76_connac_pm *pm = &dev->pm;
	bool reinit = true;
	int err, i;

	for (i = 0; i < 10; i++) {
		u32 val = mt76_rr(dev, MT_WF_SW_DEF_CR_USB_MCU_EVENT);

		if (!(val & MT_WF_SW_SER_TRIGGER_SUSPEND)) {
			reinit = false;
			break;
		}
		if (val & MT_WF_SW_SER_DONE_SUSPEND) {
			mt76_wr(dev, MT_WF_SW_DEF_CR_USB_MCU_EVENT, 0);
			break;
		}

		msleep(20);
	}

	if (reinit || mt792x_dma_need_reinit(dev)) {
		err = mt792xu_dma_init(dev, true);
		if (err)
			goto failed;
	}

	err = mt76u_resume_rx(&dev->mt76);
	if (err < 0)
		goto failed;

	err = mt76_connac_mcu_set_hif_suspend(&dev->mt76, false, true);
failed:
	pm->suspended = false;

	if (err < 0)
		mt792x_reset(&dev->mt76);

	return err;
}
#endif /* CONFIG_PM */

MODULE_DEVICE_TABLE(usb, mt7921u_device_table);
MODULE_FIRMWARE(MT7921_FIRMWARE_WM);
MODULE_FIRMWARE(MT7921_ROM_PATCH);

static void mt7921u_disconnect(struct usb_interface *usb_intf)
{
	struct mt792x_dev *dev = usb_get_intfdata(usb_intf);

	/* Stop an I/Q stream while the chip still answers. */
	if (dev)
		mt7921_iqs_shutdown(dev);
	mt792xu_disconnect(usb_intf);
}

static struct usb_driver mt7921u_driver = {
	.name		= KBUILD_MODNAME,
	.id_table	= mt7921u_device_table,
	.probe		= mt7921u_probe,
	.disconnect	= mt7921u_disconnect,
#ifdef CONFIG_PM
	.suspend	= mt7921u_suspend,
	.resume		= mt7921u_resume,
	.reset_resume	= mt7921u_resume,
#endif /* CONFIG_PM */
	.soft_unbind	= 1,
	.disable_hub_initiated_lpm = 1,
};
module_usb_driver(mt7921u_driver);

MODULE_DESCRIPTION("MediaTek MT7921U (USB) wireless driver");
MODULE_AUTHOR("Lorenzo Bianconi <lorenzo@kernel.org>");
MODULE_LICENSE("Dual BSD/GPL");
