// SPDX-License-Identifier: GPL-2.0
/*
 * Apple T6041 (M4 generation) Type-C PHY driver, USB2/HS only.
 *
 * The M4 ATC PHY ("atc-phy,t6040") has a different core/lane register
 * layout than the t8103..t8112 PHY driven by atc.c, but its USB2 PHY and
 * PIPE handler blocks are identical. This driver implements just enough
 * of the atc.c contract (usb2/usb3 phys, the dwc3 reset controller and
 * the typec orientation/mode switches) for USB2 host/device operation
 * with dwc3-apple, with the PIPE handler permanently muxed to the dummy
 * USB3 PHY. SuperSpeed needs the full t6040 core/lane port of atc.c.
 *
 * USB2 PHY and PIPE handler sequences are taken from atc.c (hardware-
 * verified on T6041 via the m1n1 bring-up: same offsets and semantics).
 *
 * Copyright (C) The Asahi Linux Contributors
 */

#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <dt-bindings/phy/phy.h>
#include <linux/reset-controller.h>
#include <linux/usb/typec_mux.h>
#include <linux/usb/typec_altmode.h>
#include <linux/usb/typec_dp.h>
#include <linux/usb/typec_tbt.h>

/* PIPE handler registers */
#define PIPEHANDLER_OVERRIDE 0x00
#define PIPEHANDLER_OVERRIDE_RXVALID BIT(0)
#define PIPEHANDLER_OVERRIDE_RXDETECT BIT(2)

#define PIPEHANDLER_OVERRIDE_VALUES 0x04
#define PIPEHANDLER_OVERRIDE_VAL_RXDETECT0 BIT(1)
#define PIPEHANDLER_OVERRIDE_VAL_RXDETECT1 BIT(2)

#define PIPEHANDLER_MUX_CTRL 0x0c
#define PIPEHANDLER_MUX_CTRL_CLK GENMASK(5, 3)
#define PIPEHANDLER_MUX_CTRL_DATA GENMASK(2, 0)
#define PIPEHANDLER_MUX_CTRL_CLK_OFF 0
#define PIPEHANDLER_MUX_CTRL_CLK_DUMMY 4
#define PIPEHANDLER_MUX_CTRL_DATA_DUMMY 2

#define PIPEHANDLER_LOCK_REQ 0x10
#define PIPEHANDLER_LOCK_ACK 0x14
#define PIPEHANDLER_LOCK_EN BIT(0)
#define PIPEHANDLER_LOCK_ACK_TIMEOUT_US 1000

#define PIPEHANDLER_AON_GEN 0x1c
#define PIPEHANDLER_AON_GEN_DWC3_FORCE_CLAMP_EN BIT(4)
#define PIPEHANDLER_AON_GEN_DWC3_RESET_N BIT(0)

#define PIPEHANDLER_NONSELECTED_OVERRIDE 0x20
#define PIPEHANDLER_NATIVE_RESET BIT(12)
#define PIPEHANDLER_NATIVE_POWER_DOWN GENMASK(3, 0)

/* USB2 PHY registers */
#define USB2PHY_USBCTL 0x00
#define USB2PHY_USBCTL_RUN BIT(1)
#define USB2PHY_USBCTL_ISOLATION BIT(2)

#define USB2PHY_CTL 0x04
#define USB2PHY_CTL_RESET BIT(0)
#define USB2PHY_CTL_PORT_RESET BIT(1)
#define USB2PHY_CTL_APB_RESET_N BIT(2)
#define USB2PHY_CTL_SIDDQ BIT(3)

#define USB2PHY_SIG 0x08
#define USB2PHY_SIG_VBUSDET_FORCE_VAL BIT(0)
#define USB2PHY_SIG_VBUSDET_FORCE_EN BIT(1)
#define USB2PHY_SIG_VBUSVLDEXT_FORCE_VAL BIT(2)
#define USB2PHY_SIG_VBUSVLDEXT_FORCE_EN BIT(3)
#define USB2PHY_SIG_HOST (7 << 12)

#define USB2PHY_MISCTUNE 0x1c
#define USB2PHY_MISCTUNE_APBCLK_GATE_OFF BIT(29)
#define USB2PHY_MISCTUNE_REFCLK_GATE_OFF BIT(30)

/*
 * DIAGNOSTIC: when true, the reset ops and the usb2 power cycling become
 * no-ops, leaving the PHY/pipehandler exactly in the state m1n1 left them
 * (the state the dwc3 core demonstrably soft-resets from when bound
 * directly). Used to bisect the dwc3 CSFTRST timeout; remove when solved.
 */
static bool atcphy_t6041_minimal_ops;
module_param_named(minimal_ops, atcphy_t6041_minimal_ops, bool, 0644);

struct atcphy_t6041 {
	struct device *dev;

	void __iomem *usb2phy;
	void __iomem *pipehandler;

	struct mutex lock;
	bool usb2_powered;
	bool pipehandler_up;

	struct phy *phy_usb2;
	struct phy *phy_usb3;
	struct phy_provider *phy_provider;
	struct reset_controller_dev rcdev;
	struct typec_switch_dev *sw;
	struct typec_mux_dev *mux;
};

static inline void mask32(void __iomem *reg, u32 mask, u32 set)
{
	u32 value = readl(reg);

	value &= ~mask;
	value |= set;
	writel(value, reg);
}

static inline void set32(void __iomem *reg, u32 set)
{
	mask32(reg, 0, set);
}

static inline void clear32(void __iomem *reg, u32 clear)
{
	mask32(reg, clear, 0);
}

static void atcphy_t6041_usb2_power_off(struct atcphy_t6041 *atcphy)
{
	if (atcphy_t6041_minimal_ops)
		return;

	if (!atcphy->usb2_powered)
		return;

	/* Disable the PHY, this clears USB2PHY_USBCTL_RUN */
	writel(USB2PHY_USBCTL_ISOLATION, atcphy->usb2phy + USB2PHY_USBCTL);
	udelay(10);

	/* Switch the PHY to low power mode */
	set32(atcphy->usb2phy + USB2PHY_CTL, USB2PHY_CTL_SIDDQ);
	udelay(10);

	/* Enable all resets */
	set32(atcphy->usb2phy + USB2PHY_CTL, USB2PHY_CTL_PORT_RESET);
	udelay(10);
	set32(atcphy->usb2phy + USB2PHY_CTL, USB2PHY_CTL_RESET);
	udelay(10);
	clear32(atcphy->usb2phy + USB2PHY_CTL, USB2PHY_CTL_APB_RESET_N);
	udelay(10);
	set32(atcphy->usb2phy + USB2PHY_MISCTUNE, USB2PHY_MISCTUNE_APBCLK_GATE_OFF);
	set32(atcphy->usb2phy + USB2PHY_MISCTUNE, USB2PHY_MISCTUNE_REFCLK_GATE_OFF);

	atcphy->usb2_powered = false;
}

static void atcphy_t6041_usb2_power_on(struct atcphy_t6041 *atcphy)
{
	if (atcphy_t6041_minimal_ops)
		return;

	if (atcphy->usb2_powered)
		return;

	set32(atcphy->usb2phy + USB2PHY_SIG,
	      USB2PHY_SIG_VBUSDET_FORCE_VAL | USB2PHY_SIG_VBUSDET_FORCE_EN |
		      USB2PHY_SIG_VBUSVLDEXT_FORCE_VAL | USB2PHY_SIG_VBUSVLDEXT_FORCE_EN);
	udelay(10);

	/* Take the PHY out of its low power state */
	clear32(atcphy->usb2phy + USB2PHY_CTL, USB2PHY_CTL_SIDDQ);
	udelay(10);

	/* Release reset */
	clear32(atcphy->usb2phy + USB2PHY_CTL, USB2PHY_CTL_RESET);
	udelay(10);
	clear32(atcphy->usb2phy + USB2PHY_CTL, USB2PHY_CTL_PORT_RESET);
	udelay(10);
	set32(atcphy->usb2phy + USB2PHY_CTL, USB2PHY_CTL_APB_RESET_N);
	udelay(10);
	clear32(atcphy->usb2phy + USB2PHY_MISCTUNE, USB2PHY_MISCTUNE_APBCLK_GATE_OFF);
	clear32(atcphy->usb2phy + USB2PHY_MISCTUNE, USB2PHY_MISCTUNE_REFCLK_GATE_OFF);

	/* Enable the PHY */
	writel(USB2PHY_USBCTL_RUN, atcphy->usb2phy + USB2PHY_USBCTL);

	/*
	 * Give the PHY time to stabilize its clocks before dwc3 touches it.
	 * In every hardware-verified working flow (m1n1's bring-up followed
	 * by the kernel's dwc3 init) there is ample time between the PHY
	 * power cycle and the dwc3 core soft reset; without a settle here
	 * the soft reset times out.
	 */
	msleep(50);

	atcphy->usb2_powered = true;
}

static int atcphy_t6041_configure_pipehandler_dummy(struct atcphy_t6041 *atcphy)
{
	/*
	 * Minimal, hardware-proven t6041 dummy-PHY setup (identical to m1n1
	 * usb_phy_bringup(), which is the state the dwc3 core demonstrably
	 * initializes from on this SoC). The atc.c-style sequence (RX
	 * override forces + lock/unlock dance around the mux switch) made
	 * the subsequent dwc3 core soft reset time out here.
	 * 0x9332 = NATIVE_POWER_DOWN=2 | 0x30 | 0x300 | NATIVE_RESET |
	 * DUMMY_PHY_EN.
	 */
	if (atcphy_t6041_minimal_ops)
		return 0;

	writel(FIELD_PREP(PIPEHANDLER_MUX_CTRL_CLK, PIPEHANDLER_MUX_CTRL_CLK_DUMMY) |
		       FIELD_PREP(PIPEHANDLER_MUX_CTRL_DATA, PIPEHANDLER_MUX_CTRL_DATA_DUMMY),
	       atcphy->pipehandler + PIPEHANDLER_MUX_CTRL);
	writel(0x9332, atcphy->pipehandler + PIPEHANDLER_NONSELECTED_OVERRIDE);

	return 0;
}

/* dwc3 reset controller, consumed by dwc3-apple */

static void atcphy_t6041_dwc3_reset_assert_locked(struct atcphy_t6041 *atcphy)
{
	/* Identical to the hardware-proven m1n1 usb_phy_bringup_host() cycle */
	if (atcphy_t6041_minimal_ops)
		return;

	clear32(atcphy->pipehandler + PIPEHANDLER_AON_GEN, PIPEHANDLER_AON_GEN_DWC3_RESET_N);
	set32(atcphy->pipehandler + PIPEHANDLER_AON_GEN,
	      PIPEHANDLER_AON_GEN_DWC3_FORCE_CLAMP_EN);
}

static int atcphy_t6041_dwc3_reset_assert(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct atcphy_t6041 *atcphy = container_of(rcdev, struct atcphy_t6041, rcdev);
	int ret;

	guard(mutex)(&atcphy->lock);

	atcphy_t6041_dwc3_reset_assert_locked(atcphy);

	if (atcphy->pipehandler_up) {
		ret = atcphy_t6041_configure_pipehandler_dummy(atcphy);
		if (ret)
			dev_warn(atcphy->dev, "Failed to switch PIPE to dummy: %d\n", ret);
		else
			atcphy->pipehandler_up = false;
	}

	atcphy_t6041_usb2_power_off(atcphy);

	return 0;
}

static int atcphy_t6041_dwc3_reset_deassert(struct reset_controller_dev *rcdev,
					    unsigned long id)
{
	struct atcphy_t6041 *atcphy = container_of(rcdev, struct atcphy_t6041, rcdev);

	guard(mutex)(&atcphy->lock);

	if (atcphy_t6041_minimal_ops)
		return 0;

	/*
	 * Mirror m1n1's hardware-proven release sequence: refresh the dummy
	 * PIPE mux + non-selected override while still clamped, then unclamp
	 * and release the reset.
	 */
	atcphy_t6041_configure_pipehandler_dummy(atcphy);

	clear32(atcphy->pipehandler + PIPEHANDLER_AON_GEN,
		PIPEHANDLER_AON_GEN_DWC3_FORCE_CLAMP_EN);
	set32(atcphy->pipehandler + PIPEHANDLER_AON_GEN, PIPEHANDLER_AON_GEN_DWC3_RESET_N);

	/* Let the dwc3 core come out of reset before it is programmed. */
	msleep(50);

	return 0;
}

static const struct reset_control_ops atcphy_t6041_dwc3_reset_ops = {
	.assert = atcphy_t6041_dwc3_reset_assert,
	.deassert = atcphy_t6041_dwc3_reset_deassert,
};

static int atcphy_t6041_reset_xlate(struct reset_controller_dev *rcdev,
				    const struct of_phandle_args *reset_spec)
{
	return 0;
}

/* USB2 PHY */

static int atcphy_t6041_usb2_set_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	struct atcphy_t6041 *atcphy = phy_get_drvdata(phy);

	guard(mutex)(&atcphy->lock);

	/*
	 * dwc3-apple calls this right before deasserting the dwc3 reset and
	 * initializing the core. The mode must be configured while the PHY
	 * is powered off, but the PHY must be up and clocking by the time
	 * the dwc3 core soft reset runs (it times out otherwise). Do a full
	 * off -> configure -> on cycle so this holds regardless of what
	 * state the typec mux left the PHY in.
	 */
	atcphy_t6041_usb2_power_off(atcphy);

	switch (mode) {
	case PHY_MODE_USB_HOST:
		set32(atcphy->usb2phy + USB2PHY_SIG, USB2PHY_SIG_HOST);
		break;
	case PHY_MODE_USB_DEVICE:
		clear32(atcphy->usb2phy + USB2PHY_SIG, USB2PHY_SIG_HOST);
		break;
	default:
		return -EINVAL;
	}

	atcphy_t6041_usb2_power_on(atcphy);

	return 0;
}

static const struct phy_ops atcphy_t6041_usb2_phy_ops = {
	.owner = THIS_MODULE,
	.set_mode = atcphy_t6041_usb2_set_mode,
};

/*
 * USB3 PHY: there is no SuperSpeed support (the t6040 core/lane layout is
 * not implemented), so any requested mode just (re)configures the PIPE
 * handler to the dummy PHY, which makes dwc3/xhci run USB2-only.
 */

static int atcphy_t6041_usb3_set_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	struct atcphy_t6041 *atcphy = phy_get_drvdata(phy);
	int ret;

	guard(mutex)(&atcphy->lock);

	if (atcphy->pipehandler_up)
		return 0;

	switch (mode) {
	case PHY_MODE_USB_HOST:
	case PHY_MODE_USB_DEVICE:
		ret = atcphy_t6041_configure_pipehandler_dummy(atcphy);
		/*
		 * Note: pipehandler_up deliberately stays false — the dummy
		 * PHY is the "down" state the reset/teardown paths expect.
		 */
		return ret;
	default:
		return -EINVAL;
	}
}

static int atcphy_t6041_usb3_power_off(struct phy *phy)
{
	struct atcphy_t6041 *atcphy = phy_get_drvdata(phy);
	int ret;

	guard(mutex)(&atcphy->lock);

	ret = atcphy_t6041_configure_pipehandler_dummy(atcphy);
	if (ret)
		dev_warn(atcphy->dev, "Failed to switch pipe to dummy: %d\n", ret);
	atcphy->pipehandler_up = false;

	atcphy_t6041_usb2_power_off(atcphy);

	return 0;
}

static const struct phy_ops atcphy_t6041_usb3_phy_ops = {
	.owner = THIS_MODULE,
	.set_mode = atcphy_t6041_usb3_set_mode,
	.power_off = atcphy_t6041_usb3_power_off,
};

/* typec orientation switch: USB2 D+/D- are orientation-agnostic here */

static int atcphy_t6041_sw_set(struct typec_switch_dev *sw, enum typec_orientation orientation)
{
	return 0;
}

/* typec mode mux */

static int atcphy_t6041_mux_set(struct typec_mux_dev *mux, struct typec_mux_state *state)
{
	struct atcphy_t6041 *atcphy = typec_mux_get_drvdata(mux);
	bool on;

	guard(mutex)(&atcphy->lock);

	if (state->mode == TYPEC_STATE_SAFE) {
		on = false;
	} else if (state->alt) {
		dev_warn(atcphy->dev,
			 "Alternate mode SVID 0x%x not supported (USB2-only PHY); staying USB2\n",
			 state->alt->svid);
		on = true;
	} else {
		/* Any USB mode: USB2 works, USB3/USB4 fall back to USB2 */
		on = true;
	}

	/*
	 * The USB2 PHY is powered on by the usb2 phy set_mode hook (dwc3
	 * bring-up path); here only handle the power-down on disconnect.
	 */
	if (!on)
		atcphy_t6041_usb2_power_off(atcphy);

	return 0;
}

static struct phy *atcphy_t6041_xlate(struct device *dev, const struct of_phandle_args *args)
{
	struct atcphy_t6041 *atcphy = dev_get_drvdata(dev);

	if (args->args_count != 1)
		return ERR_PTR(-EINVAL);

	switch (args->args[0]) {
	case PHY_TYPE_USB2:
		return atcphy->phy_usb2;
	case PHY_TYPE_USB3:
		return atcphy->phy_usb3;
	}

	return ERR_PTR(-ENODEV);
}

static void atcphy_t6041_mux_unregister(void *data)
{
	typec_mux_unregister(data);
}

static void atcphy_t6041_sw_unregister(void *data)
{
	typec_switch_unregister(data);
}

static int atcphy_t6041_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct atcphy_t6041 *atcphy;
	struct typec_switch_desc sw_desc = { 0 };
	struct typec_mux_desc mux_desc = { 0 };
	int ret;

	atcphy = devm_kzalloc(dev, sizeof(*atcphy), GFP_KERNEL);
	if (!atcphy)
		return -ENOMEM;

	atcphy->dev = dev;
	mutex_init(&atcphy->lock);
	dev_set_drvdata(dev, atcphy);

	atcphy->usb2phy = devm_platform_ioremap_resource_byname(pdev, "usb2phy");
	if (IS_ERR(atcphy->usb2phy))
		return PTR_ERR(atcphy->usb2phy);

	atcphy->pipehandler = devm_platform_ioremap_resource_byname(pdev, "pipehandler");
	if (IS_ERR(atcphy->pipehandler))
		return PTR_ERR(atcphy->pipehandler);

	/*
	 * m1n1 leaves the PHY running (it uses it for its USB gadget) with
	 * the pipehandler on the dummy PHY. Leave that proven state alone;
	 * dwc3-apple asserts our reset at its probe (powering the USB2 PHY
	 * off) and the first cable event brings everything up cleanly.
	 */
	atcphy->usb2_powered = true;

	atcphy->rcdev.owner = THIS_MODULE;
	atcphy->rcdev.nr_resets = 1;
	atcphy->rcdev.ops = &atcphy_t6041_dwc3_reset_ops;
	atcphy->rcdev.of_node = dev->of_node;
	atcphy->rcdev.of_reset_n_cells = 0;
	atcphy->rcdev.of_xlate = atcphy_t6041_reset_xlate;

	ret = devm_reset_controller_register(dev, &atcphy->rcdev);
	if (ret)
		return ret;

	atcphy->phy_usb2 = devm_phy_create(dev, NULL, &atcphy_t6041_usb2_phy_ops);
	if (IS_ERR(atcphy->phy_usb2))
		return PTR_ERR(atcphy->phy_usb2);
	phy_set_drvdata(atcphy->phy_usb2, atcphy);

	atcphy->phy_usb3 = devm_phy_create(dev, NULL, &atcphy_t6041_usb3_phy_ops);
	if (IS_ERR(atcphy->phy_usb3))
		return PTR_ERR(atcphy->phy_usb3);
	phy_set_drvdata(atcphy->phy_usb3, atcphy);

	atcphy->phy_provider = devm_of_phy_provider_register(dev, atcphy_t6041_xlate);
	if (IS_ERR(atcphy->phy_provider))
		return PTR_ERR(atcphy->phy_provider);

	sw_desc.drvdata = atcphy;
	sw_desc.fwnode = dev->fwnode;
	sw_desc.set = atcphy_t6041_sw_set;
	atcphy->sw = typec_switch_register(dev, &sw_desc);
	if (IS_ERR(atcphy->sw))
		return PTR_ERR(atcphy->sw);
	ret = devm_add_action_or_reset(dev, atcphy_t6041_sw_unregister, atcphy->sw);
	if (ret)
		return ret;

	mux_desc.drvdata = atcphy;
	mux_desc.fwnode = dev->fwnode;
	mux_desc.set = atcphy_t6041_mux_set;
	atcphy->mux = typec_mux_register(dev, &mux_desc);
	if (IS_ERR(atcphy->mux))
		return PTR_ERR(atcphy->mux);
	ret = devm_add_action_or_reset(dev, atcphy_t6041_mux_unregister, atcphy->mux);
	if (ret)
		return ret;

	dev_info(dev, "Apple T6041 ATC PHY initialized (USB2 only)\n");

	return 0;
}

static const struct of_device_id atcphy_t6041_match[] = {
	{ .compatible = "apple,t6041-atcphy" },
	{},
};
MODULE_DEVICE_TABLE(of, atcphy_t6041_match);

static struct platform_driver atcphy_t6041_driver = {
	.driver = {
		.name = "phy-apple-atc-t6041",
		.of_match_table = atcphy_t6041_match,
	},
	.probe = atcphy_t6041_probe,
};
module_platform_driver(atcphy_t6041_driver);

MODULE_AUTHOR("Marcel Bierling <marcel@hackerman.art>");
MODULE_DESCRIPTION("Apple T6041 Type-C PHY driver (USB2 only)");
MODULE_LICENSE("GPL");
