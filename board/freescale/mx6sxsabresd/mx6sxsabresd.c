/*
 * Copyright (C) 2014 Freescale Semiconductor, Inc.
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/io.h>
#include <asm/imx-common/mxc_i2c.h>
#include <linux/sizes.h>
#include <common.h>
#include <fsl_esdhc.h>
#include <mmc.h>
#include <i2c.h>
#include <miiphy.h>
#include <netdev.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
#include "../common/pfuze.h"
#include <usb.h>
#include <usb/ehci-ci.h>

#define ASTRO 1

DECLARE_GLOBAL_DATA_PTR;
#define GPMI_PAD_CTRL0 (PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_100K_UP)
#define GPMI_PAD_CTRL1 (PAD_CTL_DSE_40ohm | PAD_CTL_SPEED_MED | \
			PAD_CTL_SRE_FAST)
#define GPMI_PAD_CTRL2 (GPMI_PAD_CTRL0 | GPMI_PAD_CTRL1)
#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED | \
		      PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_22K_UP  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define I2C_PAD_CTRL    (PAD_CTL_PKE | PAD_CTL_PUE |            \
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |               \
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS |			\
	PAD_CTL_ODE)
#define I2C_PAD MUX_PAD_CTRL(I2C_PAD_CTRL)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP | PAD_CTL_PUE |     \
	PAD_CTL_SPEED_HIGH   |                                   \
	PAD_CTL_DSE_48ohm   | PAD_CTL_SRE_FAST)

#define ENET_CLK_PAD_CTRL  (PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_120ohm   | PAD_CTL_SRE_FAST)

#define ENET_RX_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |          \
	PAD_CTL_SPEED_HIGH   | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL    (PAD_CTL_PKE | PAD_CTL_PUE |            \
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |               \
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS |			\
	PAD_CTL_ODE)

#define LCD_PAD_CTRL    (PAD_CTL_HYS | PAD_CTL_PUS_100K_UP | PAD_CTL_PUE | \
	PAD_CTL_PKE | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm)
#ifdef ASTRO
iomux_v3_cfg_t gpmi_pads[] = {
  MX6_PAD_NAND_ALE__RAWNAND_ALE       | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
  MX6_PAD_NAND_CE0_B__RAWNAND_CE0_B   | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
  MX6_PAD_NAND_CLE__RAWNAND_CLE       | MUX_PAD_CTRL(GPMI_PAD_CTRL2),     
  MX6_PAD_NAND_DATA00__RAWNAND_DATA00 | MUX_PAD_CTRL(GPMI_PAD_CTRL2),          
  MX6_PAD_NAND_DATA01__RAWNAND_DATA01 | MUX_PAD_CTRL(GPMI_PAD_CTRL2),          
  MX6_PAD_NAND_DATA02__RAWNAND_DATA02 | MUX_PAD_CTRL(GPMI_PAD_CTRL2),          
  MX6_PAD_NAND_DATA03__RAWNAND_DATA03      | MUX_PAD_CTRL(GPMI_PAD_CTRL2),     
  MX6_PAD_NAND_DATA04__RAWNAND_DATA04      | MUX_PAD_CTRL(GPMI_PAD_CTRL2),     
  MX6_PAD_NAND_DATA05__RAWNAND_DATA05      | MUX_PAD_CTRL(GPMI_PAD_CTRL2),     
  MX6_PAD_NAND_DATA06__RAWNAND_DATA06      | MUX_PAD_CTRL(GPMI_PAD_CTRL2),     
  MX6_PAD_NAND_DATA07__RAWNAND_DATA07      | MUX_PAD_CTRL(GPMI_PAD_CTRL2),     
  MX6_PAD_NAND_READY_B__RAWNAND_READY_B    | MUX_PAD_CTRL(GPMI_PAD_CTRL0),
  MX6_PAD_NAND_RE_B__RAWNAND_RE_B          | MUX_PAD_CTRL(GPMI_PAD_CTRL2),     
  MX6_PAD_NAND_WE_B__RAWNAND_WE_B          | MUX_PAD_CTRL(GPMI_PAD_CTRL2),     
  MX6_PAD_NAND_WP_B__RAWNAND_WP_B          | MUX_PAD_CTRL(GPMI_PAD_CTRL2),
};
static void setup_gpmi_nand(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	/* config gpmi nand iomux */
	imx_iomux_v3_setup_multiple_pads(gpmi_pads, ARRAY_SIZE(gpmi_pads));

	//       	setup_gpmi_io_clk((MXC_CCM_CS2CDR_QSPI2_CLK_PODF(0) |
	//		MXC_CCM_CS2CDR_QSPI2_CLK_PRED(3) |
	//		MXC_CCM_CS2CDR_QSPI2_CLK_SEL(3)));

	/* enable apbh clock gating */
	//	setbits_le32(&mxc_ccm->CCGR0, MXC_CCM_CCGR0_APBHDMA_MASK);
}

#endif
int dram_init(void)
{
	gd->ram_size = imx_ddr_size();

	return 0;
}
#ifdef ASTRO
static iomux_v3_cfg_t const uart1_pads[] = {
  MX6_PAD_ENET2_COL__UART1_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
  MX6_PAD_ENET2_CRS__UART1_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};
static iomux_v3_cfg_t const uart2_pads[] = {
  MX6_PAD_GPIO1_IO06__UART2_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
  MX6_PAD_GPIO1_IO07__UART2_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};
static iomux_v3_cfg_t const uart3_pads[] = {
  MX6_PAD_SD3_DATA5__UART3_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
  MX6_PAD_SD3_DATA4__UART3_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};
static iomux_v3_cfg_t const uart4_pads[] = {
  MX6_PAD_SD2_DATA1__UART4_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
  MX6_PAD_SD2_DATA0__UART4_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};
static iomux_v3_cfg_t const uart5_pads[] = {
  MX6_PAD_KEY_COL3__UART5_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
  MX6_PAD_KEY_ROW3__UART5_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};
static iomux_v3_cfg_t const uart6_pads[] = {
  MX6_PAD_KEY_COL1__UART6_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
  MX6_PAD_SD2_DATA2__UART6_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};
#else
static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_GPIO1_IO04__UART1_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_GPIO1_IO05__UART1_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};
#endif
#ifdef ASTRO
static iomux_v3_cfg_t const usdhc4_pads[] = {
  MX6_PAD_SD4_CLK__USDHC4_CLK	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
  MX6_PAD_SD4_CMD__USDHC4_CMD	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
  MX6_PAD_SD4_DATA0__USDHC4_DATA0	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
  MX6_PAD_SD4_DATA1__USDHC4_DATA1	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
  MX6_PAD_SD4_DATA2__USDHC4_DATA2	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
  MX6_PAD_SD4_DATA3__USDHC4_DATA3 	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
  MX6_PAD_KEY_COL2__USDHC4_CD_B	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
};
static iomux_v3_cfg_t const usdhc2_pads[] = {

};

static iomux_v3_cfg_t const usdhc3_pads[] = {
};

#else
static iomux_v3_cfg_t const usdhc2_pads[] = {
	MX6_PAD_SD2_CLK__USDHC2_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_CMD__USDHC2_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DATA0__USDHC2_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DATA1__USDHC2_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DATA2__USDHC2_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DATA3__USDHC2_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc3_pads[] = {
	MX6_PAD_SD3_CLK__USDHC3_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_CMD__USDHC3_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DATA0__USDHC3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DATA1__USDHC3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DATA2__USDHC3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DATA3__USDHC3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DATA4__USDHC3_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DATA5__USDHC3_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DATA6__USDHC3_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DATA7__USDHC3_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),

	/* CD pin */
	MX6_PAD_KEY_COL0__GPIO2_IO_10 | MUX_PAD_CTRL(NO_PAD_CTRL),

	/* RST_B, used for power reset cycle */
	MX6_PAD_KEY_COL1__GPIO2_IO_11 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc4_pads[] = {
	MX6_PAD_SD4_CLK__USDHC4_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_CMD__USDHC4_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DATA0__USDHC4_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DATA1__USDHC4_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DATA2__USDHC4_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DATA3__USDHC4_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DATA7__GPIO6_IO_21 | MUX_PAD_CTRL(NO_PAD_CTRL),
};
#endif
#ifdef ASTRO
static iomux_v3_cfg_t const fec1_pads[] = {
	MX6_PAD_ENET1_MDIO__ENET1_MDIO | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_MDC__ENET1_MDC   | MUX_PAD_CTRL(ENET_PAD_CTRL),

	MX6_PAD_RGMII1_RD0__ENET1_RX_DATA_0 | MUX_PAD_CTRL(ENET_PAD_CTRL), 
	MX6_PAD_RGMII1_RD1__ENET1_RX_DATA_1 | MUX_PAD_CTRL(ENET_PAD_CTRL), 
	MX6_PAD_RGMII1_RD2__ENET1_RX_DATA_2 | MUX_PAD_CTRL(ENET_PAD_CTRL), 
	MX6_PAD_RGMII1_RD3__ENET1_RX_DATA_3 | MUX_PAD_CTRL(ENET_PAD_CTRL), 
	MX6_PAD_RGMII1_RXC__ENET1_RX_CLK    | MUX_PAD_CTRL(ENET_PAD_CTRL), 
	MX6_PAD_RGMII1_RX_CTL__ENET1_RX_EN  | MUX_PAD_CTRL(ENET_PAD_CTRL), 
	MX6_PAD_RGMII1_TD0__ENET1_TX_DATA_0 | MUX_PAD_CTRL(ENET_PAD_CTRL), 
	MX6_PAD_RGMII1_TD1__ENET1_TX_DATA_1 | MUX_PAD_CTRL(ENET_PAD_CTRL), 
	MX6_PAD_RGMII1_TD2__ENET1_TX_DATA_2 | MUX_PAD_CTRL(ENET_PAD_CTRL), 
	MX6_PAD_RGMII1_TD3__ENET1_TX_DATA_3 | MUX_PAD_CTRL(ENET_PAD_CTRL), 
	MX6_PAD_RGMII1_TXC__ENET1_RGMII_TXC | MUX_PAD_CTRL(ENET_PAD_CTRL), 
	MX6_PAD_RGMII1_TX_CTL__ENET1_TX_EN  | MUX_PAD_CTRL(ENET_PAD_CTRL), 
};
static iomux_v3_cfg_t const fec2_pads[] = {
	MX6_PAD_RGMII2_RD0__ENET2_RX_DATA_0  | MUX_PAD_CTRL(ENET_PAD_CTRL), 
	MX6_PAD_RGMII2_RD1__ENET2_RX_DATA_1  | MUX_PAD_CTRL(ENET_PAD_CTRL), 
	MX6_PAD_RGMII2_RD2__ENET2_RX_DATA_2  | MUX_PAD_CTRL(ENET_PAD_CTRL), 
	MX6_PAD_RGMII2_RD3__ENET2_RX_DATA_3  | MUX_PAD_CTRL(ENET_PAD_CTRL), 
	MX6_PAD_RGMII2_RXC__ENET2_RX_ER      | MUX_PAD_CTRL(ENET_PAD_CTRL), 
	MX6_PAD_RGMII2_RX_CTL__ENET2_RX_EN   | MUX_PAD_CTRL(ENET_PAD_CTRL), 
	MX6_PAD_RGMII2_TD0__ENET2_TX_DATA_0  | MUX_PAD_CTRL(ENET_PAD_CTRL), 
	MX6_PAD_RGMII2_TD1__ENET2_TX_DATA_1  | MUX_PAD_CTRL(ENET_PAD_CTRL), 
	MX6_PAD_RGMII2_TD2__ENET2_TX_DATA_2  | MUX_PAD_CTRL(ENET_PAD_CTRL), 
	MX6_PAD_RGMII2_TD3__ENET2_TX_DATA_3  | MUX_PAD_CTRL(ENET_PAD_CTRL), 
	MX6_PAD_RGMII2_TXC__ENET2_RGMII_TXC  | MUX_PAD_CTRL(ENET_PAD_CTRL), 
	MX6_PAD_RGMII2_TX_CTL__ENET2_TX_EN   | MUX_PAD_CTRL(ENET_PAD_CTRL), 
	MX6_PAD_ENET1_COL__ENET2_MDC         | MUX_PAD_CTRL(ENET_PAD_CTRL), 
	MX6_PAD_ENET1_CRS__ENET2_MDIO        | MUX_PAD_CTRL(ENET_PAD_CTRL), 
	MX6_PAD_ENET2_RX_CLK__ENET2_RX_CLK   | MUX_PAD_CTRL(ENET_PAD_CTRL), 


	/* add the  PHY Reset */

};
#else
static iomux_v3_cfg_t const fec1_pads[] = {
	MX6_PAD_ENET1_MDC__ENET1_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_MDIO__ENET1_MDIO | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII1_RX_CTL__ENET1_RX_EN | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX6_PAD_RGMII1_RD0__ENET1_RX_DATA_0 | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX6_PAD_RGMII1_RD1__ENET1_RX_DATA_1 | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX6_PAD_RGMII1_RD2__ENET1_RX_DATA_2 | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX6_PAD_RGMII1_RD3__ENET1_RX_DATA_3 | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX6_PAD_RGMII1_RXC__ENET1_RX_CLK | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX6_PAD_RGMII1_TX_CTL__ENET1_TX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII1_TD0__ENET1_TX_DATA_0 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII1_TD1__ENET1_TX_DATA_1 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII1_TD2__ENET1_TX_DATA_2 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII1_TD3__ENET1_TX_DATA_3 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII1_TXC__ENET1_RGMII_TXC | MUX_PAD_CTRL(ENET_PAD_CTRL),
};

static iomux_v3_cfg_t const peri_3v3_pads[] = {
	MX6_PAD_QSPI1A_DATA0__GPIO4_IO_16 | MUX_PAD_CTRL(NO_PAD_CTRL),
};
#endif
static iomux_v3_cfg_t const phy_control_pads[] = {
	/* 25MHz Ethernet PHY Clock */
#ifdef ASTRO
	MX6_PAD_ENET1_TX_CLK__ENET1_REF_CLK1 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_TX_CLK__ENET2_REF_CLK2  | MUX_PAD_CTRL(ENET_PAD_CTRL), 

#else
	MX6_PAD_ENET2_RX_CLK__ENET2_REF_CLK_25M | MUX_PAD_CTRL(ENET_CLK_PAD_CTRL),

	/* ENET PHY Power */
	MX6_PAD_ENET2_COL__GPIO2_IO_6 | MUX_PAD_CTRL(NO_PAD_CTRL),

	/* AR8031 PHY Reset */
	MX6_PAD_ENET2_CRS__GPIO2_IO_7 | MUX_PAD_CTRL(NO_PAD_CTRL),
#endif
};


static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
#ifdef ASTRO
	imx_iomux_v3_setup_multiple_pads(uart2_pads, ARRAY_SIZE(uart2_pads));
	imx_iomux_v3_setup_multiple_pads(uart3_pads, ARRAY_SIZE(uart3_pads));
	imx_iomux_v3_setup_multiple_pads(uart4_pads, ARRAY_SIZE(uart4_pads));
	imx_iomux_v3_setup_multiple_pads(uart5_pads, ARRAY_SIZE(uart5_pads));
	imx_iomux_v3_setup_multiple_pads(uart6_pads, ARRAY_SIZE(uart6_pads));
#endif
}

static int setup_fec(void)
{
	struct iomuxc *iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;
	struct anatop_regs *anatop = (struct anatop_regs *)ANATOP_BASE_ADDR;
	int reg, ret;

	/* Use 125MHz anatop loopback REF_CLK1 for ENET1 */
	clrsetbits_le32(&iomuxc_regs->gpr[1], IOMUX_GPR1_FEC1_MASK, 0);

	ret = enable_fec_anatop_clock(0, ENET_125MHZ);
	if (ret)
		return ret;

	imx_iomux_v3_setup_multiple_pads(phy_control_pads,
					 ARRAY_SIZE(phy_control_pads));

	/* Enable the ENET power, active low */
	gpio_direction_output(IMX_GPIO_NR(2, 6) , 0);

	/* Reset AR8031 PHY */
	gpio_direction_output(IMX_GPIO_NR(2, 7) , 0);
	mdelay(10);
	gpio_set_value(IMX_GPIO_NR(2, 7), 1);

	reg = readl(&anatop->pll_enet);
	reg |= BM_ANADIG_PLL_ENET_REF_25M_ENABLE;
	writel(reg, &anatop->pll_enet);

	return 0;
}

int board_eth_init(bd_t *bis)
{
	imx_iomux_v3_setup_multiple_pads(fec1_pads, ARRAY_SIZE(fec1_pads));
#ifdef ASTRO
	imx_iomux_v3_setup_multiple_pads(fec2_pads, ARRAY_SIZE(fec1_pads));
#endif
	setup_fec();

	return 0; //cpu_eth_init(bis);
}

#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)
#ifdef ASTRO
static struct i2c_pads_info i2c_pad_info1 = {

	.scl = {
		.i2c_mode =  MX6_PAD_GPIO1_IO00__I2C1_SCL | I2C_PAD,
		.gpio_mode = MX6_PAD_GPIO1_IO00__GPIO1_IO_0 | I2C_PAD,
		.gp = IMX_GPIO_NR(1, 00)
	},
	.sda = {
		.i2c_mode = MX6_PAD_GPIO1_IO01__I2C1_SDA | I2C_PAD,
		.gpio_mode = MX6_PAD_GPIO1_IO01__GPIO1_IO_1 | I2C_PAD,
		.gp = IMX_GPIO_NR(1, 01)
	}
};
static struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode = MX6_PAD_GPIO1_IO02__I2C2_SCL | I2C_PAD,
		.gpio_mode = MX6_PAD_GPIO1_IO02__GPIO1_IO_2 | I2C_PAD,
		.gp = IMX_GPIO_NR(1, 02)
	},
	.sda = {
		.i2c_mode = MX6_PAD_GPIO1_IO03__I2C2_SDA | I2C_PAD,
		.gpio_mode = MX6_PAD_GPIO1_IO03__GPIO1_IO_3 | I2C_PAD,
		.gp = IMX_GPIO_NR(1, 03)
	}
};

#else

/* I2C1 for PMIC */
static struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX6_PAD_GPIO1_IO00__I2C1_SCL | PC,
		.gpio_mode = MX6_PAD_GPIO1_IO00__GPIO1_IO_0 | PC,
		.gp = IMX_GPIO_NR(1, 0),
	},
	.sda = {
		.i2c_mode = MX6_PAD_GPIO1_IO01__I2C1_SDA | PC,
		.gpio_mode = MX6_PAD_GPIO1_IO01__GPIO1_IO_1 | PC,
		.gp = IMX_GPIO_NR(1, 1),
	},
};
#endif
int power_init_board(void)
{
	struct pmic *p;
	unsigned int reg;
	int ret;

	p = pfuze_common_init(I2C_PMIC);
	if (!p)
		return -ENODEV;

	ret = pfuze_mode_init(p, APS_PFM);
	if (ret < 0)
		return ret;

	/* Enable power of VGEN5 3V3, needed for SD3 */
	pmic_reg_read(p, PFUZE100_VGEN5VOL, &reg);
	reg &= ~LDO_VOL_MASK;
	reg |= (LDOB_3_30V | (1 << LDO_EN));
	pmic_reg_write(p, PFUZE100_VGEN5VOL, reg);

	return 0;
}
#if !defined(ASTRO)
#ifdef CONFIG_USB_EHCI_MX6
#define USB_OTHERREGS_OFFSET	0x800
#define UCTRL_PWR_POL		(1 << 9)

static iomux_v3_cfg_t const usb_otg_pads[] = {
	/* OGT1 */
	MX6_PAD_GPIO1_IO09__USB_OTG1_PWR | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_GPIO1_IO10__ANATOP_OTG1_ID | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* OTG2 */
	MX6_PAD_GPIO1_IO12__USB_OTG2_PWR | MUX_PAD_CTRL(NO_PAD_CTRL)
};

static void setup_usb(void)
{
	imx_iomux_v3_setup_multiple_pads(usb_otg_pads,
					 ARRAY_SIZE(usb_otg_pads));
}

int board_usb_phy_mode(int port)
{
	if (port == 1)
		return USB_INIT_HOST;
	else
		return usb_phy_mode(port);
}

int board_ehci_hcd_init(int port)
{
	u32 *usbnc_usb_ctrl;

	if (port > 1)
		return -EINVAL;

	usbnc_usb_ctrl = (u32 *)(USB_BASE_ADDR + USB_OTHERREGS_OFFSET +
				 port * 4);

	/* Set Power polarity */
	setbits_le32(usbnc_usb_ctrl, UCTRL_PWR_POL);

	return 0;
}
#endif
#endif !ASTRO
int board_phy_config(struct phy_device *phydev)
{
	/*
	 * Enable 1.8V(SEL_1P5_1P8_POS_REG) on
	 * Phy control debug reg 0
	 */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x1f);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x8);

	/* rgmii tx clock delay enable */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x05);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x100);

	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

int board_early_init_f(void)
{
	setup_iomux_uart();
#ifdef ASTRO
	enable_rgb();
#else
	/* Enable PERI_3V3, which is used by SD2, ENET, LVDS, BT */
	imx_iomux_v3_setup_multiple_pads(peri_3v3_pads,
					 ARRAY_SIZE(peri_3v3_pads));

	/* Active high for ncp692 */
	gpio_direction_output(IMX_GPIO_NR(4, 16) , 1);

#ifdef CONFIG_USB_EHCI_MX6
	setup_usb();
#endif
#endif ASTRO
	return 0;
}

static struct fsl_esdhc_cfg usdhc_cfg[3] = {
	{USDHC2_BASE_ADDR, 0, 4},
	{USDHC3_BASE_ADDR},
	{USDHC4_BASE_ADDR},
};

#define USDHC3_CD_GPIO	IMX_GPIO_NR(2, 10)
#define USDHC3_PWR_GPIO	IMX_GPIO_NR(2, 11)
#define USDHC4_CD_GPIO	IMX_GPIO_NR(6, 21)

int board_mmc_get_env_dev(int devno)
{
	return devno - 1;
}

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;
#ifdef ASTRO
	switch (cfg->esdhc_base) {
	case USDHC2_BASE_ADDR:
		ret = 0; /* Assume uSDHC2 is never always present */
		break;
	case USDHC3_BASE_ADDR:
	  ret = 0; //!gpio_get_value(USDHC3_CD_GPIO);
		break;
	case USDHC4_BASE_ADDR:
	  ret = 1; //!gpio_get_value(USDHC4_CD_GPIO);
		break;
	}
#else
	switch (cfg->esdhc_base) {
	case USDHC2_BASE_ADDR:
		ret = 1; /* Assume uSDHC2 is always present */
		break;
	case USDHC3_BASE_ADDR:
		ret = !gpio_get_value(USDHC3_CD_GPIO);
		break;
	case USDHC4_BASE_ADDR:
		ret = !gpio_get_value(USDHC4_CD_GPIO);
		break;
	}
#endif
	return ret;
}

int board_mmc_init(bd_t *bis)
{
#ifndef CONFIG_SPL_BUILD
	int i, ret;

	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-Boot device node)    (Physical Port)
	 * mmc0                    USDHC2
	 * mmc1                    USDHC3
	 * mmc2                    USDHC4
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			imx_iomux_v3_setup_multiple_pads(
				usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
			break;
		case 1:
			imx_iomux_v3_setup_multiple_pads(
				usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
			gpio_direction_input(USDHC3_CD_GPIO);
			gpio_direction_output(USDHC3_PWR_GPIO, 1);
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			break;
		case 2:
			imx_iomux_v3_setup_multiple_pads(
				usdhc4_pads, ARRAY_SIZE(usdhc4_pads));
			gpio_direction_input(USDHC4_CD_GPIO);
			usdhc_cfg[2].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
				"(%d) than supported by the board\n", i + 1);
			return -EINVAL;
			}

			ret = fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
			if (ret) {
				printf("Warning: failed to initialize mmc dev %d\n", i);
				return ret;
			}
	}

	return 0;
#else
	struct src *src_regs = (struct src *)SRC_BASE_ADDR;
	u32 val;
	u32 port;

	val = readl(&src_regs->sbmr1);

	if ((val & 0xc0) != 0x40) {
		printf("Not boot from USDHC!\n");
		return -EINVAL;
	}

	port = (val >> 11) & 0x3;
	printf("port %d\n", port);
	switch (port) {
	case 1:
		imx_iomux_v3_setup_multiple_pads(
			usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
		usdhc_cfg[0].esdhc_base = USDHC2_BASE_ADDR;
		break;
	case 2:
		imx_iomux_v3_setup_multiple_pads(
			usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
		gpio_direction_input(USDHC3_CD_GPIO);
		gpio_direction_output(USDHC3_PWR_GPIO, 1);
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
		usdhc_cfg[0].esdhc_base = USDHC3_BASE_ADDR;
		break;
	case 3:
		imx_iomux_v3_setup_multiple_pads(
			usdhc4_pads, ARRAY_SIZE(usdhc4_pads));
		gpio_direction_input(USDHC4_CD_GPIO);
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
		usdhc_cfg[0].esdhc_base = USDHC4_BASE_ADDR;
		break;
	}

	gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
	return fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
#endif
}
#ifdef ASTRO
static iomux_v3_cfg_t const ecspi1_pads[] = {
  MX6_PAD_KEY_COL0__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
  MX6_PAD_KEY_ROW0__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
  MX6_PAD_KEY_ROW1__ECSPI1_SS0 | MUX_PAD_CTRL(SPI_PAD_CTRL),
  MX6_PAD_QSPI1A_DATA1__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
};
static iomux_v3_cfg_t const ecspi3_pads[] = {
  MX6_PAD_SD4_DATA4__ECSPI3_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
  MX6_PAD_SD4_DATA5__ECSPI3_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
  MX6_PAD_SD4_DATA6__ECSPI3_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
  MX6_PAD_SD4_DATA7__ECSPI3_SS0  | MUX_PAD_CTRL(SPI_PAD_CTRL),
};
static iomux_v3_cfg_t const ecspi4_pads[] = {
  MX6_PAD_SD2_CLK__ECSPI4_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
  MX6_PAD_SD2_CMD__ECSPI4_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
  MX6_PAD_SD2_DATA3__ECSPI4_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
  MX6_PAD_SD3_DATA2__ECSPI4_SS0 | MUX_PAD_CTRL(SPI_PAD_CTRL),
};
static void setup_spi(void)
{
	imx_iomux_v3_setup_multiple_pads(ecspi1_pads, ARRAY_SIZE(ecspi1_pads));
	imx_iomux_v3_setup_multiple_pads(ecspi3_pads, ARRAY_SIZE(ecspi3_pads));
	imx_iomux_v3_setup_multiple_pads(ecspi4_pads, ARRAY_SIZE(ecspi4_pads));
}

#else

#ifdef CONFIG_FSL_QSPI

#define QSPI_PAD_CTRL1	\
	(PAD_CTL_SRE_FAST | PAD_CTL_SPEED_HIGH | \
	 PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_47K_UP | PAD_CTL_DSE_40ohm)

static iomux_v3_cfg_t const quadspi_pads[] = {
	MX6_PAD_NAND_WP_B__QSPI2_A_DATA_0	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_READY_B__QSPI2_A_DATA_1	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_CE0_B__QSPI2_A_DATA_2	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_CE1_B__QSPI2_A_DATA_3	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_ALE__QSPI2_A_SS0_B		| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_CLE__QSPI2_A_SCLK		| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_DATA07__QSPI2_A_DQS	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_DATA01__QSPI2_B_DATA_0	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_DATA00__QSPI2_B_DATA_1	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_WE_B__QSPI2_B_DATA_2	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_RE_B__QSPI2_B_DATA_3	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_DATA03__QSPI2_B_SS0_B	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_DATA02__QSPI2_B_SCLK	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_NAND_DATA05__QSPI2_B_DQS	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
};

int board_qspi_init(void)
{
	/* Set the iomux */
	imx_iomux_v3_setup_multiple_pads(quadspi_pads,
					 ARRAY_SIZE(quadspi_pads));

	/* Set the clock */
	enable_qspi_clk(1);

	return 0;
}
#endif
#endif ASTRO
#ifdef ASTRO
static iomux_v3_cfg_t const rgb_pads[] = {
  MX6_PAD_GPIO1_IO04__GPIO1_IO_4  | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_GPIO1_IO05__GPIO1_IO_5 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_GPIO1_IO08__GPIO1_IO_8 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_GPIO1_IO09__GPIO1_IO_9 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_GPIO1_IO10__GPIO1_IO_10 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_GPIO1_IO11__GPIO1_IO_11 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_GPIO1_IO12__GPIO1_IO_12 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_GPIO1_IO13__GPIO1_IO_13 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_KEY_COL4__GPIO2_IO_14   | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_KEY_ROW4__GPIO2_IO_19   | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_LCD1_CLK__GPIO3_IO_0    | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_LCD1_DATA00__SRC_BT_CFG_0 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_LCD1_DATA01__SRC_BT_CFG_1 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_LCD1_DATA02__SRC_BT_CFG_2 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_LCD1_DATA03__SRC_BT_CFG_3 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_LCD1_DATA04__SRC_BT_CFG_4 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_LCD1_DATA05__SRC_BT_CFG_5 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_LCD1_DATA06__SRC_BT_CFG_6 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_LCD1_DATA07__SRC_BT_CFG_7 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_LCD1_DATA08__SRC_BT_CFG_8 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_LCD1_DATA09__SRC_BT_CFG_9 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_LCD1_DATA10__SRC_BT_CFG_10 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_LCD1_DATA11__SRC_BT_CFG_11 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_LCD1_DATA12__SRC_BT_CFG_12 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_LCD1_DATA13__SRC_BT_CFG_13 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_LCD1_DATA14__SRC_BT_CFG_14 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_LCD1_DATA15__SRC_BT_CFG_15 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_LCD1_DATA16__SRC_BT_CFG_24 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_LCD1_DATA17__SRC_BT_CFG_25 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_LCD1_DATA18__SRC_BT_CFG_26 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_LCD1_DATA19__SRC_BT_CFG_27 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_LCD1_DATA20__SRC_BT_CFG_28 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_LCD1_DATA21__SRC_BT_CFG_29 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_LCD1_DATA22__SRC_BT_CFG_30 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_LCD1_DATA23__SRC_BT_CFG_31 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_LCD1_ENABLE__GPIO3_IO_25 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_LCD1_HSYNC__GPIO3_IO_26  | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_LCD1_RESET__GPIO3_IO_27  | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_LCD1_VSYNC__GPIO3_IO_28  | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_QSPI1A_DATA0__GPIO4_IO_16 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_QSPI1A_DATA2__GPIO4_IO_18 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_QSPI1A_DATA3__GPIO4_IO_19 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_QSPI1A_DQS__GPIO4_IO_20   | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_QSPI1A_SCLK__GPIO4_IO_21  | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_QSPI1A_SS0_B__GPIO4_IO_22 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_QSPI1A_SS1_B__GPIO4_IO_23 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_QSPI1B_DATA0__GPIO4_IO_24 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_QSPI1B_DATA1__GPIO4_IO_25 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_QSPI1B_DATA2__GPIO4_IO_26 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_QSPI1B_DATA3__GPIO4_IO_27 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_QSPI1B_DQS__GPIO4_IO_28   | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_QSPI1B_SCLK__GPIO4_IO_29  | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_QSPI1B_SS0_B__GPIO4_IO_30 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_QSPI1B_SS1_B__GPIO4_IO_31 | MUX_PAD_CTRL(NO_PAD_CTRL),
  
  MX6_PAD_SD3_CLK__GPIO7_IO_0 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_SD3_CMD__GPIO7_IO_1 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_SD3_DATA0__GPIO7_IO_2 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_SD3_DATA1__GPIO7_IO_3 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_SD3_DATA3__GPIO7_IO_5 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_SD3_DATA6__GPIO7_IO_8 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_SD3_DATA7__GPIO7_IO_9 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_SD4_RESET_B__GPIO6_IO_22 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_USB_H_DATA__GPIO7_IO_10 | MUX_PAD_CTRL(NO_PAD_CTRL),
  MX6_PAD_USB_H_STROBE__GPIO7_IO_11 | MUX_PAD_CTRL(NO_PAD_CTRL),
};
void enable_rgb()
{
	imx_iomux_v3_setup_multiple_pads(rgb_pads, ARRAY_SIZE(rgb_pads));
}



#else
#ifdef CONFIG_VIDEO_MXS
static iomux_v3_cfg_t const lcd_pads[] = {
	MX6_PAD_LCD1_CLK__LCDIF1_CLK | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_ENABLE__LCDIF1_ENABLE | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_HSYNC__LCDIF1_HSYNC | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_VSYNC__LCDIF1_VSYNC | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA00__LCDIF1_DATA_0 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA01__LCDIF1_DATA_1 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA02__LCDIF1_DATA_2 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA03__LCDIF1_DATA_3 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA04__LCDIF1_DATA_4 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA05__LCDIF1_DATA_5 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA06__LCDIF1_DATA_6 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA07__LCDIF1_DATA_7 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA08__LCDIF1_DATA_8 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA09__LCDIF1_DATA_9 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA10__LCDIF1_DATA_10 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA11__LCDIF1_DATA_11 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA12__LCDIF1_DATA_12 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA13__LCDIF1_DATA_13 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA14__LCDIF1_DATA_14 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA15__LCDIF1_DATA_15 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA16__LCDIF1_DATA_16 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA17__LCDIF1_DATA_17 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA18__LCDIF1_DATA_18 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA19__LCDIF1_DATA_19 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA20__LCDIF1_DATA_20 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA21__LCDIF1_DATA_21 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA22__LCDIF1_DATA_22 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_DATA23__LCDIF1_DATA_23 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD1_RESET__GPIO3_IO_27 | MUX_PAD_CTRL(NO_PAD_CTRL),

	/* Use GPIO for Brightness adjustment, duty cycle = period */
	MX6_PAD_SD1_DATA2__GPIO6_IO_4 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static int setup_lcd(void)
{
	enable_lcdif_clock(LCDIF1_BASE_ADDR);

	imx_iomux_v3_setup_multiple_pads(lcd_pads, ARRAY_SIZE(lcd_pads));

	/* Reset the LCD */
	gpio_direction_output(IMX_GPIO_NR(3, 27) , 0);
	udelay(500);
	gpio_direction_output(IMX_GPIO_NR(3, 27) , 1);

	/* Set Brightness to high */
	gpio_direction_output(IMX_GPIO_NR(6, 4) , 1);

	return 0;
}
#endif
#endif ASTRO
int board_init(void)
{
	/* Address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;
#ifdef ASTRO
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);
	setup_spi();
#else

#ifdef CONFIG_SYS_I2C_MXC
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
#endif

#ifdef CONFIG_FSL_QSPI
	board_qspi_init();
#endif

#ifdef CONFIG_VIDEO_MXS
	setup_lcd();
#endif
#endif ASTRO
	return 0;
}

int checkboard(void)
{
#ifdef ASTRO
	puts("Board: Astronautics Comm Server IMx6\n");
#else
	puts("Board: MX6SX SABRE SDB\n");
#endif
	return 0;
}

#ifdef CONFIG_SPL_BUILD
#include <libfdt.h>
#include <spl.h>
#include <asm/arch/mx6-ddr.h>

const struct mx6sx_iomux_ddr_regs mx6_ddr_ioregs = {
	.dram_dqm0 = 0x00000028,
	.dram_dqm1 = 0x00000028,
	.dram_dqm2 = 0x00000028,
	.dram_dqm3 = 0x00000028,
	.dram_ras = 0x00000020,
	.dram_cas = 0x00000020,
	.dram_odt0 = 0x00000020,
	.dram_odt1 = 0x00000020,
	.dram_sdba2 = 0x00000000,
	.dram_sdcke0 = 0x00003000,
	.dram_sdcke1 = 0x00003000,
	.dram_sdclk_0 = 0x00000030,
	.dram_sdqs0 = 0x00000028,
	.dram_sdqs1 = 0x00000028,
	.dram_sdqs2 = 0x00000028,
	.dram_sdqs3 = 0x00000028,
	.dram_reset = 0x00000020,
};

const struct mx6sx_iomux_grp_regs mx6_grp_ioregs = {
	.grp_addds = 0x00000020,
	.grp_ddrmode_ctl = 0x00020000,
	.grp_ddrpke = 0x00000000,
	.grp_ddrmode = 0x00020000,
	.grp_b0ds = 0x00000028,
	.grp_b1ds = 0x00000028,
	.grp_ctlds = 0x00000020,
	.grp_ddr_type = 0x000c0000,
	.grp_b2ds = 0x00000028,
	.grp_b3ds = 0x00000028,
};

const struct mx6_mmdc_calibration mx6_mmcd_calib = {
	.p0_mpwldectrl0 = 0x00290025,
	.p0_mpwldectrl1 = 0x00220022,
	.p0_mpdgctrl0 = 0x41480144,
	.p0_mpdgctrl1 = 0x01340130,
	.p0_mprddlctl = 0x3C3E4244,
	.p0_mpwrdlctl = 0x34363638,
};

static struct mx6_ddr3_cfg mem_ddr = {
	.mem_speed = 1600,
	.density = 4,
	.width = 32,
	.banks = 8,
	.rowaddr = 15,
	.coladdr = 10,
	.pagesz = 2,
	.trcd = 1375,
	.trcmin = 4875,
	.trasmin = 3500,
};

static void ccgr_init(void)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	writel(0xFFFFFFFF, &ccm->CCGR0);
	writel(0xFFFFFFFF, &ccm->CCGR1);
	writel(0xFFFFFFFF, &ccm->CCGR2);
	writel(0xFFFFFFFF, &ccm->CCGR3);
	writel(0xFFFFFFFF, &ccm->CCGR4);
	writel(0xFFFFFFFF, &ccm->CCGR5);
	writel(0xFFFFFFFF, &ccm->CCGR6);
	writel(0xFFFFFFFF, &ccm->CCGR7);
}

static void spl_dram_init(void)
{
	struct mx6_ddr_sysinfo sysinfo = {
		.dsize = mem_ddr.width/32,
		.cs_density = 24,
		.ncs = 1,
		.cs1_mirror = 0,
		.rtt_wr = 2,
		.rtt_nom = 2,		/* RTT_Nom = RZQ/2 */
		.walat = 1,		/* Write additional latency */
		.ralat = 5,		/* Read additional latency */
		.mif3_mode = 3,		/* Command prediction working mode */
		.bi_on = 1,		/* Bank interleaving enabled */
		.sde_to_rst = 0x10,	/* 14 cycles, 200us (JEDEC default) */
		.rst_to_cke = 0x23,	/* 33 cycles, 500us (JEDEC default) */
		.ddr_type = DDR_TYPE_DDR3,
		.refsel = 1,	/* Refresh cycles at 32KHz */
		.refr = 7,	/* 8 refresh commands per refresh cycle */
	};

	mx6sx_dram_iocfg(mem_ddr.width, &mx6_ddr_ioregs, &mx6_grp_ioregs);
	mx6_dram_cfg(&sysinfo, &mx6_mmcd_calib, &mem_ddr);
}

void board_init_f(ulong dummy)
{
	/* setup AIPS and disable watchdog */
	arch_cpu_init();

	ccgr_init();

	/* iomux and setup of i2c */
	board_early_init_f();

	/* setup GP timer */
	timer_init();

	/* UART clocks enabled and gd valid - init serial console */
	preloader_console_init();

	/* DDR initialization */
	spl_dram_init();

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	/* load/boot image from boot device */
	board_init_r(NULL, 0);
}
#endif
