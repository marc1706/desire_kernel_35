/* arch/arm/mach-msm/board-bravoc.h
 *
 * Copyright (C) 2009 HTC Corporation.
 * Author: Haley Teng <Haley_Teng@htc.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/

#ifndef __ARCH_ARM_MACH_MSM_BOARD_BRAVOC_H
#define __ARCH_ARM_MACH_MSM_BOARD_BRAVOC_H

#include <mach/board.h>

#define MSM_SMI_BASE		0x02B00000
#define MSM_SMI_SIZE		0x01500000


#define MSM_PMEM_VENC_BASE	0x02B00000
#define MSM_PMEM_VENC_SIZE	0x00800000

#define MSM_GPU_MEM_BASE	0x03300000
#define MSM_GPU_MEM_SIZE	0x00300000

#define MSM_RAM_CONSOLE_BASE	0x03A00000
#define MSM_RAM_CONSOLE_SIZE	0x00040000

#define MSM_FB_BASE		0x03B00000
#define MSM_FB_SIZE		0x00300000

#define MSM_EBI1_BANK0_BASE	0x20000000
#define MSM_EBI1_BANK0_SIZE	0x0E800000

#define MSM_EBI1_BANK1_BASE 	0x30000000
#define MSM_EBI1_BANK1_SIZE	0x0C000000

#define MSM_PMEM_MDP_BASE	0x3C000000
#define MSM_PMEM_MDP_SIZE	0x02000000

#define MSM_PMEM_ADSP_BASE	0x3E000000
#define MSM_PMEM_ADSP_SIZE	0x02000000

#define	BRAVOC_GPIO_PS_HOLD		25

#define BRAVOC_GPIO_UP_INT_N		35
#define BRAVOC_GPIO_UP_RESET_N	82
#define BRAVOC_GPIO_LS_EN		119

#define BRAVOC_GPIO_TP_INT_N		92
#define BRAVOC_GPIO_TP_LS_EN		93
#define BRAVOC_GPIO_TP_EN		160

#define BRAVOC_GPIO_POWER_KEY		94

#define BRAVOC_GPIO_RESET_BTN		27
#define BRAVOC_GPIO_WIFI_SHUTDOWN_N	127
#define BRAVOC_GPIO_WIFI_IRQ		152

#define BRAVOC_GPIO_JOGBALL_POWER	150

#define BRAVOC_GPIO_KP_MPIN0		42
#define BRAVOC_GPIO_KP_MPIN1		41
#define BRAVOC_GPIO_KP_MPIN2		40
#define BRAVOC_GPIO_KP_MKOUT0		33
#define BRAVOC_GPIO_KP_MKOUT1		32
#define BRAVOC_GPIO_KP_MKOUT2		31

/* Bluetooth */
#define BRAVOC_GPIO_BT_UART1_RTS	43
#define BRAVOC_GPIO_BT_UART1_CTS	44
#define BRAVOC_GPIO_BT_UART1_RX	45
#define BRAVOC_GPIO_BT_UART1_TX	46
#define BRAVOC_GPIO_BT_RESET_N	146
#define BRAVOC_GPIO_BT_SHUTDOWN_N	128
#define BRAVOC_GPIO_BT_HOST_WAKE	86
#define BRAVOC_GPIO_BT_CHIP_WAKE	28

#define BRAVOC_GPIO_UART3_RX 139
#define BRAVOC_GPIO_UART3_TX 140

#define BRAVOC_GPIO_COMPASS_RST_N	107
#define BRAVOC_GPIO_COMPASS_INT_N	153
#define BRAVOC_PROJECT_NAME		"bravoc"
#define BRAVOC_LAYOUTS		{ \
		{ {-1,  0, 0}, { 0, -1,  0}, {0, 0,  1} }, /*cspec_Hlayout*/ \
		{ { 0, -1, 0}, { 1,  0,  0}, {0, 0, -1} }, /*cspec_Alayout*/ \
		{ { 0, -1, 0}, { 1,  0,  0}, {0, 0,  1} }, /*m_Hlayout*/ \
		{ {-1,  0, 0}, { 0,  0, -1}, {0, 1,  0} }  /*m_Alayout*/ \
					}

/* Proximity */
#define BRAVOC_GPIO_PROXIMITY_INT	90
#define BRAVOC_GPIO_PROXIMITY_EN	120

/* Battery */
#define BRAVOC_GPIO_MBAT_IN		39
#define BRAVOC_GPIO_MCHG_EN_N		22

#define BRAVOC_GPIO_DQ_PWRDN_N		87

/*Audio */
#define BRAVOC_AUD_JACKHP_EN		157
#define BRAVOC_AUD_2V5_EN		158

#define BRAVOC_AUD_SPK_EN		104


/* Bluetooth PCM */
#define BRAVOC_BT_PCM_OUT		68
#define BRAVOC_BT_PCM_IN		69
#define BRAVOC_BT_PCM_SYNC		70
#define BRAVOC_BT_PCM_CLK		71

#define BRAVOC_VIB_3V3_EN		89

/* flash light */
#define BRAVOC_GPIO_FLASHLIGHT_TORCH (26)
#define BRAVOC_GPIO_FLASHLIGHT_FLASH (84)

/* LCM */
#define BRAVOC_LCD_SPI_CLK		(17)
#define BRAVOC_LCD_SPI_CSz		(20)
#define BRAVOC_LCD_RSTz			(29)
#define BRAVOC_LCD_R0			(113) /* XB new added */
#define BRAVOC_LCD_R1			(114)
#define BRAVOC_LCD_R2			(115)
#define BRAVOC_LCD_R3			(116)
#define BRAVOC_LCD_R4			(117)
#define BRAVOC_LCD_R5			(118)
#define BRAVOC_LCD_G0			(121)
#define BRAVOC_LCD_G1			(122)
#define BRAVOC_LCD_G2			(123)
#define BRAVOC_LCD_G3			(124)
#define BRAVOC_LCD_G4			(125)
#define BRAVOC_LCD_G5			(126)
#define BRAVOC_LCD_B0			(129) /* XB new added*/
#define BRAVOC_LCD_B1			(130)
#define BRAVOC_LCD_B2			(131)
#define BRAVOC_LCD_B3			(132)
#define BRAVOC_LCD_B4			(133)
#define BRAVOC_LCD_B5			(134)
#define BRAVOC_LCD_PCLK			(135)
#define BRAVOC_LCD_VSYNC		(136)
#define BRAVOC_LCD_HSYNC		(137)
#define BRAVOC_LCD_DE			(138)
#define BRAVOC_LCD_ID0			(147)

#define BRAVOC_GPIO_USB_ID_PIN          (142)
#define BRAVOC_GPIO_USB_ID1_PIN          (36)
#define BRAVOC_USB_PHY_3V3_ENABLE		(161)

unsigned int bravoc_get_engineerid(void);

#endif /* __ARCH_ARM_MACH_MSM_BOARD_BRAVOC_H */
