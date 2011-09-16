/* arch/arm/mach-msm/board-bravo.h
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

#ifndef __ARCH_ARM_MACH_MSM_BOARD_BRAVO_H
#define __ARCH_ARM_MACH_MSM_BOARD_BRAVO_H

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
#define MSM_EBI1_BANK1_SIZE	0x03600000

#define MSM_PMEM_MDP_BASE	0x33600000
#define MSM_PMEM_MDP_SIZE	0x02000000

#define MSM_PMEM_ADSP_BASE	0x35600000
#define MSM_PMEM_ADSP_SIZE	0x02A00000

#define MSM_MEM_128MB_OFFSET	0x08000000

/* PCB ID */
#define BRAVO_PCBID_EVT1		0
#define BRAVO_PCBID_EVT2		1

#define BRAVO_GPIO_PS_HOLD		25

#define BRAVO_GPIO_UP_INT_N		35
#define BRAVO_GPIO_UP_RESET_N	82
#define BRAVO_GPIO_LS_EN			119

#define BRAVO_GPIO_TP_INT_N		92
#define BRAVO_GPIO_TP_LS_EN		93
#define BRAVO_GPIO_TP_EN		160

#define BRAVO_GPIO_POWER_KEY		94
#define BRAVO_GPIO_SDMC_CD_N		153

#define BRAVO_GPIO_WIFI_SHUTDOWN_N	127
#define BRAVO_GPIO_WIFI_IRQ		152
#define BRAVO_GPIO_BALL_UP		38
#define BRAVO_GPIO_BALL_DOWN		37
#define BRAVO_GPIO_BALL_LEFT		145
#define BRAVO_GPIO_BALL_RIGHT	21

/* BT */
#define BRAVO_GPIO_BT_UART1_RTS	43
#define BRAVO_GPIO_BT_UART1_CTS	44
#define BRAVO_GPIO_BT_UART1_RX	45
#define BRAVO_GPIO_BT_UART1_TX	46
#define BRAVO_GPIO_BT_RESET_N	146
#define BRAVO_GPIO_BT_SHUTDOWN_N	128
#define BRAVO_GPIO_BT_HOST_WAKE	86
#define BRAVO_GPIO_BT_CHIP_WAKE	57

/* serial debugger */
#define BRAVO_GPIO_UART3_RX 139
#define BRAVO_GPIO_UART3_TX 140

#define BRAVO_GPIO_COMPASS_RST_N	107
#define BRAVO_GPIO_COMPASS_INT		153
#define BRAVO_PROJECT_NAME		"bravo"
#define BRAVO_LAYOUTS			{ \
		{ {-1,  0, 0}, { 0, -1,  0}, {0, 0,  1} }, \
		{ { 0, -1, 0}, { 1,  0,  0}, {0, 0, -1} }, \
		{ { 0, -1, 0}, { 1,  0,  0}, {0, 0,  1} }, \
		{ {-1,  0, 0}, { 0,  0, -1}, {0, 1,  0} }  \
					}
/* Proximity */
#define BRAVO_GPIO_PROXIMITY_EN		120
#define BRAVO_GPIO_PROXIMITY_INT_N	90

/* Battery */
#define BRAVO_GPIO_MBAT_IN		39
#define BRAVO_GPIO_MCHG_EN_N		22
#define BRAVO_GPIO_ISET			16
#define BRAVO_GPIO_DQ_PWRDN_N	87

/*Audio */
#define BRAVO_AUD_JACKHP_EN		157
#define BRAVO_AUD_2V5_EN		158

/* Bluetooth PCM */
#define BRAVO_BT_PCM_OUT		68
#define BRAVO_BT_PCM_IN		69
#define BRAVO_BT_PCM_SYNC		70
#define BRAVO_BT_PCM_CLK		71

#define BRAVO_VIB_3V3_EN		89

/* flash light */
#define BRAVO_GPIO_FLASHLIGHT_TORCH (58)
#define BRAVO_GPIO_FLASHLIGHT_FLASH (84)

/* keypad */
#define BRAVO_GPIO_KP_MKOUT0		33
#define BRAVO_GPIO_KP_MKOUT1		32
#define BRAVO_GPIO_KP_MKOUT2		31
#define BRAVO_GPIO_KP_MPIN0		42
#define BRAVO_GPIO_KP_MPIN1		41
#define BRAVO_GPIO_KP_MPIN2		40

/* 3.5mm remote control key interrupt shutdown signal */
#define BRAVO_GPIO_35MM_KEY_INT_SHUTDOWN	19

#define BRAVO_GPIO_USB_ID_PIN			142/*for BRAVO USB ID pin*/
#define BRAVO_GPIO_USB_ID1_PIN		36/*for BRAVO USB ID pin*/
#define BRAVO_USB_PHY_3V3_ENABLE		161 /*for BRAVO USB 3V3*/

/* display relative */
#define BRAVO_LCD_SPI_CLK            (17)
#define BRAVO_LCD_SPI_DO             (18)
#define BRAVO_LCD_SPI_CSz            (20)
#define BRAVO_LCD_RSTz               (29)
#define BRAVO_LCD_R1                 (114)
#define BRAVO_LCD_R2                 (115)
#define BRAVO_LCD_R3                 (116)
#define BRAVO_LCD_R4                 (117)
#define BRAVO_LCD_R5                 (118)
#define BRAVO_LCD_G0                 (121)
#define BRAVO_LCD_G1                 (122)
#define BRAVO_LCD_G2                 (123)
#define BRAVO_LCD_G3                 (124)
#define BRAVO_LCD_G4                 (125)
#define BRAVO_LCD_G5                 (126)
#define BRAVO_LCD_B1                 (130)
#define BRAVO_LCD_B2                 (131)
#define BRAVO_LCD_B3                 (132)
#define BRAVO_LCD_B4                 (133)
#define BRAVO_LCD_B5                 (134)
#define BRAVO_LCD_PCLK               (135)
#define BRAVO_LCD_VSYNC              (136)
#define BRAVO_LCD_HSYNC              (137)
#define BRAVO_LCD_DE                 (138)
#define BRAVO_LCD_ID0		     (147)

#endif /* __ARCH_ARM_MACH_MSM_BOARD_BRAVO_H */
