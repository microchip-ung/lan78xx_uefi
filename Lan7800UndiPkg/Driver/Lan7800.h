/*

    Copyright (C) 2009 - 2020. Microchip Technology Inc. and its 
            subsidiaries (Microchip).  All rights reserved.

    You are permitted to use the software and its derivatives with Microchip 
    products. See the license agreement accompanying this software, if any, 
    for additional info regarding your rights and obligations.

    SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY 
    KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY 
    WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A 
    PARTICULAR PURPOSE. IN NO EVENT SHALL MICROCHIP, SMSC, OR ITS LICENSORS 
    BE LIABLE OR OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, 
    CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY FOR 
    ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO 
    ANY INCIDENTAL, SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES, OR OTHER 
    SIMILAR COSTS. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP AND ITS 
    LICENSORS LIABILITY WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY, THAT YOU 
    PAID DIRECTLY TO MICROCHIP TO USE THIS SOFTWARE. MICROCHIP PROVIDES THIS 
    SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE TERMS.  
*/

#ifndef _LAN7800_H_
#define _LAN7800_H_

// Tx Command A
#define TX_CMD_A_IGE_                   (0x20000000)
#define TX_CMD_A_ICE_                   (0x10000000)
#define TX_CMD_A_LSO_                   (0x08000000)
#define TX_CMD_A_IPE_                   (0x04000000)
#define TX_CMD_A_TPE_                   (0x02000000)
#define TX_CMD_A_IVTG_                  (0x01000000)
#define TX_CMD_A_RVTG_                  (0x00800000)
#define TX_CMD_A_FCS_                   (0x00400000)
#define TX_CMD_A_LEN_MASK_              (0x000FFFFF)

// Tx Command B
#define TX_CMD_B_MSS_SHIFT_             (16)
#define TX_CMD_B_MSS_MASK_              (0x3FFF0000)
#define TX_CMD_B_MSS_MIN_               ((unsigned short)8)
#define TX_CMD_B_VTAG_MASK_             (0x0000FFFF)
#define TX_CMD_B_VTAG_PRI_MASK_         (0x0000E000)
#define TX_CMD_B_VTAG_CFI_MASK_         (0x00001000)
#define TX_CMD_B_VTAG_VID_MASK_         (0x00000FFF)

// Rx Command A
#define RX_CMD_A_ICE_                   (0x80000000)
#define RX_CMD_A_TCE_                   (0x40000000)
#define RX_CMD_A_CSE_MASK_              (0xC0000000)
#define RX_CMD_A_IPV_                   (0x20000000)
#define RX_CMD_A_PID_MASK_              (0x18000000)
#define RX_CMD_A_PID_NONE_IP_           (0x00000000)
#define RX_CMD_A_PID_TCP_IP_            (0x08000000)
#define RX_CMD_A_PID_UDP_IP_            (0x10000000)
#define RX_CMD_A_PID_IP_                (0x18000000)
#define RX_CMD_A_PFF_                   (0x04000000)
#define RX_CMD_A_BAM_                   (0x02000000)
#define RX_CMD_A_MAM_                   (0x01000000)
#define RX_CMD_A_FVTG_                  (0x00800000)
#define RX_CMD_A_RED_                   (0x00400000)
#define RX_CMD_A_RX_ERRS_MASK_          (0xC03F0000)
#define RX_CMD_A_RWT_                   (0x00200000)
#define RX_CMD_A_RUNT_                  (0x00100000)
#define RX_CMD_A_LONG_                  (0x00080000)
#define RX_CMD_A_RXE_                   (0x00040000)
#define RX_CMD_A_DRB_                   (0x00020000)
#define RX_CMD_A_FCS_                   (0x00010000)
#define RX_CMD_A_UAM_                   (0x00008000)
#define RX_CMD_A_ICSM_                  (0x00004000)
#define RX_CMD_A_LEN_MASK_              (0x00003FFF)

// Rx Command B
#define RX_CMD_B_CSUM_SHIFT_            (16)
#define RX_CMD_B_CSUM_MASK_             (0xFFFF0000)
#define RX_CMD_B_VTAG_MASK_             (0x0000FFFF)
#define RX_CMD_B_VTAG_PRI_MASK_         (0x0000E000)
#define RX_CMD_B_VTAG_CFI_MASK_         (0x00001000)
#define RX_CMD_B_VTAG_VID_MASK_         (0x00000FFF)

// Rx Command C
#define RX_CMD_C_WAKE_SHIFT_            (15)
#define RX_CMD_C_WAKE_                  (0x8000)
#define RX_CMD_C_REF_FAIL_SHIFT_        (14)
#define RX_CMD_C_REF_FAIL_              (0x4000)

// Flow FCT macros
#define FLOW_THRESHOLD(n)              ((((n) + 511) / 512) & 0x7F)
#define FLOW_CTRL_THRESHOLD(on, off)   ((FLOW_THRESHOLD(on)  << 0) | \
                                        (FLOW_THRESHOLD(off) << 8))
/* Flow control turned on when Rx FIFO level rises above this level (bytes) */
#define FLOW_ON_SS                     9216
#define FLOW_ON_HS                     8704

/* Flow control turned off when Rx FIFO level falls below this level (bytes) */
#define FLOW_OFF_SS                    4096
#define FLOW_OFF_HS                    1024

/*
 ****************************************************************************
 *    SCSRs
 ****************************************************************************
 */
#define NUMBER_OF_REGS                  (193)

#define ID_REV                          (0x00)
#define ID_REV_CHIP_ID_MASK_            (0xFFFF0000)
#define ID_REV_CHIP_REV_MASK_           (0x0000FFFF)
#define ID_REV_CHIP_ID_7800_            (0x7800)

#define FPGA_REV                        (0x04)
#define FPGA_REV_MINOR_MASK_            (0x0000FF00)
#define FPGA_REV_MAJOR_MASK_            (0x000000FF)

#define INT_STS                         (0x0C)
#define INT_STS_CLEAR_ALL_              (0xFFFFFFFF)
#define INT_STS_EEE_TX_LPI_STRT_        (0x04000000)
#define INT_STS_EEE_TX_LPI_STOP_        (0x02000000)
#define INT_STS_EEE_RX_LPI_             (0x01000000)
#define INT_STS_RDFO_                   (0x00400000)
#define INT_STS_TXE_                    (0x00200000)
#define INT_STS_TX_DIS_                 (0x00080000)
#define INT_STS_RX_DIS_                 (0x00040000)
#define INT_STS_PHY_INT_                (0x00020000)
#define INT_STS_DP_INT_                 (0x00010000)
#define INT_STS_MAC_ERR_                (0x00008000)
#define INT_STS_TDFU_                   (0x00004000)
#define INT_STS_TDFO_                   (0x00002000)
#define INT_STS_UFX_FP_                 (0x00001000)
#define INT_STS_GPIO_MASK_              (0x00000FFF)
#define INT_STS_GPIO11_                 (0x00000800)
#define INT_STS_GPIO10_                 (0x00000400)
#define INT_STS_GPIO9_                  (0x00000200)
#define INT_STS_GPIO8_                  (0x00000100)
#define INT_STS_GPIO7_                  (0x00000080)
#define INT_STS_GPIO6_                  (0x00000040)
#define INT_STS_GPIO5_                  (0x00000020)
#define INT_STS_GPIO4_                  (0x00000010)
#define INT_STS_GPIO3_                  (0x00000008)
#define INT_STS_GPIO2_                  (0x00000004)
#define INT_STS_GPIO1_                  (0x00000002)
#define INT_STS_GPIO0_                  (0x00000001)

#define HW_CFG                          (0x010)
#define HW_CFG_CLK125_EN_               (0x02000000)
#define HW_CFG_REFCLK25_EN_             (0x01000000)
#define HW_CFG_LED3_EN_                 (0x00800000)
#define HW_CFG_LED2_EN_                 (0x00400000)
#define HW_CFG_LED1_EN_                 (0x00200000)
#define HW_CFG_LED0_EN_                 (0x00100000)
#define HW_CFG_EEE_PHY_LUSU_            (0x00020000)
#define HW_CFG_EEE_TSU_                 (0x00010000)
#define HW_CFG_NETDET_STS_              (0x00008000)
#define HW_CFG_NETDET_EN_               (0x00004000)
#define HW_CFG_EEM_                     (0x00002000)
#define HW_CFG_RST_PROTECT_             (0x00001000)
#define HW_CFG_CONNECT_BUF_             (0x00000400)
#define HW_CFG_CONNECT_EN_              (0x00000200)
#define HW_CFG_CONNECT_POL_             (0x00000100)
#define HW_CFG_SUSPEND_N_SEL_MASK_      (0x000000C0)
#define HW_CFG_SUSPEND_N_SEL_2          (0x00000000)
#define HW_CFG_SUSPEND_N_SEL_12N        (0x00000040)
#define HW_CFG_SUSPEND_N_SEL_012N       (0x00000080)
#define HW_CFG_SUSPEND_N_SEL_0123N      (0x000000C0)
#define HW_CFG_SUSPEND_N_POL_           (0x00000020)
#define HW_CFG_MEF_                     (0x00000010)
#define HW_CFG_ETC_                     (0x00000008)
#define HW_CFG_LRST_                    (0x00000002)
#define HW_CFG_SRST_                    (0x00000001)

#define PMT_CTL                         (0x014)
#define PMT_CTL_EEE_WAKEUP_EN_          (0x00002000)
#define PMT_CTL_EEE_WUPS_               (0x00001000)
#define PMT_CTL_MAC_SRST_               (0x00000800)
#define PMT_CTL_PHY_PWRUP_              (0x00000400)
#define PMT_CTL_RES_CLR_WKP_MASK_       (0x00000300)
#define PMT_CTL_RES_CLR_WKP_STS_        (0x00000200)
#define PMT_CTL_RES_CLR_WKP_EN_         (0x00000100)
#define PMT_CTL_READY_                  (0x00000080)
#define PMT_CTL_SUS_MODE_MASK_          (0x00000060)
#define PMT_CTL_SUS_MODE_0_             (0x00000000)
#define PMT_CTL_SUS_MODE_1_             (0x00000020)
#define PMT_CTL_SUS_MODE_2_             (0x00000040)
#define PMT_CTL_SUS_MODE_3_             (0x00000060)
#define PMT_CTL_PHY_RST_                (0x00000010)
#define PMT_CTL_WOL_EN_                 (0x00000008)
#define PMT_CTL_PHY_WAKE_EN_            (0x00000004)
#define PMT_CTL_WUPS_MASK_              (0x00000003)
#define PMT_CTL_WUPS_MLT_               (0x00000003)
#define PMT_CTL_WUPS_MAC_               (0x00000002)
#define PMT_CTL_WUPS_PHY_               (0x00000001)

#define GPIO_CFG0                       (0x018)
#define GPIO_CFG0_GPIOEN_MASK_          (0x0000F000)
#define GPIO_CFG0_GPIOEN3_              (0x00008000)
#define GPIO_CFG0_GPIOEN2_              (0x00004000)
#define GPIO_CFG0_GPIOEN1_              (0x00002000)
#define GPIO_CFG0_GPIOEN0_              (0x00001000)
#define GPIO_CFG0_GPIOBUF_MASK_         (0x00000F00)
#define GPIO_CFG0_GPIOBUF3_             (0x00000800)
#define GPIO_CFG0_GPIOBUF2_             (0x00000400)
#define GPIO_CFG0_GPIOBUF1_             (0x00000200)
#define GPIO_CFG0_GPIOBUF0_             (0x00000100)
#define GPIO_CFG0_GPIODIR_MASK_         (0x000000F0)
#define GPIO_CFG0_GPIODIR3_             (0x00000080)
#define GPIO_CFG0_GPIODIR2_             (0x00000040)
#define GPIO_CFG0_GPIODIR1_             (0x00000020)
#define GPIO_CFG0_GPIODIR0_             (0x00000010)
#define GPIO_CFG0_GPIOD_MASK_           (0x0000000F)
#define GPIO_CFG0_GPIOD3_               (0x00000008)
#define GPIO_CFG0_GPIOD2_               (0x00000004)
#define GPIO_CFG0_GPIOD1_               (0x00000002)
#define GPIO_CFG0_GPIOD0_               (0x00000001)

#define GPIO_CFG1                       (0x01C)
#define GPIO_CFG1_GPIOEN_MASK_          (0xFF000000)
#define GPIO_CFG1_GPIOEN11_             (0x80000000)
#define GPIO_CFG1_GPIOEN10_             (0x40000000)
#define GPIO_CFG1_GPIOEN9_              (0x20000000)
#define GPIO_CFG1_GPIOEN8_              (0x10000000)
#define GPIO_CFG1_GPIOEN7_              (0x08000000)
#define GPIO_CFG1_GPIOEN6_              (0x04000000)
#define GPIO_CFG1_GPIOEN5_              (0x02000000)
#define GPIO_CFG1_GPIOEN4_              (0x01000000)
#define GPIO_CFG1_GPIOBUF_MASK_         (0x00FF0000)
#define GPIO_CFG1_GPIOBUF11_            (0x00800000)
#define GPIO_CFG1_GPIOBUF10_            (0x00400000)
#define GPIO_CFG1_GPIOBUF9_             (0x00200000)
#define GPIO_CFG1_GPIOBUF8_             (0x00100000)
#define GPIO_CFG1_GPIOBUF7_             (0x00080000)
#define GPIO_CFG1_GPIOBUF6_             (0x00040000)
#define GPIO_CFG1_GPIOBUF5_             (0x00020000)
#define GPIO_CFG1_GPIOBUF4_             (0x00010000)
#define GPIO_CFG1_GPIODIR_MASK_         (0x0000FF00)
#define GPIO_CFG1_GPIODIR11_            (0x00008000)
#define GPIO_CFG1_GPIODIR10_            (0x00004000)
#define GPIO_CFG1_GPIODIR9_             (0x00002000)
#define GPIO_CFG1_GPIODIR8_             (0x00001000)
#define GPIO_CFG1_GPIODIR7_             (0x00000800)
#define GPIO_CFG1_GPIODIR6_             (0x00000400)
#define GPIO_CFG1_GPIODIR5_             (0x00000200)
#define GPIO_CFG1_GPIODIR4_             (0x00000100)
#define GPIO_CFG1_GPIOD_MASK_           (0x000000FF)
#define GPIO_CFG1_GPIOD11_              (0x00000080)
#define GPIO_CFG1_GPIOD10_              (0x00000040)
#define GPIO_CFG1_GPIOD9_               (0x00000020)
#define GPIO_CFG1_GPIOD8_               (0x00000010)
#define GPIO_CFG1_GPIOD7_               (0x00000008)
#define GPIO_CFG1_GPIOD6_               (0x00000004)
#define GPIO_CFG1_GPIOD6_               (0x00000004)
#define GPIO_CFG1_GPIOD5_               (0x00000002)
#define GPIO_CFG1_GPIOD4_               (0x00000001)

#define GPIO_WAKE                       (0x020)
#define GPIO_WAKE_GPIOPOL_MASK_         (0x0FFF0000)
#define GPIO_WAKE_GPIOPOL11_            (0x08000000)
#define GPIO_WAKE_GPIOPOL10_            (0x04000000)
#define GPIO_WAKE_GPIOPOL9_             (0x02000000)
#define GPIO_WAKE_GPIOPOL8_             (0x01000000)
#define GPIO_WAKE_GPIOPOL7_             (0x00800000)
#define GPIO_WAKE_GPIOPOL6_             (0x00400000)
#define GPIO_WAKE_GPIOPOL5_             (0x00200000)
#define GPIO_WAKE_GPIOPOL4_             (0x00100000)
#define GPIO_WAKE_GPIOPOL3_             (0x00080000)
#define GPIO_WAKE_GPIOPOL2_             (0x00040000)
#define GPIO_WAKE_GPIOPOL1_             (0x00020000)
#define GPIO_WAKE_GPIOPOL0_             (0x00010000)
#define GPIO_WAKE_GPIOWK_MASK_          (0x00000FFF)
#define GPIO_WAKE_GPIOWK11_             (0x00000800)
#define GPIO_WAKE_GPIOWK10_             (0x00000400)
#define GPIO_WAKE_GPIOWK9_              (0x00000200)
#define GPIO_WAKE_GPIOWK8_              (0x00000100)
#define GPIO_WAKE_GPIOWK7_              (0x00000080)
#define GPIO_WAKE_GPIOWK6_              (0x00000040)
#define GPIO_WAKE_GPIOWK5_              (0x00000020)
#define GPIO_WAKE_GPIOWK4_              (0x00000010)
#define GPIO_WAKE_GPIOWK3_              (0x00000008)
#define GPIO_WAKE_GPIOWK2_              (0x00000004)
#define GPIO_WAKE_GPIOWK1_              (0x00000002)
#define GPIO_WAKE_GPIOWK0_              (0x00000001)

#define DP_SEL                          (0x024)
#define DP_SEL_DPRDY_                   (0x80000000)
#define DP_SEL_RSEL_MASK_               (0x0000000F)
#define DP_SEL_RSEL_USB_PHY_CSRS_       (0x0000000F)
#define DP_SEL_RSEL_OTP_64BIT_          (0x00000009)
#define DP_SEL_RSEL_OTP_8BIT_           (0x00000008)
#define DP_SEL_RSEL_UTX_BUF_RAM_        (0x00000007)
#define DP_SEL_RSEL_DESC_RAM_           (0x00000005)
#define DP_SEL_RSEL_TXFIFO_             (0x00000004)
#define DP_SEL_RSEL_RXFIFO_             (0x00000003)
#define DP_SEL_RSEL_LSO_                (0x00000002)
#define DP_SEL_RSEL_VLAN_DA_            (0x00000001)
#define DP_SEL_RSEL_URXBUF_             (0x00000000)
/*

/*
 * FIXME
 * Check value
 */
#define DP_SEL_VHF_HASH_LEN             (16)
#define DP_SEL_VHF_VLAN_LEN             (128)

#define DP_CMD                          (0x028)
#define DP_CMD_WRITE_                   (0x00000001)
#define DP_CMD_READ_                    (0x00000000)

#define DP_ADDR                         (0x02C)
#define DP_ADDR_MASK_                   (0x00003FFF)

#define DP_DATA                         (0x030)

#define E2P_CMD                         (0x040)
#define E2P_CMD_EPC_BUSY_               (0x80000000)
#define E2P_CMD_EPC_CMD_MASK_           (0x70000000)
#define E2P_CMD_EPC_CMD_RELOAD_         (0x70000000)
#define E2P_CMD_EPC_CMD_ERAL_           (0x60000000)
#define E2P_CMD_EPC_CMD_ERASE_          (0x50000000)
#define E2P_CMD_EPC_CMD_WRAL_           (0x40000000)
#define E2P_CMD_EPC_CMD_WRITE_          (0x30000000)
#define E2P_CMD_EPC_CMD_EWEN_           (0x20000000)
#define E2P_CMD_EPC_CMD_EWDS_           (0x10000000)
#define E2P_CMD_EPC_CMD_READ_           (0x00000000)
#define E2P_CMD_EPC_TIMEOUT_            (0x00000400)
#define E2P_CMD_EPC_DL_                 (0x00000200)
#define E2P_CMD_EPC_ADDR_MASK_          (0x000001FF)

#define E2P_DATA                        (0x044)
#define E2P_DATA_EEPROM_DATA_MASK_      (0x000000FF)

#define BOS_ATTR                        (0x050)
#define BOS_ATTR_BLOCK_SIZE_MASK_       (0x000000FF)

#define SS_ATTR                         (0x054)
#define SS_ATTR_POLL_INT_MASK_          (0x00FF0000)
#define SS_ATTR_DEV_DESC_SIZE_MASK_     (0x0000FF00)
#define SS_ATTR_CFG_BLK_SIZE_MASK_      (0x000000FF)

#define HS_ATTR                         (0x058)
#define HS_ATTR_POLL_INT_MASK_          (0x00FF0000)
#define HS_ATTR_DEV_DESC_SIZE_MASK_     (0x0000FF00)
#define HS_ATTR_CFG_BLK_SIZE_MASK_      (0x000000FF)

#define FS_ATTR                         (0x05C)
#define FS_ATTR_POLL_INT_MASK_          (0x00FF0000)
#define FS_ATTR_DEV_DESC_SIZE_MASK_     (0x0000FF00)
#define FS_ATTR_CFG_BLK_SIZE_MASK_      (0x000000FF)

#define STR_ATTR0                           (0x060)
#define STR_ATTR0_CFGSTR_DESC_SIZE_MASK_    (0xFF000000)
#define STR_ATTR0_SERSTR_DESC_SIZE_MASK_    (0x00FF0000)
#define STR_ATTR0_PRODSTR_DESC_SIZE_MASK_   (0x0000FF00)
#define STR_ATTR0_MANUF_DESC_SIZE_MASK_     (0x000000FF)

#define STR_ATTR1                           (0x064)
#define STR_ATTR1_INTSTR_DESC_SIZE_MASK_    (0x000000FF)

#define STR_FLAG_ATTR                       (0x068)
#define STR_FLAG_ATTR_PME_FLAGS_MASK_       (0x000000FF)

#define USB_CFG0                        (0x080)
#define USB_CFG_LPM_RESPONSE_           (0x80000000)
#define USB_CFG_LPM_CAPABILITY_         (0x40000000)
#define USB_CFG_LPM_ENBL_SLPM_          (0x20000000)
#define USB_CFG_HIRD_THR_MASK_          (0x1F000000)
#define USB_CFG_HIRD_THR_960_           (0x1C000000)
#define USB_CFG_HIRD_THR_885_           (0x1B000000)
#define USB_CFG_HIRD_THR_810_           (0x1A000000)
#define USB_CFG_HIRD_THR_735_           (0x19000000)
#define USB_CFG_HIRD_THR_660_           (0x18000000)
#define USB_CFG_HIRD_THR_585_           (0x17000000)
#define USB_CFG_HIRD_THR_510_           (0x16000000)
#define USB_CFG_HIRD_THR_435_           (0x15000000)
#define USB_CFG_HIRD_THR_360_           (0x14000000)
#define USB_CFG_HIRD_THR_285_           (0x13000000)
#define USB_CFG_HIRD_THR_210_           (0x12000000)
#define USB_CFG_HIRD_THR_135_           (0x11000000)
#define USB_CFG_HIRD_THR_60_            (0x10000000)
#define USB_CFG_MAX_BURST_BI_MASK_      (0x00F00000)
#define USB_CFG_MAX_BURST_BO_MASK_      (0x000F0000)
#define USB_CFG_MAX_DEV_SPEED_MASK_     (0x0000E000)
#define USB_CFG_MAX_DEV_SPEED_SS_       (0x00008000)
#define USB_CFG_MAX_DEV_SPEED_HS_       (0x00000000)
#define USB_CFG_MAX_DEV_SPEED_FS_       (0x00002000)
#define USB_CFG_PHY_BOOST_MASK_         (0x00000180)
#define USB_CFG_PHY_BOOST_PLUS_12_      (0x00000180)
#define USB_CFG_PHY_BOOST_PLUS_8_       (0x00000100)
#define USB_CFG_PHY_BOOST_PLUS_4_       (0x00000080)
#define USB_CFG_PHY_BOOST_NORMAL_       (0x00000000)
#define USB_CFG_BIR_                    (0x00000040)
#define USB_CFG_BCE_                    (0x00000020)
#define USB_CFG_PORT_SWAP_              (0x00000010)
#define USB_CFG_LPM_EN_                 (0x00000008)
#define USB_CFG_RMT_WKP_                (0x00000004)
#define USB_CFG_PWR_SEL_                (0x00000002)
#define USB_CFG_STALL_BO_DIS_           (0x00000001)

#define USB_CFG1                        (0x084)
#define USB_CFG1_U1_TIMEOUT_MASK_       (0xFF000000)
#define USB_CFG1_U2_TIMEOUT_MASK_       (0x00FF0000)
#define USB_CFG1_HS_TOUT_CAL_MASK_      (0x0000E000)
#define USB_CFG1_DEV_U2_INIT_EN_        (0x00001000)
#define USB_CFG1_DEV_U2_EN_             (0x00000800)
#define USB_CFG1_DEV_U1_INIT_EN_        (0x00000400)
#define USB_CFG1_DEV_U1_EN_             (0x00000200)
#define USB_CFG1_LTM_ENABLE_            (0x00000100)
#define USB_CFG1_FS_TOUT_CAL_MASK_      (0x00000070)
#define USB_CFG1_SCALE_DOWN_MASK_       (0x00000003)
#define USB_CFG1_SCALE_DOWN_MODE3_      (0x00000003)
#define USB_CFG1_SCALE_DOWN_MODE2_      (0x00000002)
#define USB_CFG1_SCALE_DOWN_MODE1_      (0x00000001)
#define USB_CFG1_SCALE_DOWN_MODE0_      (0x00000000)

#define USB_CFG2                            (0x088)
#define USB_CFG2_SS_DETACH_TIME_MASK_       (0xFFFF0000)
#define USB_CFG2_HS_DETACH_TIME_MASK_       (0x0000FFFF)

#define BURST_CAP                       (0x090)
#define BURST_CAP_SIZE_MASK_            (0x000000FF)

#define BULK_IN_DLY                     (0x094)
#define BULK_IN_DLY_MASK_               (0x0000FFFF)

#define INT_EP_CTL                      (0x098)
#define INT_EP_INTEP_ON_                (0x80000000)
#define INT_STS_EEE_TX_LPI_STRT_EN_     (0x04000000)
#define INT_STS_EEE_TX_LPI_STOP_EN_     (0x02000000)
#define INT_STS_EEE_RX_LPI_EN_          (0x01000000)
#define INT_EP_RDFO_EN_                 (0x00400000)
#define INT_EP_TXE_EN_                  (0x00200000)
#define INT_EP_TX_DIS_EN_               (0x00080000)
#define INT_EP_RX_DIS_EN_               (0x00040000)
#define INT_EP_PHY_INT_EN_              (0x00020000)
#define INT_EP_DP_INT_EN_               (0x00010000)
#define INT_EP_MAC_ERR_EN_              (0x00008000)
#define INT_EP_TDFU_EN_                 (0x00004000)
#define INT_EP_TDFO_EN_                 (0x00002000)
#define INT_EP_UTX_FP_EN_               (0x00001000)
#define INT_EP_GPIO_EN_MASK_            (0x00000FFF)

#define PIPE_CTL                        (0x09C)
#define PIPE_CTL_TXSWING_               (0x00000040)
#define PIPE_CTL_TXMARGIN_MASK_         (0x00000038)
#define PIPE_CTL_TXDEEMPHASIS_MASK_     (0x00000006)
#define PIPE_CTL_ELASTICITYBUFFERMODE_  (0x00000001)

#define USB_STATUS                      (0x0A8)
#define USB_STATUS_REMOTE_WK_           (0x00100000)
#define USB_STATUS_FUNC_REMOTE_WK_      (0x00080000)
#define USB_STATUS_LTM_ENABLE_          (0x00040000)
#define USB_STATUS_U2_ENABLE_           (0x00020000)
#define USB_STATUS_U1_ENABLE_           (0x00010000)
#define USB_STATUS_SET_SEL_             (0x00000020)
#define USB_STATUS_REMOTE_WK_STS_       (0x00000010)
#define USB_STATUS_FUNC_REMOTE_WK_STS_  (0x00000008)
#define USB_STATUS_LTM_ENABLE_STS_      (0x00000004)
#define USB_STATUS_U2_ENABLE_STS_       (0x00000002)
#define USB_STATUS_U1_ENABLE_STS_       (0x00000001)

#define USB_CFG3                        (0x0AC)
#define USB_CFG3_EN_U2_LTM_             (0x40000000)
#define USB_CFG3_BULK_OUT_NUMP_OVR_     (0x20000000)
#define USB_CFG3_DIS_FAST_U1_EXIT_      (0x10000000)
#define USB_CFG3_LPM_NYET_THR_          (0x0F000000)
#define USB_CFG3_RX_DET_2_POL_LFPS_     (0x00800000)
#define USB_CFG3_LFPS_FILT_             (0x00400000)
#define USB_CFG3_SKIP_RX_DET_           (0x00200000)
#define USB_CFG3_DELAY_P1P2P3_          (0x001C0000)
#define USB_CFG3_DELAY_PHY_PWR_CHG_     (0x00020000)
#define USB_CFG3_U1U2_EXIT_FR_          (0x00010000)
#define USB_CFG3_REQ_P1P2P3             (0x00008000)
#define USB_CFG3_HST_PRT_CMPL_          (0x00004000)
#define USB_CFG3_DIS_SCRAMB_            (0x00002000)
#define USB_CFG3_PWR_DN_SCALE_          (0x00001FFF)

#define RFE_CTL                         (0x0B0)
#define RFE_CTL_IGMP_COE_               (0x00004000)
#define RFE_CTL_ICMP_COE_               (0x00002000)
#define RFE_CTL_TCPUDP_COE_             (0x00001000)
#define RFE_CTL_IP_COE_                 (0x00000800)
#define RFE_CTL_BCAST_EN_               (0x00000400)
#define RFE_CTL_MCAST_EN_               (0x00000200)
#define RFE_CTL_UCAST_EN_               (0x00000100)
#define RFE_CTL_VLAN_STRIP_             (0x00000080)
#define RFE_CTL_DISCARD_UNTAGGED_       (0x00000040)
#define RFE_CTL_VLAN_FILTER_            (0x00000020)
#define RFE_CTL_SA_FILTER_              (0x00000010)
#define RFE_CTL_MCAST_HASH_             (0x00000008)
#define RFE_CTL_DA_HASH_                (0x00000004)
#define RFE_CTL_DA_PERFECT_             (0x00000002)
#define RFE_CTL_RST_                    (0x00000001)

#define VLAN_TYPE                       (0x0B4)
#define VLAN_TYPE_MASK_                 (0x0000FFFF)

#define FCT_RX_CTL                      (0x0C0)
#define FCT_RX_CTL_EN_                  (0x80000000)
#define FCT_RX_CTL_RST_                 (0x40000000)
#define FCT_RX_CTL_SBF_                 (0x02000000)
#define FCT_RX_CTL_OVFL_                (0x01000000)
#define FCT_RX_CTL_DROP_                (0x00800000)
#define FCT_RX_CTL_NOT_EMPTY_           (0x00400000)
#define FCT_RX_CTL_EMPTY_               (0x00200000)
#define FCT_RX_CTL_DIS_                 (0x00100000)
#define FCT_RX_CTL_USED_MASK_           (0x0000FFFF)

#define FCT_TX_CTL                      (0x0C4)
#define FCT_TX_CTL_EN_                  (0x80000000)
#define FCT_TX_CTL_RST_                 (0x40000000)
#define FCT_TX_CTL_NOT_EMPTY_           (0x00400000)
#define FCT_TX_CTL_EMPTY_               (0x00200000)
#define FCT_TX_CTL_DIS_                 (0x00100000)
#define FCT_TX_CTL_USED_MASK_           (0x0000FFFF)

#define FCT_RX_FIFO_END                 (0x0C8)
#define FCT_RX_FIFO_END_MASK_           (0x0000007F)

#define FCT_TX_FIFO_END                 (0x0CC)
#define FCT_TX_FIFO_END_MASK_           (0x0000003F)

#define FCT_FLOW                        (0x0D0)
#define FCT_FLOW_OFF_MASK_              (0x00007F00)
#define FCT_FLOW_ON_MASK_               (0x0000007F)

#define RX_DP_STOR                      (0x0D4)
#define RX_DP_STORE_TOT_RXUSED_MASK_    (0xFFFF0000)
#define RX_DP_STORE_UTX_RXUSED_MASK_    (0x0000FFFF)

#define TX_DP_STOR                      (0x0D8)
#define TX_DP_STORE_TOT_TXUSED_MASK_    (0xFFFF0000)
#define TX_DP_STORE_URX_TXUSED_MASK_    (0x0000FFFF)

#define LTM_BELT_IDLE0                  (0x0E0)
#define LTM_BELT_IDLE0_IDLE1000_        (0x0FFF0000)
#define LTM_BELT_IDLE0_IDLE100_         (0x00000FFF)

#define LTM_BELT_IDLE1                  (0x0E4)
#define LTM_BELT_IDLE1_IDLE10_          (0x00000FFF)

#define LTM_BELT_ACT0                   (0x0E8)
#define LTM_BELT_ACT0_ACT1000_          (0x0FFF0000)
#define LTM_BELT_ACT0_ACT100_           (0x00000FFF)

#define LTM_BELT_ACT1                   (0x0EC)
#define LTM_BELT_ACT1_ACT10_            (0x00000FFF)

#define LTM_INACTIVE0                   (0x0F0)
#define LTM_INACTIVE0_TIMER1000_        (0xFFFF0000)
#define LTM_INACTIVE0_TIMER100_         (0x0000FFFF)

#define LTM_INACTIVE1                   (0x0F4)
#define LTM_INACTIVE1_TIMER10_          (0x0000FFFF)

#define MAC_CR                          (0x100)
#define MAC_CR_GMII_EN_                 (0x00080000)
#define MAC_CR_EEE_TX_CLK_STOP_EN_      (0x00040000)
#define MAC_CR_EEE_EN_                  (0x00020000)
#define MAC_CR_EEE_TLAR_EN_             (0x00010000)
#define MAC_CR_ADP_                     (0x00002000)
#define MAC_CR_AUTO_DUPLEX_             (0x00001000)
#define MAC_CR_AUTO_SPEED_              (0x00000800)
#define MAC_CR_LOOPBACK_                (0x00000400)
#define MAC_CR_BOLMT_MASK_              (0x000000C0)
#define MAC_CR_FULL_DUPLEX_             (0x00000008)
#define MAC_CR_SPEED_MASK_              (0x00000006)
#define MAC_CR_SPEED_1000_              (0x00000004)
#define MAC_CR_SPEED_100_               (0x00000002)
#define MAC_CR_SPEED_10_                (0x00000000)
#define MAC_CR_RST_                     (0x00000001)

#define MAC_RX                          (0x104)
#define MAC_RX_MAX_SIZE_SHIFT_          (16)
#define MAC_RX_MAX_SIZE_MASK_           (0x3FFF0000)
#define MAC_RX_FCS_STRIP_               (0x00000010)
#define MAC_RX_VLAN_FSE_                (0x00000004)
#define MAC_RX_RXD_                     (0x00000002)
#define MAC_RX_RXEN_                    (0x00000001)

#define MAC_TX                          (0x108)
#define MAC_TX_BAD_FCS_                 (0x00000004)
#define MAC_TX_TXD_                     (0x00000002)
#define MAC_TX_TXEN_                    (0x00000001)

#define FLOW                            (0x10C)
#define FLOW_CR_FORCE_FC_               (0x80000000)
#define FLOW_CR_TX_FCEN_                (0x40000000)
#define FLOW_CR_RX_FCEN_                (0x20000000)
#define FLOW_CR_FPF_                    (0x10000000)
#define FLOW_CR_FCPT_MASK_              (0x0000FFFF)

#define RAND_SEED                       (0x110)
#define RAND_SEED_MASK_                 (0x0000FFFF)

#define ERR_STS                         (0x114)
#define ERR_STS_FERR_                   (0x00000100)
#define ERR_STS_LERR_                   (0x00000080)
#define ERR_STS_RFERR_                  (0x00000040)
#define ERR_STS_ECERR_                  (0x00000010)
#define ERR_STS_ALERR_                  (0x00000008)
#define ERR_STS_URERR_                  (0x00000004)

#define RX_ADDRH                        (0x118)
#define RX_ADDRH_MASK_                  (0x0000FFFF)

#define RX_ADDRL                        (0x11C)
#define RX_ADDRL_MASK_                  (0xFFFFFFFF)

#define MII_ACC                         (0x120)
#define MII_ACC_PHY_ADDR_SHIFT_         (11)
#define MII_ACC_PHY_ADDR_MASK_          (0x0000F800)
#define MII_ACC_MIIRINDA_SHIFT_         (6)
#define MII_ACC_MIIRINDA_MASK_          (0x000007C0)
#define MII_ACC_MII_READ_               (0x00000000)
#define MII_ACC_MII_WRITE_              (0x00000002)
#define MII_ACC_MII_BUSY_               (0x00000001)

#define MII_DATA                        (0x124)
#define MII_DATA_MASK_                  (0x0000FFFF)

#define MAC_RGMII_ID                    (0x128)
#define MAC_RGMII_ID_TXC_DELAY_EN_      (0x00000002)
#define MAC_RGMII_ID_RXC_DELAY_EN_      (0x00000001)

#define EEE_TX_LPI_REQ_DLY              (0x130)
#define EEE_TX_LPI_REQ_DLY_CNT_MASK_    (0xFFFFFFFF)

#define EEE_TW_TX_SYS                   (0x134)
#define EEE_TW_TX_SYS_CNT1G_MASK_       (0xFFFF0000)
#define EEE_TW_TX_SYS_CNT100M_MASK_     (0x0000FFFF)

#define EEE_TX_LPI_REM_DLY              (0x138)
#define EEE_TX_LPI_REM_DLY_CNT_         (0x00FFFFFF)

#define WUCSR                           (0x140)
#define WUCSR_TESTMODE_                 (0x80000000)
#define WUCSR_RFE_WAKE_EN_              (0x00004000)
#define WUCSR_EEE_TX_WAKE_              (0x00002000)
#define WUCSR_EEE_TX_WAKE_EN_           (0x00001000)
#define WUCSR_EEE_RX_WAKE_              (0x00000800)
#define WUCSR_EEE_RX_WAKE_EN_           (0x00000400)
#define WUCSR_RFE_WAKE_FR_              (0x00000200)
#define WUCSR_STORE_WAKE_               (0x00000100)
#define WUCSR_PFDA_FR_                  (0x00000080)
#define WUCSR_WUFR_                     (0x00000040)
#define WUCSR_MPR_                      (0x00000020)
#define WUCSR_BCST_FR_                  (0x00000010)
#define WUCSR_PFDA_EN_                  (0x00000008)
#define WUCSR_WAKE_EN_                  (0x00000004)
#define WUCSR_MPEN_                     (0x00000002)
#define WUCSR_BCST_EN_                  (0x00000001)

#define WK_SRC                          (0x144)
#define WK_SRC_GPIOx_INT_WK_SHIFT_      (20)
#define WK_SRC_GPIOx_INT_WK_MASK_       (0xFFF00000)
#define WK_SRC_IPV6_TCPSYN_RCD_WK_      (0x00010000)
#define WK_SRC_IPV4_TCPSYN_RCD_WK_      (0x00008000)
#define WK_SRC_EEE_TX_WK_               (0x00004000)
#define WK_SRC_EEE_RX_WK_               (0x00002000)
#define WK_SRC_GOOD_FR_WK_              (0x00001000)
#define WK_SRC_PFDA_FR_WK_              (0x00000800)
#define WK_SRC_MP_FR_WK_                (0x00000400)
#define WK_SRC_BCAST_FR_WK_             (0x00000200)
#define WK_SRC_WU_FR_WK_                (0x00000100)
#define WK_SRC_WUFF_MATCH_MASK_         (0x0000001F)

#define WUF_CFG0                        (0x150)
#define NUM_OF_WUF_CFG                  (32)
#define WUF_CFG_BEGIN                   (WUF_CFG0)
#define WUF_CFG_END                     (WUF_CFG_BEGIN + (4*(NUM_OF_WUF_CFG-1)))
#define WUF_CFG(index)                  (WUF_CFG_BEGIN + (4*(index)))   // FIXME add sanity check
#define WUF_CFGx_EN_                    (0x80000000)
#define WUF_CFGx_TYPE_MASK_             (0x03000000)
#define WUF_CFGx_TYPE_MCAST_            (0x02000000)
#define WUF_CFGx_TYPE_ALL_              (0x01000000)
#define WUF_CFGx_TYPE_UCAST_            (0x00000000)
#define WUF_CFGx_OFFSET_SHIFT_          (16)
#define WUF_CFGx_OFFSET_MASK_           (0x00FF0000)
#define WUF_CFGx_CRC16_MASK_            (0x0000FFFF)

#define WUF_MASK0_0                     (0x200)
#define WUF_MASK0_1                     (0x204)
#define WUF_MASK0_2                     (0x208)
#define WUF_MASK0_3                     (0x20C)
#define NUM_OF_WUF_MASK                 (32)
#define WUF_MASK0_BEGIN                 (WUF_MASK0_0)
#define WUF_MASK1_BEGIN                 (WUF_MASK0_1)
#define WUF_MASK2_BEGIN                 (WUF_MASK0_2)
#define WUF_MASK3_BEGIN                 (WUF_MASK0_3)
#define WUF_MASK0_END                   (WUF_MASK0_BEGIN + (0x10*(NUM_OF_WUF_MASK-1))
#define WUF_MASK1_END                   (WUF_MASK1_BEGIN + (0x10*(NUM_OF_WUF_MASK-1))
#define WUF_MASK2_END                   (WUF_MASK2_BEGIN + (0x10*(NUM_OF_WUF_MASK-1))
#define WUF_MASK3_END                   (WUF_MASK3_BEGIN + (0x10*(NUM_OF_WUF_MASK-1))
#define WUF_MASK0(index)                (WUF_MASK0_BEGIN + (0x10*(index)))  // FIXME add sanity check
#define WUF_MASK1(index)                (WUF_MASK1_BEGIN + (0x10*(index)))  // FIXME add sanity check
#define WUF_MASK2(index)                (WUF_MASK2_BEGIN + (0x10*(index)))  // FIXME add sanity check
#define WUF_MASK3(index)                (WUF_MASK3_BEGIN + (0x10*(index)))  // FIXME add sanity check

#define MAF_BASE                        (0x400)
#define MAF_HIx                         (0x00)
#define MAF_LOx                         (0x04)
#define NUM_OF_MAF                      (33)
#define MAF_HI_BEGIN                    (MAF_BASE + MAF_HIx)
#define MAF_LO_BEGIN                    (MAF_BASE + MAF_LOx)
#define MAF_HI_END                      (MAF_BASE + (8*(NUM_OF_MAF-1) + MAF_HIx)
#define MAF_LO_END                      (MAF_BASE + (8*(NUM_OF_MAF-1) + MAF_LOx)
#define MAF_HI(index)                   (MAF_BASE + (8*index) + MAF_HIx)
#define MAF_LO(index)                   (MAF_BASE + (8*index) + MAF_LOx)
#define MAF_HI_VALID_                   (0x80000000)
#define MAF_HI_TYPE_MASK_               (0x40000000)
#define MAF_HI_TYPE_SRC_                (0x40000000)
#define MAF_HI_TYPE_DST_                (0x00000000)
#define MAF_HI_ADDR_MASK                (0x0000FFFF)
#define MAF_LO_ADDR_MASK                (0xFFFFFFFF)

#define WUCSR2                          (0x600)
#define WUCSR2_CSUM_DISABLE_            (0x80000000)
#define WUCSR2_NA_SA_SEL_               (0x00000100)
#define WUCSR2_NS_RCD_                  (0x00000080)
#define WUCSR2_ARP_RCD_                 (0x00000040)
#define WUCSR2_IPV6_TCPSYN_RCD_         (0x00000020)
#define WUCSR2_IPV4_TCPSYN_RCD_         (0x00000010)
#define WUCSR2_NS_OFFLOAD_EN_           (0x00000008)
#define WUCSR2_ARP_OFFLOAD_EN_          (0x00000004)
#define WUCSR2_IPV6_TCPSYN_WAKE_EN_     (0x00000002)
#define WUCSR2_IPV4_TCPSYN_WAKE_EN_     (0x00000001)

#define PHY_DEV_ID                      (0x700)
#define PHY_DEV_ID_REV_SHIFT_           (28)
#define PHY_DEV_ID_REV_SHIFT_           (28)
#define PHY_DEV_ID_REV_MASK_            (0xF0000000)
#define PHY_DEV_ID_MODEL_SHIFT_         (22)
#define PHY_DEV_ID_MODEL_MASK_          (0x0FC00000)
#define PHY_DEV_ID_OUI_MASK_            (0x003FFFFF)

#define OTP_BASE_ADDR                   (0x00001000)
#define OTP_ADDR_RANGE_                 (0x1FF)

#define OTP_PWR_DN                      (OTP_BASE_ADDR + 4 * 0x00)
#define OTP_PWR_DN_PWRDN_N_             (0x01)

#define OTP_ADDR1                       (OTP_BASE_ADDR + 4 * 0x01)
#define OTP_ADDR1_15_11                 (0x1F)

#define OTP_ADDR2                       (OTP_BASE_ADDR + 4 * 0x02)
#define OTP_ADDR2_10_3                  (0xFF)

#define OTP_ADDR3                       (OTP_BASE_ADDR + 4 * 0x03)
#define OTP_ADDR3_2_0                   (0x03)

#define OTP_PRGM_DATA                   (OTP_BASE_ADDR + 4 * 0x04)

#define OTP_PRGM_MODE                   (OTP_BASE_ADDR + 4 * 0x05)
#define OTP_PRGM_MODE_PGM_MODE_BYTE_    (0x01)

#define OTP_RD_DATA                     (OTP_BASE_ADDR + 4 * 0x06)

#define OTP_FUNC_CMD                    (OTP_BASE_ADDR + 4 * 0x08)
#define OTP_FUNC_CMD_RESET_             (0x04)
#define OTP_FUNC_CMD_PROGRAM_           (0x02)
#define OTP_FUNC_CMD_READ_              (0x01)

#define OTP_TST_CMD                     (OTP_BASE_ADDR + 4 * 0x09)
#define OTP_TST_CMD_TEST_DEC_SEL_       (0x10)
#define OTP_TST_CMD_PROGRAMVERIFY_      (0x08)
#define OTP_TST_CMD_WRTEST_             (0x04)
#define OTP_TST_CMD_TESTDEC_            (0x02)
#define OTP_TST_CMD_BLANKCHECK_         (0x01)

#define OTP_CMD_GO                      (OTP_BASE_ADDR + 4 * 0x0A)
#define OTP_CMD_GO_GO_                  (0x01)

#define OTP_PASS_FAIL                   (OTP_BASE_ADDR + 4 * 0x0B)
#define OTP_PASS_FAIL_PASS_             (0x02)
#define OTP_PASS_FAIL_FAIL_             (0x01)

#define OTP_STATUS                      (OTP_BASE_ADDR + 4 * 0x0C)
#define OTP_STATUS_OTP_LOCK_            (0x10)
#define OTP_STATUS_WEB_                 (0x08)
#define OTP_STATUS_PGMEN                (0x04)
#define OTP_STATUS_CPUMPEN_             (0x02)
#define OTP_STATUS_BUSY_                (0x01)

#define OTP_MAX_PRG                     (OTP_BASE_ADDR + 4 * 0x0D)
#define OTP_MAX_PRG_MAX_PROG            (0x1F)

#define OTP_INTR_STATUS                 (OTP_BASE_ADDR + 4 * 0x10)
#define OTP_INTR_STATUS_READY_          (0x01)

#define OTP_INTR_MASK                   (OTP_BASE_ADDR + 4 * 0x11)
#define OTP_INTR_MASK_READY_            (0x01)

#define OTP_RSTB_PW1                    (OTP_BASE_ADDR + 4 * 0x14)
#define OTP_RSTB_PW2                    (OTP_BASE_ADDR + 4 * 0x15)
#define OTP_PGM_PW1                     (OTP_BASE_ADDR + 4 * 0x18)
#define OTP_PGM_PW2                     (OTP_BASE_ADDR + 4 * 0x19)
#define OTP_READ_PW1                    (OTP_BASE_ADDR + 4 * 0x1C)
#define OTP_READ_PW2                    (OTP_BASE_ADDR + 4 * 0x1D)
#define OTP_TCRST                       (OTP_BASE_ADDR + 4 * 0x20)
#define OTP_RSRD                        (OTP_BASE_ADDR + 4 * 0x21)
#define OTP_TREADEN_VAL                 (OTP_BASE_ADDR + 4 * 0x22)
#define OTP_TDLES_VAL                   (OTP_BASE_ADDR + 4 * 0x23)
#define OTP_TWWL_VAL                    (OTP_BASE_ADDR + 4 * 0x24)
#define OTP_TDLEH_VAL                   (OTP_BASE_ADDR + 4 * 0x25)
#define OTP_TWPED_VAL                   (OTP_BASE_ADDR + 4 * 0x26)
#define OTP_TPES_VAL                    (OTP_BASE_ADDR + 4 * 0x27)
#define OTP_TCPS_VAL                    (OTP_BASE_ADDR + 4 * 0x28)
#define OTP_TCPH_VAL                    (OTP_BASE_ADDR + 4 * 0x29)
#define OTP_TPGMVFY_VAL                 (OTP_BASE_ADDR + 4 * 0x2A)
#define OTP_TPEH_VAL                    (OTP_BASE_ADDR + 4 * 0x2B)
#define OTP_TPGRST_VAL                  (OTP_BASE_ADDR + 4 * 0x2C)
#define OTP_TCLES_VAL                   (OTP_BASE_ADDR + 4 * 0x2D)
#define OTP_TCLEH_VAL                   (OTP_BASE_ADDR + 4 * 0x2E)
#define OTP_TRDES_VAL                   (OTP_BASE_ADDR + 4 * 0x2F)
#define OTP_TBCACC_VAL                  (OTP_BASE_ADDR + 4 * 0x30)
#define OTP_TAAC_VAL                    (OTP_BASE_ADDR + 4 * 0x31)
#define OTP_TACCT_VAL                   (OTP_BASE_ADDR + 4 * 0x32)
#define OTP_TRDEP_VAL                   (OTP_BASE_ADDR + 4 * 0x38)
#define OTP_TPGSV_VAL                   (OTP_BASE_ADDR + 4 * 0x39)
#define OTP_TPVSR_VAL                   (OTP_BASE_ADDR + 4 * 0x3A)
#define OTP_TPVHR_VAL                   (OTP_BASE_ADDR + 4 * 0x3B)
#define OTP_TPVSA_VAL                   (OTP_BASE_ADDR + 4 * 0x3C)

/*
 * LAN8810 PHY Definitions
 */
#define LAN_PHY_ADDR                    (1)

#define PHY_CTRL                        (0x00)
#define MII_CR_SPEED_SELECT_MSB_        (0x0040)
#define MII_CR_COLL_TEST_ENABLE_        (0x0080)
#define MII_CR_RESET_                   (0x8000)
#define MII_CR_LOOPBACK_                (0x4000)
#define MII_CR_SPEED_SELECT_LSB_        (0x2000)
#define MII_CR_AUTO_NEG_EN_             (0x1000)
#define MII_CR_POWER_DOWN_              (0x0800)
#define MII_CR_ISOLATE_                 (0x0400)
#define MII_CR_RESTART_AUTO_NEG_        (0x0200)
#define MII_CR_FULL_DUPLEX_             (0x0100)

#define PHY_STATUS                      (0x01)
#define MII_SR_100T4_CAPS_              (0x8000)
#define MII_SR_100X_FD_CAPS_            (0x4000)
#define MII_SR_100X_HD_CAPS_            (0x2000)
#define MII_SR_10T_FD_CAPS_             (0x1000)
#define MII_SR_10T_HD_CAPS_             (0x0800)
#define MII_SR_100T2_FD_CAPS_           (0x0400)
#define MII_SR_100T2_HD_CAPS_           (0x0200)
#define MII_SR_EXTENDED_STATUS_         (0x0100)
#define MII_SR_AUTONEG_COMPLETE_        (0x0020)
#define MII_SR_REMOTE_FAULT_            (0x0010)
#define MII_SR_AUTONEG_CAPS_            (0x0008)
#define MII_SR_LINK_STATUS_             (0x0004)
#define MII_SR_JABBER_DETECT_           (0x0002)
#define MII_SR_EXTENDED_CAPS_           (0x0001)

#define PHY_ID1                         (0x02)

#define PHY_ID2                         (0x03)

#define PHY_AUTONEG_ADV                 (0x04)
#define NWAY_AR_NEXT_PAGE_              (0x8000)
#define NWAY_AR_REMOTE_FAULT_           (0x2000)
#define NWAY_AR_ASM_DIR_                (0x0800)
#define NWAY_AR_PAUSE_                  (0x0400)
#define NWAY_AR_100T4_CAPS_             (0x0200)
#define NWAY_AR_100TX_FD_CAPS_          (0x0100)
#define NWAY_AR_SELECTOR_FIELD_         (0x001F)
#define NWAY_AR_100TX_HD_CAPS_          (0x0080)
#define NWAY_AR_10T_FD_CAPS_            (0x0040)
#define NWAY_AR_10T_HD_CAPS_            (0x0020)
#define NWAY_AR_ALL_CAPS_               (NWAY_AR_10T_HD_CAPS_ | \
                                         NWAY_AR_10T_FD_CAPS_ | \
                                         NWAY_AR_100TX_HD_CAPS_ | \
                                         NWAY_AR_100TX_FD_CAPS_)
#define NWAY_AR_PAUSE_MASK              (NWAY_AR_PAUSE_ | NWAY_AR_ASM_DIR_)

#define PHY_LP_ABILITY                  (0x05)
#define NWAY_LPAR_NEXT_PAGE_            (0x8000)
#define NWAY_LPAR_ACKNOWLEDGE_          (0x4000)
#define NWAY_LPAR_REMOTE_FAULT_         (0x2000)
#define NWAY_LPAR_ASM_DIR_              (0x0800)
#define NWAY_LPAR_PAUSE_                (0x0400)
#define NWAY_LPAR_100T4_CAPS_           (0x0200)
#define NWAY_LPAR_100TX_FD_CAPS_        (0x0100)
#define NWAY_LPAR_100TX_HD_CAPS_        (0x0080)
#define NWAY_LPAR_10T_FD_CAPS_          (0x0040)
#define NWAY_LPAR_10T_HD_CAPS_          (0x0020)
#define NWAY_LPAR_SELECTOR_FIELD_       (0x001F)

#define PHY_AUTONEG_EXP                 (0x06)
#define NWAY_ER_PAR_DETECT_FAULT_       (0x0010)
#define NWAY_ER_LP_NEXT_PAGE_CAPS_      (0x0008)
#define NWAY_ER_NEXT_PAGE_CAPS_         (0x0004)
#define NWAY_ER_PAGE_RXD_               (0x0002)
#define NWAY_ER_LP_NWAY_CAPS_           (0x0001)

#define PHY_NEXT_PAGE_TX                (0x07)
#define NPTX_NEXT_PAGE_                 (0x8000)
#define NPTX_MSG_PAGE_                  (0x2000)
#define NPTX_ACKNOWLDGE2_               (0x1000)
#define NPTX_TOGGLE_                    (0x0800)
#define NPTX_MSG_CODE_FIELD_            (0x0001)

#define PHY_LP_NEXT_PAGE                (0x08)
#define LP_RNPR_NEXT_PAGE_              (0x8000)
#define LP_RNPR_ACKNOWLDGE_             (0x4000)
#define LP_RNPR_MSG_PAGE_               (0x2000)
#define LP_RNPR_ACKNOWLDGE2_            (0x1000)
#define LP_RNPR_TOGGLE_                 (0x0800)
#define LP_RNPR_MSG_CODE_FIELD_         (0x0001)

#define PHY_1000T_CTRL                  (0x09)
#define CR_1000T_TEST_MODE_4_           (0x8000)
#define CR_1000T_TEST_MODE_3_           (0x6000)
#define CR_1000T_TEST_MODE_2_           (0x4000)
#define CR_1000T_TEST_MODE_1_           (0x2000)
#define CR_1000T_MS_ENABLE_             (0x1000)
#define CR_1000T_MS_VALUE_              (0x0800)
#define CR_1000T_REPEATER_DTE_          (0x0400)
#define CR_1000T_FD_CAPS_               (0x0200)
#define CR_1000T_HD_CAPS_               (0x0100)
#define CR_1000T_ASYM_PAUSE_            (0x0080)
#define CR_1000T_TEST_MODE_NORMAL_      (0x0000)

#define PHY_1000T_STATUS                (0x0A)
#define SR_1000T_MS_CONFIG_FAULT_       (0x8000)
#define SR_1000T_MS_CONFIG_RES_         (0x4000)
#define SR_1000T_LOCAL_RX_STATUS_       (0x2000)
#define SR_1000T_REMOTE_RX_STATUS_      (0x1000)
#define SR_1000T_LP_FD_CAPS_            (0x0800)
#define SR_1000T_LP_HD_CAPS_            (0x0400)
#define SR_1000T_ASYM_PAUSE_DIR_        (0x0100)
#define SR_1000T_IDLE_ERROR_CNT_        (0x00FF)
#define SR_1000T_REMOTE_RX_STATUS_SHIFT         12
#define SR_1000T_LOCAL_RX_STATUS_SHIFT          13
#define SR_1000T_PHY_EXCESSIVE_IDLE_ERR_COUNT   5
#define FFE_IDLE_ERR_COUNT_TIMEOUT_20           20
#define FFE_IDLE_ERR_COUNT_TIMEOUT_100          100

// Reserved Reg's
#define PHY_REG11                       (0x0B)
#define PHY_REG12                       (0x0C)
#define PHY_REG13                       (0x0D)
#define PHY_REG14                       (0x0E)

#define PHY_EXT_STATUS                  (0x0F)
#define IEEE_ESR_1000X_FD_CAPS_         (0x8000)
#define IEEE_ESR_1000X_HD_CAPS_         (0x4000)
#define IEEE_ESR_1000T_FD_CAPS_         (0x2000)
#define IEEE_ESR_1000T_HD_CAPS_         (0x1000)
#define PHY_TX_POLARITY_MASK_           (0x0100)
#define PHY_TX_NORMAL_POLARITY_         (0x0000)
#define AUTO_POLARITY_DISABLE_          (0x0010)

#define PHY_MMD_CTL                     (0x0D)
#define PHY_MMD_CTRL_OP_MASK_           (0xC000)
#define PHY_MMD_CTRL_OP_REG_            (0x0000)
#define PHY_MMD_CTRL_OP_DNI_            (0x4000)
#define PHY_MMD_CTRL_OP_DPIRW_          (0x8000)
#define PHY_MMD_CTRL_OP_DPIWO_          (0xC000)
#define PHY_MMD_CTRL_DEV_ADDR_MASK_     (0x001F)

#define PHY_MMD_REG_DATA                (0x0E)

/*
 *  VTSE Vendor Specific registers
 */
#define PHY_VTSE_BYPASS                         (0x12)
#define PHY_VTSE_BYPASS_DISABLE_PAIR_SWAP_      (0x0020)

#define PHY_VTSE_INT_MASK                       (0x19)
#define PHY_VTSE_INT_MASK_MDINTPIN_EN_          (0x8000)
#define PHY_VTSE_INT_MASK_SPEED_CHANGE_         (0x4000)
#define PHY_VTSE_INT_MASK_LINK_CHANGE_          (0x2000)
#define PHY_VTSE_INT_MASK_FDX_CHANGE_           (0x1000)
#define PHY_VTSE_INT_MASK_AUTONEG_ERR_          (0x0800)
#define PHY_VTSE_INT_MASK_AUTONEG_DONE_         (0x0400)
#define PHY_VTSE_INT_MASK_POE_DETECT_           (0x0200)
#define PHY_VTSE_INT_MASK_SYMBOL_ERR_           (0x0100)
#define PHY_VTSE_INT_MASK_FAST_LINK_FAIL_       (0x0080)
#define PHY_VTSE_INT_MASK_WOL_EVENT_            (0x0040)
#define PHY_VTSE_INT_MASK_EXTENDED_INT_         (0x0020)
#define PHY_VTSE_INT_MASK_RESERVED_             (0x0010)
#define PHY_VTSE_INT_MASK_FALSE_CARRIER_        (0x0008)
#define PHY_VTSE_INT_MASK_LINK_SPEED_DS_        (0x0004)
#define PHY_VTSE_INT_MASK_MASTER_SLAVE_DONE_    (0x0002)
#define PHY_VTSE_INT_MASK_RX__ER_               (0x0001)

#define PHY_VTSE_INT_STS                        (0x1A)
#define PHY_VTSE_INT_STS_INT_ACTIVE_            (0x8000)
#define PHY_VTSE_INT_STS_SPEED_CHANGE_          (0x4000)
#define PHY_VTSE_INT_STS_LINK_CHANGE_           (0x2000)
#define PHY_VTSE_INT_STS_FDX_CHANGE_            (0x1000)
#define PHY_VTSE_INT_STS_AUTONEG_ERR_           (0x0800)
#define PHY_VTSE_INT_STS_AUTONEG_DONE_          (0x0400)
#define PHY_VTSE_INT_STS_POE_DETECT_            (0x0200)
#define PHY_VTSE_INT_STS_SYMBOL_ERR_            (0x0100)
#define PHY_VTSE_INT_STS_FAST_LINK_FAIL_        (0x0080)
#define PHY_VTSE_INT_STS_WOL_EVENT_             (0x0040)
#define PHY_VTSE_INT_STS_EXTENDED_INT_          (0x0020)
#define PHY_VTSE_INT_STS_RESERVED_              (0x0010)
#define PHY_VTSE_INT_STS_FALSE_CARRIER_         (0x0008)
#define PHY_VTSE_INT_STS_LINK_SPEED_DS_         (0x0004)
#define PHY_VTSE_INT_STS_MASTER_SLAVE_DONE_     (0x0002)
#define PHY_VTSE_INT_STS_RX_ER_                 (0x0001)

#define PHY_VTSE_DEV_ACTL_STS                   (0x1C)
#define PHY_VTSE_DEV_ACTL_STS_AUTONEG_DONE_     (0x8000)
#define PHY_VTSE_DEV_ACTL_STS_AUTONEG_DIS_      (0x4000)
#define PHY_VTSE_DEV_ACTL_STS_AMDIX_            (0x2000)
#define PHY_VTSE_DEV_ACTL_STS_CD_PAIR_SWAP_     (0x1000)
#define PHY_VTSE_DEV_ACTL_STS_A_POL_INV_        (0x0800)
#define PHY_VTSE_DEV_ACTL_STS_B_POL_INV_        (0x0400)
#define PHY_VTSE_DEV_ACTL_STS_C_POL_INV_        (0x0200)
#define PHY_VTSE_DEV_ACTL_STS_D_POL_INV_        (0x0100)
#define PHY_VTSE_DEV_ACTL_STS_ACTIPHY_TOH_      (0x0080)
#define PHY_VTSE_DEV_ACTL_STS_ACTIPHY_EN_       (0x0040)
#define PHY_VTSE_DEV_ACTL_STS_FDX_STS_          (0x0020)
#define PHY_VTSE_DEV_ACTL_STS_SPEED_MASK_       (0x0018)
#define PHY_VTSE_DEV_ACTL_STS_SPEED_10_         (0x0000)
#define PHY_VTSE_DEV_ACTL_STS_SPEED_100_        (0x0008)
#define PHY_VTSE_DEV_ACTL_STS_SPEED_1000_       (0x0010)
#define PHY_VTSE_DEV_ACTL_STS_SPEED_RESERVED_   (0x0018)
#define PHY_VTSE_DEV_ACTL_STS_ACTIPHY_TOL_      (0x0004)
#define PHY_VTSE_DEV_ACTL_STS_MEDIA_MODE_MASK_  (0x0003)

#define PHY_EXT_GPIO_PAGE                       (0x1F)
#define PHY_EXT_GPIO_PAGE_SPACE_0               (0x0000)
#define PHY_EXT_GPIO_PAGE_SPACE_1               (0x0001)
#define PHY_EXT_GPIO_PAGE_SPACE_2               (0x0002)

/*
 * Extended Register Page 1 space
 */
#define PHY_EXT_MODE_CTRL                       (0x13)
#define PHY_EXT_MODE_CTRL_MDIX_MASK_            (0x000C)
#define PHY_EXT_MODE_CTRL_AUTO_MDIX_            (0x0000)
#define PHY_EXT_MODE_CTRL_MDI_                  (0x0008)
#define PHY_EXT_MODE_CTRL_MDI_X_                (0x000C)

#define PHY_ANA_10BASE_T_HD                     0x01
#define PHY_ANA_10BASE_T_FD                     0x02
#define PHY_ANA_100BASE_TX_HD                   0x04
#define PHY_ANA_100BASE_TX_FD                   0x08
#define PHY_ANA_1000BASE_T_FD                   0x10
#define PHY_ANA_ALL_SUPPORTED_MEDIA             (PHY_ANA_10BASE_T_HD |   \
                                                 PHY_ANA_10BASE_T_FD |   \
                                                 PHY_ANA_100BASE_TX_HD | \
                                                 PHY_ANA_100BASE_TX_FD | \
                                                 PHY_ANA_1000BASE_T_FD)

#define PHY_ACTIPHY_CTRL                        (0x14)
#define PHY_ACTIPHY_CTRL_DIS_CARRIER_EXT_       (0x8000)
#define PHY_ACTIPHY_CTRL_SLEEP_TIM_MASK_        (0x6000)
#define PHY_ACTIPHY_CTRL_WAKE_TIM_MASK_         (0x1800)
#define PHY_ACTIPHY_CTRL_SLOW_MDC_              (0x0400)
#define PHY_ACTIPHY_CTRL_PHY_ADDR_REVERSAL_     (0x0200)
#define PHY_ACTIPHY_CTRL_MEDIA_MODE_STS_        (0x00C0)
#define PHY_ACTIPHY_CTRL_MEDIA_MODE_STS_COPPER_ (0x0040)
#define PHY_ACTIPHY_CTRL_10BT_NO_PREAM_EN_      (0x0020)
#define PHY_ACTIPHY_CTRL_SPEED_AUTO_DS_EN_      (0x0010)
/*
 * PHY MMD registers
 */
#define PHY_MMD_DEV_3                           3

#define PHY_EEE_PCS_STATUS                      (0x1)
#define PHY_EEE_PCS_STATUS_TX_LPI_RCVD_         ((WORD)0x0800)
#define PHY_EEE_PCS_STATUS_RX_LPI_RCVD_         ((WORD)0x0400)
#define PHY_EEE_PCS_STATUS_TX_LPI_IND_          ((WORD)0x0200)
#define PHY_EEE_PCS_STATUS_RX_LPI_IND_          ((WORD)0x0100)
#define PHY_EEE_PCS_STATUS_PCS_RCV_LNK_STS_     ((WORD)0x0004)

#define PHY_EEE_CAPABILITIES                    (0x14)
#define PHY_EEE_CAPABILITIES_1000BT_EEE_        ((WORD)0x0004)
#define PHY_EEE_CAPABILITIES_100BT_EEE_         ((WORD)0x0002)

#define PHY_MMD_DEV_7                           7

#define PHY_EEE_ADVERTISEMENT                   (0x3C)
#define PHY_EEE_ADVERTISEMENT_1000BT_EEE_       ((WORD)0x0004)
#define PHY_EEE_ADVERTISEMENT_100BT_EEE_        ((WORD)0x0002)

#define PHY_EEE_LP_ADVERTISEMENT                (0x3D)
#define PHY_EEE_1000BT_EEE_CAPABLE_             ((WORD)0x0004)
#define PHY_EEE_100BT_EEE_CAPABLE_              ((WORD)0x0002)

/* USB Vendor Requests */
#define USB_VENDOR_REQUEST_WRITE_REGISTER       0xA0
#define USB_VENDOR_REQUEST_READ_REGISTER        0xA1
#define USB_VENDOR_REQUEST_GET_STATS            0xA2

//Other parameters
#define SS_USB_PKT_SIZE                 (1024)
#define HS_USB_PKT_SIZE                 (512)
#define FS_USB_PKT_SIZE                 (64)

#define MAX_RX_FIFO_SIZE                (12 * 1024)
#define MAX_TX_FIFO_SIZE                (12 * 1024)
#define DEFAULT_BURST_CAP_SIZE          (MAX_RX_FIFO_SIZE)
#define DEFAULT_BULK_IN_DELAY           (0x0800)

#define LAN78XX_USB_VENDOR_ID           (0x0424)
#define LAN7800_USB_PRODUCT_ID          (0x7800)

#define EEPROM_MAC_OFFSET               (0x01)
#define MAX_EEPROM_SIZE                 512
#define ETH_ALEN						0x6
#define OTP_INDICATOR_1                 (0xF3)
#define OTP_INDICATOR_2                 (0xF7)

EFI_STATUS
Lan7800ReadRegister(
	IN LAN7800_ADAPTER_DATA	*Adapter,
	IN UINT16				Address,
	OUT UINT32				*Data
	);

EFI_STATUS
Lan7800WriteRegister(
	IN LAN7800_ADAPTER_DATA	*Adapter,
	IN UINT16				Address,
	OUT UINT32				Data
  );

EFI_STATUS
Lan7800ReadPhyRegister(
	IN LAN7800_ADAPTER_DATA	*Adapter,
	IN UINT8				PhyId,
	IN UINT32				Index,
	OUT UINT32				*Data
	);

EFI_STATUS
Lan7800WritePhyRegister (
	IN LAN7800_ADAPTER_DATA	*Adapter,
	IN UINT8				PhyId,
	IN UINT32				Index,
	OUT UINT32				Data
	);

EFI_STATUS 
Lan7800DeviceInitialize(
	IN LAN7800_ADAPTER_DATA	*Adapter
	);

EFI_STATUS 
Lan7800Reset(
	IN LAN7800_ADAPTER_DATA	*Adapter
	);

EFI_STATUS 
Lan7800SetMulticast(
	IN LAN7800_ADAPTER_DATA	*Adapter
	);

EFI_STATUS 
Lan7800SetMacAddress(
	IN LAN7800_ADAPTER_DATA	*Adapter
	);

EFI_STATUS 
Lan7800LinkCheck(
	IN LAN7800_ADAPTER_DATA	*Adapter,
	UINT16					*Flags
	);

EFI_STATUS 
Lan7800Transmit(
	IN LAN7800_ADAPTER_DATA	*Adapter,
	IN UINT8				*FrameAddr,
	IN UINT16				MeadiaHeaderLen,
	IN UINT32				DataLen
	);

EFI_STATUS 
Lan7800Receive(
	IN LAN7800_ADAPTER_DATA	*Adapter
	);

BOOLEAN
IsBroadcastMacAddress(
  IN  UINT8    *MacAddr
  );

BOOLEAN
IsMulticaseMacAddress(
  IN  UINT8    *MacAddr
  );

VOID
PacketDump(
	UINT8	*Data,
	UINT32	Length
	);

EFI_STATUS
Lan7800WriteEeprom(
	IN LAN7800_ADAPTER_DATA	*Adapter,
	IN UINTN				Offset,
	IN UINTN				Length,
	OUT UINT8				*Data
	);

EFI_STATUS
Lan7800ReadEeprom(
	IN LAN7800_ADAPTER_DATA	*Adapter,
	IN UINTN				Offset,
	IN UINTN				Length,
	OUT UINT8				*Data
	);

EFI_STATUS
Lan7800GetEepromLength(
	IN LAN7800_ADAPTER_DATA	*Adapter,
	OUT UINT32				*Length
	);

EFI_STATUS
Lan7800ReadOtpInternal(
	IN LAN7800_ADAPTER_DATA	*Adapter,
	IN UINTN				Offset,
	IN UINTN				Length,
	OUT UINT8				*Data
	);

EFI_STATUS
Lan7800ReadOtp(
	IN LAN7800_ADAPTER_DATA	*Adapter,
	IN UINTN				Offset,
	IN UINTN				Length,
	OUT UINT8				*Data
	);

EFI_STATUS
Lan7800GetLinkStatus(
	IN LAN7800_ADAPTER_DATA	*Adapter,
	OUT	UINT32				*LinkSpeed,
	OUT	UINT32				*Duplex
	);

EFI_STATUS
Lan7800StandardDiagnostics(
	IN LAN7800_ADAPTER_DATA	*Adapter
	);

EFI_STATUS
Lan7800ExtendedDiagnostics(
	IN LAN7800_ADAPTER_DATA	*Adapter
	);

EFI_STATUS
Lan7800ManufacturingDiagnostics(
	IN LAN7800_ADAPTER_DATA	*Adapter
	);

#endif /* _LAN7800_H_ */

