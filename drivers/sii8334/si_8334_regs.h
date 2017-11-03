/*
 *****************************************************************************
 *
 * Copyright 2010, Silicon Image, Inc.  All rights reserved.
 * No part of this work may be reproduced, modified, distributed, transmitted,
 * transcribed, or translated into any language or computer format, in any form
 * or by any means without written permission of: Silicon Image, Inc., 1060
 * East Arques Avenue, Sunnyvale, California 94085
 *****************************************************************************
 */
/*
 *****************************************************************************
 * @file  TPI_Regs.h
 *
 * @brief Implementation of the Foo API.
 *
 *****************************************************************************
*/

/***********************************************************************************/
/*  Copyright (c) 2002-2009, Silicon Image, Inc.  All rights reserved.             */
/*  No part of this work may be reproduced, modified, distributed, transmitted,    */
/*  transcribed, or translated into any language or computer format, in any form   */
/*  or by any means without written permission of: Silicon Image, Inc.,            */
/*  1060 East Arques Avenue, Sunnyvale, California 94085                           */
/***********************************************************************************/


#define REG_SRST		(TX_PAGE_3 | 0x0000)		// Was 0x0005

#define REG_DISC_CTRL1	(TX_PAGE_3 | 0x0010)		// Was 0x0090
#define REG_DISC_CTRL2	(TX_PAGE_3 | 0x0011)		// Was 0x0091
#define REG_DISC_CTRL3	(TX_PAGE_3 | 0x0012)		// Was 0x0092
#define REG_DISC_CTRL4	(TX_PAGE_3 | 0x0013)		// Was 0x0093
#define REG_DISC_CTRL5	(TX_PAGE_3 | 0x0014)		// Was 0x0094
#define REG_DISC_CTRL6	(TX_PAGE_3 | 0x0015)		// Was 0x0095
#define REG_DISC_CTRL7	(TX_PAGE_3 | 0x0016)		// Was 0x0096
#define REG_DISC_CTRL8	(TX_PAGE_3 | 0x0017)		// Was 0x0097
#define REG_DISC_CTRL9	(TX_PAGE_3 | 0x0018)
#define REG_DISC_CTRL10	(TX_PAGE_3 | 0x0019)
#define REG_DISC_CTRL11	(TX_PAGE_3 | 0x001A)
#define REG_DISC_STAT	(TX_PAGE_3 | 0x001B)		// Was 0x0098
#define REG_DISC_STAT2	(TX_PAGE_3 | 0x001C)		// Was 0x0099

//added by garyyuan 20110819
#define REG_INTR1		(TX_PAGE_TPI | 0x0071)
#define REG_INTR1_MASK	(TX_PAGE_TPI | 0x0075)

#define REG_INT_CTRL	(TX_PAGE_3 | 0x0020)		// Was 0x0079
#define REG_INTR4		(TX_PAGE_3 | 0x0021)		// Was 0x0074
#define REG_INTR4_MASK	(TX_PAGE_3 | 0x0022)		// Was 0x0078
#define REG_INTR5		(TX_PAGE_3 | 0x0023)
#define REG_INTR5_MASK	(TX_PAGE_3 | 0x0024)

#define REG_MHLTX_CTL1	(TX_PAGE_3 | 0x0030)		// Was 0x00A0
#define REG_MHLTX_CTL2	(TX_PAGE_3 | 0x0031)		// Was 0x00A1
#define REG_MHLTX_CTL3	(TX_PAGE_3 | 0x0032)		// Was 0x00A2
#define REG_MHLTX_CTL4	(TX_PAGE_3 | 0x0033)		// Was 0x00A3
#define REG_MHLTX_CTL5	(TX_PAGE_3 | 0x0034)		// Was 0x00A4
#define REG_MHLTX_CTL6	(TX_PAGE_3 | 0x0035)		// Was 0x00A5
#define REG_MHLTX_CTL7	(TX_PAGE_3 | 0x0036)		// Was 0x00A6
#define REG_MHLTX_CTL8	(TX_PAGE_3 | 0x0037)

#define REG_TMDS_CSTAT	(TX_PAGE_3 | 0x0040)

#define REG_MIPI_DSI_CTRL		(TX_PAGE_3 | 0x0092)
#define REG_MIPI_DSI_FORMAT		(TX_PAGE_3 | 0x0093)
#define REG_MIPI_DSI_PXL_FORMAT	(TX_PAGE_3 | 0x0094)	

// ===================================================== //

#define TPI_SYSTEM_CONTROL_DATA_REG			(TX_PAGE_TPI | 0x001A)

#define LINK_INTEGRITY_MODE_MASK			(BIT6)
#define LINK_INTEGRITY_STATIC				(0x00)
#define LINK_INTEGRITY_DYNAMIC				(0x40)

#define TMDS_OUTPUT_CONTROL_MASK			(BIT4)
#define TMDS_OUTPUT_CONTROL_ACTIVE			(0x00)
#define TMDS_OUTPUT_CONTROL_POWER_DOWN		(0x10)

#define AV_MUTE_MASK						(BIT3)
#define AV_MUTE_NORMAL						(0x00)
#define AV_MUTE_MUTED						(0x08)

#define DDC_BUS_REQUEST_MASK				(BIT2)
#define DDC_BUS_REQUEST_NOT_USING			(0x00)
#define DDC_BUS_REQUEST_REQUESTED			(0x04)

#define DDC_BUS_GRANT_MASK					(BIT1)
#define DDC_BUS_GRANT_NOT_AVAILABLE			(0x00)
#define DDC_BUS_GRANT_GRANTED				(0x02)

#define OUTPUT_MODE_MASK					(BIT0)
#define OUTPUT_MODE_DVI						(0x00)
#define OUTPUT_MODE_HDMI					(0x01)

// ===================================================== //

#define TPI_DEVICE_POWER_STATE_CTRL_REG		(TX_PAGE_TPI | 0x001E)

#define CTRL_PIN_CONTROL_MASK				(BIT4)
#define CTRL_PIN_TRISTATE					(0x00)
#define CTRL_PIN_DRIVEN_TX_BRIDGE			(0x10)

#define TX_POWER_STATE_MASK					(BIT1 | BIT0)
#define TX_POWER_STATE_D0					(0x00)
#define TX_POWER_STATE_D2					(0x02)
#define TX_POWER_STATE_D3					(0x03)

/*\
| | HDCP Implementation
| |
| | HDCP link security logic is implemented in certain transmitters; unique
| |   keys are embedded in each chip as part of the solution. The security
| |   scheme is fully automatic and handled completely by the hardware.
\*/

/// HDCP Query Data Register ============================================== ///

#define TPI_HDCP_QUERY_DATA_REG				(TX_PAGE_TPI | 0x0029)

#define EXTENDED_LINK_PROTECTION_MASK		(BIT7)
#define EXTENDED_LINK_PROTECTION_NONE		(0x00)
#define EXTENDED_LINK_PROTECTION_SECURE		(0x80)

#define LOCAL_LINK_PROTECTION_MASK			(BIT6)
#define LOCAL_LINK_PROTECTION_NONE			(0x00)
#define LOCAL_LINK_PROTECTION_SECURE		(0x40)

#define LINK_STATUS_MASK					(BIT5 | BIT4)
#define LINK_STATUS_NORMAL					(0x00)
#define LINK_STATUS_LINK_LOST				(0x10)
#define LINK_STATUS_RENEGOTIATION_REQ		(0x20)
#define LINK_STATUS_LINK_SUSPENDED			(0x30)

#define HDCP_REPEATER_MASK					(BIT3)
#define HDCP_REPEATER_NO					(0x00)
#define HDCP_REPEATER_YES					(0x08)

#define CONNECTOR_TYPE_MASK					(BIT2 | BIT0)
#define CONNECTOR_TYPE_DVI					(0x00)
#define CONNECTOR_TYPE_RSVD					(0x01)
#define CONNECTOR_TYPE_HDMI					(0x04)
#define CONNECTOR_TYPE_FUTURE				(0x05)

#define PROTECTION_TYPE_MASK				(BIT1)
#define PROTECTION_TYPE_NONE				(0x00)
#define PROTECTION_TYPE_HDCP				(0x02)

/// HDCP Control Data Register ============================================ ///

#define TPI_HDCP_CONTROL_DATA_REG			(TX_PAGE_TPI | 0x002A)

#define PROTECTION_LEVEL_MASK				(BIT0)
#define PROTECTION_LEVEL_MIN				(0x00)
#define PROTECTION_LEVEL_MAX				(0x01)

#define KSV_FORWARD_MASK					(BIT4)
#define KSV_FORWARD_ENABLE					(0x10)
#define KSV_FORWARD_DISABLE					(0x00)

/// HDCP BKSV Registers =================================================== ///

#define TPI_BKSV_1_REG						(TX_PAGE_TPI | 0x002B)
#define TPI_BKSV_2_REG						(TX_PAGE_TPI | 0x002C)
#define TPI_BKSV_3_REG						(TX_PAGE_TPI | 0x002D)
#define TPI_BKSV_4_REG						(TX_PAGE_TPI | 0x002E)
#define TPI_BKSV_5_REG						(TX_PAGE_TPI | 0x002F)

/// HDCP Revision Data Register =========================================== ///

#define TPI_HDCP_REVISION_DATA_REG			(TX_PAGE_TPI | 0x0030)

#define HDCP_MAJOR_REVISION_MASK			(BIT7 | BIT6 | BIT5 | BIT4)
#define HDCP_MAJOR_REVISION_VALUE			(0x10)

#define HDCP_MINOR_REVISION_MASK			(BIT3 | BIT2 | BIT1 | BIT0)
#define HDCP_MINOR_REVISION_VALUE			(0x02)

/// HDCP KSV and V' Value Data Register =================================== ///

#define TPI_V_PRIME_SELECTOR_REG			(TX_PAGE_TPI | 0x0031)

/// V' Value Readback Registers =========================================== ///

#define TPI_V_PRIME_7_0_REG					(TX_PAGE_TPI | 0x0032)
#define TPI_V_PRIME_15_9_REG				(TX_PAGE_TPI | 0x0033)
#define TPI_V_PRIME_23_16_REG				(TX_PAGE_TPI | 0x0034)
#define TPI_V_PRIME_31_24_REG				(TX_PAGE_TPI | 0x0035)

/// HDCP AKSV Registers =================================================== ///

#define TPI_AKSV_1_REG						(TX_PAGE_TPI | 0x0036)
#define TPI_AKSV_2_REG						(TX_PAGE_TPI | 0x0037)
#define TPI_AKSV_3_REG						(TX_PAGE_TPI | 0x0038)
#define TPI_AKSV_4_REG						(TX_PAGE_TPI | 0x0039)
#define TPI_AKSV_5_REG						(TX_PAGE_TPI | 0x003A)

/// Interrupt Status Register ============================================= ///

#define TPI_INTERRUPT_STATUS_REG			(TX_PAGE_TPI | 0x003D)

#define HDCP_AUTH_STATUS_CHANGE_EVENT_MASK	(BIT7)
#define HDCP_AUTH_STATUS_CHANGE_EVENT_NO	(0x00)
#define HDCP_AUTH_STATUS_CHANGE_EVENT_YES	(0x80)

#define HDCP_VPRIME_VALUE_READY_EVENT_MASK	(BIT6)
#define HDCP_VPRIME_VALUE_READY_EVENT_NO	(0x00)
#define HDCP_VPRIME_VALUE_READY_EVENT_YES	(0x40)

#define HDCP_SECURITY_CHANGE_EVENT_MASK		(BIT5)
#define HDCP_SECURITY_CHANGE_EVENT_NO		(0x00)
#define HDCP_SECURITY_CHANGE_EVENT_YES		(0x20)

#define AUDIO_ERROR_EVENT_MASK				(BIT4)
#define AUDIO_ERROR_EVENT_NO				(0x00)
#define AUDIO_ERROR_EVENT_YES				(0x10)

#define CPI_EVENT_MASK						(BIT3)
#define CPI_EVENT_NO						(0x00)
#define CPI_EVENT_YES						(0x08)
#define RX_SENSE_MASK						(BIT3)		// This bit is dual purpose depending on the value of 0x3C[3]
#define RX_SENSE_NOT_ATTACHED				(0x00)
#define RX_SENSE_ATTACHED					(0x08)

#define HOT_PLUG_PIN_STATE_MASK				(BIT2)
#define HOT_PLUG_PIN_STATE_LOW				(0x00)
#define HOT_PLUG_PIN_STATE_HIGH				(0x04)

#define RECEIVER_SENSE_EVENT_MASK			(BIT1)
#define RECEIVER_SENSE_EVENT_NO				(0x00)
#define RECEIVER_SENSE_EVENT_YES			(0x02)

#define HOT_PLUG_EVENT_MASK					(BIT0)
#define HOT_PLUG_EVENT_NO					(0x00)
#define HOT_PLUG_EVENT_YES					(0x01)
