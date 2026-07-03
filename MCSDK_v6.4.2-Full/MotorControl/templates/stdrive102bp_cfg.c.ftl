<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
/**
  ******************************************************************************
  * @file    stdrive102bp_cfg.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   STDRIVE102BP configuration parameters
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
    
#include "stdrive102bp_driver.h"
#include "main.h"

<#if (MC.M1_PWM_DRIVER_PN == "STDRIVE102BP")>

<#assign M1_STDRIVE102_IGATE = "5">
<#assign M1_STDRIVE102_TCC = "5">
<#assign M1_STDRIVE102_EQMODE = "BP_IGATE_EQ_TCC_LOW_RANGE">
<#list MC.M1_PhaseVoltageGeneration_FWConfig?split(",") as phv1>
<#if phv1?contains("IGATE=")> <#assign M1_STDRIVE102_IGATE = (phv1?keep_after("="))?replace("'", "")> </#if>
<#if phv1?contains("TCC=")> <#assign M1_STDRIVE102_TCC = (phv1?keep_after("="))?replace("'", "")> </#if>
<#if phv1?contains("EQMODE=")> <#assign M1_STDRIVE102_EQMODE = (phv1?keep_after("="))?replace("'", "")> </#if>
</#list>

<#if ((M1_STDRIVE102_EQMODE == "BP_IGATE_EQ_TCC_LOW_RANGE") && (M1_STDRIVE102_TCC?eval == 15))> <#assign M1_STDRIVE102_VDSDGT = "BP_VDS_DGT_4us5">
<#elseif ((M1_STDRIVE102_EQMODE != "BP_IGATE_EQ_TCC_LOW_RANGE") && (M1_STDRIVE102_TCC?eval > 8) && (M1_STDRIVE102_TCC?eval <= 11))> <#assign M1_STDRIVE102_VDSDGT = "BP_VDS_DGT_4us5">
<#elseif ((M1_STDRIVE102_EQMODE != "BP_IGATE_EQ_TCC_LOW_RANGE") && (M1_STDRIVE102_TCC?eval > 11) && (M1_STDRIVE102_TCC?eval <= 13))> <#assign M1_STDRIVE102_VDSDGT = "BP_VDS_DGT_6us">
<#elseif ((M1_STDRIVE102_EQMODE != "BP_IGATE_EQ_TCC_LOW_RANGE") && (M1_STDRIVE102_TCC?eval > 13) && (M1_STDRIVE102_TCC?eval <= 15))> <#assign M1_STDRIVE102_VDSDGT = "BP_VDS_DGT_7us">
<#else> <#assign M1_STDRIVE102_VDSDGT = "BP_VDS_DGT_3us5">
</#if>

<#if (MC.M1_DRIVE_TYPE == "FOC")> <#assign M1_STDRIVE102_PGA_REF_SEL = "BP_PGA_HALF_VDD"> 
<#elseif (MC.M1_DRIVE_TYPE == "SIX_STEP")> <#assign M1_STDRIVE102_PGA_REF_SEL = "BP_PGA_NO_OFFSET">
<#else> <#assign M1_STDRIVE102_PGA_REF_SEL = "BP_PGA_NO_OFFSET">
</#if>

<#if MC.M1_CURRENT_MONITOR_READING == true>
  <#if (MC.M1_CUR_MON_GAIN_FACTOR == "4")> <#assign M1_STDRIVE102_PGA_GAIN = "BP_PGA_GAIN_4">
  <#elseif (MC.M1_CUR_MON_GAIN_FACTOR == "8")> <#assign M1_STDRIVE102_PGA_GAIN = "BP_PGA_GAIN_8"> 
  <#elseif (MC.M1_CUR_MON_GAIN_FACTOR == "16")> <#assign M1_STDRIVE102_PGA_GAIN = "BP_PGA_GAIN_16"> 
  <#elseif (MC.M1_CUR_MON_GAIN_FACTOR == "32")> <#assign M1_STDRIVE102_PGA_GAIN = "BP_PGA_GAIN_32"> 
  <#else> <#assign M1_STDRIVE102_PGA_GAIN = "BP_PGA_GAIN_32"> 
  </#if>
<#else>
  <#if (MC.M1_AMPLIFICATION_GAIN == "4")> <#assign M1_STDRIVE102_PGA_GAIN = "BP_PGA_GAIN_4">
  <#elseif (MC.M1_AMPLIFICATION_GAIN == "8")> <#assign M1_STDRIVE102_PGA_GAIN = "BP_PGA_GAIN_8"> 
  <#elseif (MC.M1_AMPLIFICATION_GAIN == "16")> <#assign M1_STDRIVE102_PGA_GAIN = "BP_PGA_GAIN_16"> 
  <#elseif (MC.M1_AMPLIFICATION_GAIN == "32")> <#assign M1_STDRIVE102_PGA_GAIN = "BP_PGA_GAIN_32"> 
  <#else> <#assign M1_STDRIVE102_PGA_GAIN = "BP_PGA_GAIN_32"> 
  </#if>
</#if>

<#if MC.M1_LOW_SIDE_SIGNALS_ENABLING == "LS_PWM_TIMER"> <#assign M1_STDRIVE102_DRVMODE = "BP_DIRECT_NO_WAIT">
<#elseif MC.M1_LOW_SIDE_SIGNALS_ENABLING == "ES_GPIO"> <#assign M1_STDRIVE102_DRVMODE = "BP_EN_IN">
<#else> <#assign M1_STDRIVE102_DRVMODE = "BP_EN_IN">
</#if>


<#assign M1_STDRIVE102_COMP1_ACTIVE = "true">
<#assign M1_STDRIVE102_COMP2_ACTIVE = "true">
<#assign M1_STDRIVE102_COMP3_ACTIVE = "true"> 
<#assign M1_STDRIVE102_COMP1_INP = "BP_COMP_DIRECT_INP">
<#assign M1_STDRIVE102_COMP2_INP = "BP_COMP_DIRECT_INP">
<#assign M1_STDRIVE102_COMP3_INP = "BP_COMP_DIRECT_INP">
<#assign M1_STDRIVE102_COMP1_INN = "BP_COMP_TH_EXT_CREF">
<#assign M1_STDRIVE102_COMP2_INN = "BP_COMP_TH_EXT_CREF">
<#assign M1_STDRIVE102_COMP3_INN = "BP_COMP_TH_EXT_CREF">
<#assign M1_STDRIVE102_COMP1_DGT = "BP_COMP_DGT_2500ns">
<#assign M1_STDRIVE102_COMP2_DGT = "BP_COMP_DGT_2500ns">
<#assign M1_STDRIVE102_COMP3_DGT = "BP_COMP_DGT_2500ns">
<#assign M1_STDRIVE102_COMP_INT_REF = "BP_COMP_TH_INT_100mV">

<#if (MC.M1_DP_TOPOLOGY != "NONE")>
	<#list MC.M1_DriverProtection_FWConfig?split(",") as gdp1>
	<#if gdp1?contains("COMP1_ACTIVE=")> <#assign M1_STDRIVE102_COMP1_ACTIVE = (gdp1?keep_after("="))?replace("'", "")> </#if>
	<#if gdp1?contains("COMP2_ACTIVE=")> <#assign M1_STDRIVE102_COMP2_ACTIVE = (gdp1?keep_after("="))?replace("'", "")> </#if>
	<#if gdp1?contains("COMP3_ACTIVE=")> <#assign M1_STDRIVE102_COMP3_ACTIVE = (gdp1?keep_after("="))?replace("'", "")> </#if>
	<#if gdp1?contains("COMP1_INP=")> <#assign M1_STDRIVE102_COMP1_INP = (gdp1?keep_after("="))?replace("'", "")> </#if>
	<#if gdp1?contains("COMP2_INP=")> <#assign M1_STDRIVE102_COMP2_INP = (gdp1?keep_after("="))?replace("'", "")> </#if>
	<#if gdp1?contains("COMP3_INP=")> <#assign M1_STDRIVE102_COMP3_INP = (gdp1?keep_after("="))?replace("'", "")> </#if>
	<#if gdp1?contains("COMP1_INN=")> <#assign M1_STDRIVE102_COMP1_INN = (gdp1?keep_after("="))?replace("'", "")> </#if>
	<#if gdp1?contains("COMP2_INN=")> <#assign M1_STDRIVE102_COMP2_INN = (gdp1?keep_after("="))?replace("'", "")> </#if>
	<#if gdp1?contains("COMP3_INN=")> <#assign M1_STDRIVE102_COMP3_INN = (gdp1?keep_after("="))?replace("'", "")> </#if>
	<#if gdp1?contains("COMP1_DGT=")> <#assign M1_STDRIVE102_COMP1_DGT = (gdp1?keep_after("="))?replace("'", "")> </#if>
	<#if gdp1?contains("COMP2_DGT=")> <#assign M1_STDRIVE102_COMP2_DGT = (gdp1?keep_after("="))?replace("'", "")> </#if>
	<#if gdp1?contains("COMP3_DGT=")> <#assign M1_STDRIVE102_COMP3_DGT = (gdp1?keep_after("="))?replace("'", "")> </#if>
	<#if gdp1?contains("COMP_INT_REF=")> <#assign M1_STDRIVE102_COMP_INT_REF = (gdp1?keep_after("="))?replace("'", "")> </#if>
	</#list>
<#else>
	<#assign M1_STDRIVE102_COMP1_ACTIVE = "false">
	<#assign M1_STDRIVE102_COMP2_ACTIVE = "false">
	<#assign M1_STDRIVE102_COMP3_ACTIVE = "false">
</#if>

const STDRIVE102BP_defaultParams_t STDRIVE102BP_defaultParams_M1 =
{
	.spi=
	{
		.spi_hdl = ${MC.M1_COMPLEX_GATE_DRIVER_INTERFACE},
		.nCS_gpio_port = M1_SPI_NSS_GPIO_Port,
		.nCS_gpio_pin = M1_SPI_NSS_Pin,
	},
	.gd =
	{
		.igate_on = (stdrv102bp_gateLevels_t)${M1_STDRIVE102_IGATE},
		.tcc = (stdrv102bp_Timings_t)${M1_STDRIVE102_TCC},
		.eq_mode_sel = ${M1_STDRIVE102_EQMODE},
		.drv_mode_sel = ${M1_STDRIVE102_DRVMODE},
	},
	.uv =
	{
		.uvlo_sel = BP_UVLO_5V5,
		.pwrgood_sel = BP_PGOOD_7V75,
	},
	.vdsm =
	{
		.vds_soft_off = true,
		.vds_count_enable = true,
		.vds_disable = BP_VDS_TDIS_10ms,
		.vds_deglitch = ${M1_STDRIVE102_VDSDGT},
		.vds_counter = 0,
	},
	.afe_ch1 =
	{
		.pga_enable = true,
		.comp_enable = ${M1_STDRIVE102_COMP1_ACTIVE},
		.comp_pos_input = ${M1_STDRIVE102_COMP1_INP},
		.comp_neg_input = ${M1_STDRIVE102_COMP1_INN},
		.comp_inversion = false,
		.pga_gain = ${M1_STDRIVE102_PGA_GAIN},
		.comp_soft_off = true,
		.comp_fault_enable = ${M1_STDRIVE102_COMP1_ACTIVE},
		.comp_counter_enable = true,
		.comp_deglitch = ${M1_STDRIVE102_COMP1_DGT},
		.comp_disable = BP_COMP_DIST_1100us,
		.comp_counter = 0,
	},
    .afe_ch2 =
	{
		.pga_enable = true,
		.comp_enable = ${M1_STDRIVE102_COMP2_ACTIVE},
		.comp_pos_input = ${M1_STDRIVE102_COMP2_INP},
		.comp_neg_input = ${M1_STDRIVE102_COMP2_INN},
		.comp_inversion = false,
		.pga_gain = ${M1_STDRIVE102_PGA_GAIN},
		.comp_soft_off = true,
		.comp_fault_enable = ${M1_STDRIVE102_COMP2_ACTIVE},
		.comp_counter_enable = true,
		.comp_deglitch = ${M1_STDRIVE102_COMP2_DGT},
		.comp_disable = BP_COMP_DIST_1100us,
		.comp_counter = 0,
	},
    .afe_ch3 =
	{
		.pga_enable = true,
		.comp_enable = ${M1_STDRIVE102_COMP3_ACTIVE},
		.comp_pos_input = ${M1_STDRIVE102_COMP3_INP},
		.comp_neg_input = ${M1_STDRIVE102_COMP3_INN},
		.comp_inversion = false,
		.pga_gain = ${M1_STDRIVE102_PGA_GAIN},
		.comp_soft_off = true,
		.comp_fault_enable = ${M1_STDRIVE102_COMP3_ACTIVE},
		.comp_counter_enable = true,
		.comp_deglitch = ${M1_STDRIVE102_COMP3_DGT},
		.comp_disable = BP_COMP_DIST_1100us,
		.comp_counter = 0,
	},
	.afe_cm =
	{
		.pga_ref_sel = ${M1_STDRIVE102_PGA_REF_SEL},
		.comp_internal_th = ${M1_STDRIVE102_COMP_INT_REF},
		.driver_safe_state = BP_MOS_ALL_OFF,
	}

};
</#if>

<#if (MC.DRIVE_NUMBER != "1") && (MC.M2_PWM_DRIVER_PN == "STDRIVE102BP")>

<#assign M2_STDRIVE102_IGATE = "5">
<#assign M2_STDRIVE102_TCC = "5">
<#assign M2_STDRIVE102_EQMODE = "BP_IGATE_EQ_TCC_LOW_RANGE">
<#list MC.M2_PhaseVoltageGeneration_FWConfig?split(",") as phv2>
<#if phv2?contains("IGATE=")> <#assign M2_STDRIVE102_IGATE = (phv2?keep_after("="))?replace("'", "")> </#if>
<#if phv2?contains("TCC=")> <#assign M2_STDRIVE102_TCC = (phv2?keep_after("="))?replace("'", "")> </#if>
<#if phv2?contains("EQMODE=")> <#assign M2_STDRIVE102_EQMODE = (phv2?keep_after("="))?replace("'", "")> </#if>
</#list>

<#if ((M2_STDRIVE102_EQMODE == "BP_IGATE_EQ_TCC_LOW_RANGE") && (M2_STDRIVE102_TCC?eval == 15))> <#assign M2_STDRIVE102_VDSDGT = "BP_VDS_DGT_4us5">
<#elseif ((M2_STDRIVE102_EQMODE != "BP_IGATE_EQ_TCC_LOW_RANGE") && (M2_STDRIVE102_TCC?eval > 8) && (M2_STDRIVE102_TCC?eval <= 11))> <#assign M2_STDRIVE102_VDSDGT = "BP_VDS_DGT_4us5">
<#elseif ((M2_STDRIVE102_EQMODE != "BP_IGATE_EQ_TCC_LOW_RANGE") && (M2_STDRIVE102_TCC?eval > 11) && (M2_STDRIVE102_TCC?eval <= 13))> <#assign M2_STDRIVE102_VDSDGT = "BP_VDS_DGT_6us">
<#elseif ((M2_STDRIVE102_EQMODE != "BP_IGATE_EQ_TCC_LOW_RANGE") && (M2_STDRIVE102_TCC?eval > 13) && (M2_STDRIVE102_TCC?eval <= 15))> <#assign M2_STDRIVE102_VDSDGT = "BP_VDS_DGT_7us">
<#else> <#assign M2_STDRIVE102_VDSDGT = "BP_VDS_DGT_3us5">
</#if>

<#if (MC.M2_DRIVE_TYPE == "FOC")> <#assign M2_STDRIVE102_PGA_REF_SEL = "BP_PGA_HALF_VDD">
<#elseif (MC.M2_DRIVE_TYPE == "SIX_STEP")> <#assign M2_STDRIVE102_PGA_REF_SEL = "BP_PGA_NO_OFFSET">
<#else> <#assign M2_STDRIVE102_PGA_REF_SEL = "BP_PGA_NO_OFFSET">
</#if>

<#if (MC.M2_AMPLIFICATION_GAIN == "4")> <#assign M2_STDRIVE102_PGA_GAIN = "BP_PGA_GAIN_4">
<#elseif (MC.M2_AMPLIFICATION_GAIN == "8")> <#assign M2_STDRIVE102_PGA_GAIN = "BP_PGA_GAIN_8"> 
<#elseif (MC.M2_AMPLIFICATION_GAIN == "16")> <#assign M2_STDRIVE102_PGA_GAIN = "BP_PGA_GAIN_16"> 
<#elseif (MC.M2_AMPLIFICATION_GAIN == "32")> <#assign M2_STDRIVE102_PGA_GAIN = "BP_PGA_GAIN_32"> 
<#else> <#assign M2_STDRIVE102_PGA_GAIN = "BP_PGA_GAIN_32"> 
</#if>

<#if MC.M2_LOW_SIDE_SIGNALS_ENABLING == "LS_PWM_TIMER"> <#assign M2_STDRIVE102_DRVMODE = "BP_DIRECT_NO_WAIT">
<#elseif MC.M2_LOW_SIDE_SIGNALS_ENABLING == "ES_GPIO"> <#assign M2_STDRIVE102_DRVMODE = "BP_EN_IN">
<#else> <#assign M2_STDRIVE102_DRVMODE = "BP_EN_IN">
</#if>


<#assign M2_STDRIVE102_COMP1_ACTIVE = "true">
<#assign M2_STDRIVE102_COMP2_ACTIVE = "true">
<#assign M2_STDRIVE102_COMP3_ACTIVE = "true"> 
<#assign M2_STDRIVE102_COMP1_INP = "BP_COMP_DIRECT_INP">
<#assign M2_STDRIVE102_COMP2_INP = "BP_COMP_DIRECT_INP">
<#assign M2_STDRIVE102_COMP3_INP = "BP_COMP_DIRECT_INP">
<#assign M2_STDRIVE102_COMP1_INN = "BP_COMP_TH_EXT_CREF">
<#assign M2_STDRIVE102_COMP2_INN = "BP_COMP_TH_EXT_CREF">
<#assign M2_STDRIVE102_COMP3_INN = "BP_COMP_TH_EXT_CREF">
<#assign M2_STDRIVE102_COMP1_DGT = "BP_COMP_DGT_2500ns">
<#assign M2_STDRIVE102_COMP2_DGT = "BP_COMP_DGT_2500ns">
<#assign M2_STDRIVE102_COMP3_DGT = "BP_COMP_DGT_2500ns">
<#assign M2_STDRIVE102_COMP_INT_REF = "BP_COMP_TH_INT_100mV">

<#if (MC.M2_DP_TOPOLOGY != "NONE")>
	<#list MC.M2_DriverProtection_FWConfig?split(",") as gdp2>
	<#if gdp2?contains("COMP1_ACTIVE=")> <#assign M2_STDRIVE102_COMP1_ACTIVE = (gdp2?keep_after("="))?replace("'", "")> </#if>
	<#if gdp2?contains("COMP2_ACTIVE=")> <#assign M2_STDRIVE102_COMP2_ACTIVE = (gdp2?keep_after("="))?replace("'", "")> </#if>
	<#if gdp2?contains("COMP3_ACTIVE=")> <#assign M2_STDRIVE102_COMP3_ACTIVE = (gdp2?keep_after("="))?replace("'", "")> </#if>
	<#if gdp2?contains("COMP1_INP=")> <#assign M2_STDRIVE102_COMP1_INP = (gdp2?keep_after("="))?replace("'", "")> </#if>
	<#if gdp2?contains("COMP2_INP=")> <#assign M2_STDRIVE102_COMP2_INP = (gdp2?keep_after("="))?replace("'", "")> </#if>
	<#if gdp2?contains("COMP3_INP=")> <#assign M2_STDRIVE102_COMP3_INP = (gdp2?keep_after("="))?replace("'", "")> </#if>
	<#if gdp2?contains("COMP1_INN=")> <#assign M2_STDRIVE102_COMP1_INN = (gdp2?keep_after("="))?replace("'", "")> </#if>
	<#if gdp2?contains("COMP2_INN=")> <#assign M2_STDRIVE102_COMP2_INN = (gdp2?keep_after("="))?replace("'", "")> </#if>
	<#if gdp2?contains("COMP3_INN=")> <#assign M2_STDRIVE102_COMP3_INN = (gdp2?keep_after("="))?replace("'", "")> </#if>
	<#if gdp2?contains("COMP1_DGT=")> <#assign M2_STDRIVE102_COMP1_DGT = (gdp2?keep_after("="))?replace("'", "")> </#if>
	<#if gdp2?contains("COMP2_DGT=")> <#assign M2_STDRIVE102_COMP2_DGT = (gdp2?keep_after("="))?replace("'", "")> </#if>
	<#if gdp2?contains("COMP3_DGT=")> <#assign M2_STDRIVE102_COMP3_DGT = (gdp2?keep_after("="))?replace("'", "")> </#if>
	<#if gdp2?contains("COMP_INT_REF=")> <#assign M2_STDRIVE102_COMP_INT_REF = (gdp2?keep_after("="))?replace("'", "")> </#if>
	</#list>
<#else>
	<#assign M2_STDRIVE102_COMP1_ACTIVE = "false">
	<#assign M2_STDRIVE102_COMP2_ACTIVE = "false">
	<#assign M2_STDRIVE102_COMP3_ACTIVE = "false">
</#if>

const STDRIVE102BP_defaultParams_t STDRIVE102BP_defaultParams_M2 =
{
	.spi=
	{
		.spi_hdl = ${MC.M2_COMPLEX_GATE_DRIVER_INTERFACE},
		.nCS_gpio_port = M2_SPI_NSS_GPIO_Port,
		.nCS_gpio_pin = M2_SPI_NSS_Pin,
	},
	.gd =
	{
		.igate_on = (stdrv102bp_gateLevels_t)${M2_STDRIVE102_IGATE},
		.tcc = (stdrv102bp_Timings_t)${M2_STDRIVE102_TCC},
		.eq_mode_sel = ${M2_STDRIVE102_EQMODE},
		.drv_mode_sel = ${M2_STDRIVE102_DRVMODE},
	},
	.uv =
	{
		.uvlo_sel = BP_UVLO_5V5,
		.pwrgood_sel = BP_PGOOD_7V75,
	},
	.vdsm =
	{
		.vds_soft_off = true,
		.vds_count_enable = true,
		.vds_disable = BP_VDS_TDIS_10ms,
		.vds_deglitch = ${M2_STDRIVE102_VDSDGT},
		.vds_counter = 0,
	},
	.afe_ch1 =
	{
		.pga_enable = true,
		.comp_enable = ${M2_STDRIVE102_COMP1_ACTIVE},
		.comp_pos_input = ${M2_STDRIVE102_COMP1_INP},
		.comp_neg_input = ${M2_STDRIVE102_COMP1_INN},
		.comp_inversion = false,
		.pga_gain = ${M2_STDRIVE102_PGA_GAIN},
		.comp_soft_off = true,
		.comp_fault_enable = ${M2_STDRIVE102_COMP1_ACTIVE},
		.comp_counter_enable = true,
		.comp_deglitch = ${M2_STDRIVE102_COMP1_DGT},
		.comp_disable = BP_COMP_DIST_1100us,
		.comp_counter = 0,
	},
    .afe_ch2 =
	{
		.pga_enable = true,
		.comp_enable = ${M2_STDRIVE102_COMP2_ACTIVE},
		.comp_pos_input = ${M2_STDRIVE102_COMP2_INP},
		.comp_neg_input = ${M2_STDRIVE102_COMP2_INN},
		.comp_inversion = false,
		.pga_gain = ${M2_STDRIVE102_PGA_GAIN},
		.comp_soft_off = true,
		.comp_fault_enable = ${M2_STDRIVE102_COMP2_ACTIVE},
		.comp_counter_enable = true,
		.comp_deglitch = ${M2_STDRIVE102_COMP2_DGT},
		.comp_disable = BP_COMP_DIST_1100us,
		.comp_counter = 0,
	},
    .afe_ch3 =
	{
		.pga_enable = true,
		.comp_enable = ${M2_STDRIVE102_COMP3_ACTIVE},
		.comp_pos_input = ${M2_STDRIVE102_COMP3_INP},
		.comp_neg_input = ${M2_STDRIVE102_COMP3_INN},
		.comp_inversion = false,
		.pga_gain = ${M2_STDRIVE102_PGA_GAIN},
		.comp_soft_off = true,
		.comp_fault_enable = ${M2_STDRIVE102_COMP3_ACTIVE},
		.comp_counter_enable = true,
		.comp_deglitch = ${M2_STDRIVE102_COMP3_DGT},
		.comp_disable = BP_COMP_DIST_1100us,
		.comp_counter = 0,
	},
	.afe_cm =
	{
		.pga_ref_sel = ${M2_STDRIVE102_PGA_REF_SEL},
		.comp_internal_th = ${M2_STDRIVE102_COMP_INT_REF},
		.driver_safe_state = BP_MOS_ALL_OFF,
	}

};

</#if>




/************************ (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/