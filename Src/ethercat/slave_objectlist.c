#include "esc_coe.h"
#include "ethercat/utypes.h"

#include <stddef.h>

static const char acName1000[] = "Device Type";
static const char acName1008[] = "Device Name";
static const char acName1009[] = "Hardware Version";
static const char acName100A[] = "Software Version";
static const char acName1018[] = "Identity Object";
static const char acName1018_00[] = "Max SubIndex";
static const char acName1018_01[] = "Vendor ID";
static const char acName1018_02[] = "Product Code";
static const char acName1018_03[] = "Revision Number";
static const char acName1018_04[] = "Serial Number";
static const char acName1600[] = "Outputs";
static const char acName1600_00[] = "Max SubIndex";
static const char acName1600_01[] = "Controlword";
static const char acName1600_02[] = "Modes of Operation";
static const char acName1600_03[] = "LAN9252 GPIO Direction";
static const char acName1600_04[] = "LAN9252 GPO";
static const char acName1600_05[] = "MCU LED";
static const char acName1A00[] = "Inputs";
static const char acName1A00_00[] = "Max SubIndex";
static const char acName1A00_01[] = "Status Word";
static const char acName1A00_02[] = "Modes of Operation Display";
static const char acName1A00_03[] = "LAN9252 GPI";
static const char acName1A00_04[] = "Position Actual Value";
static const char acName1C00[] = "Sync Manager Communication Type";
static const char acName1C00_00[] = "Max SubIndex";
static const char acName1C00_01[] = "Communications Type SM0";
static const char acName1C00_02[] = "Communications Type SM1";
static const char acName1C00_03[] = "Communications Type SM2";
static const char acName1C00_04[] = "Communications Type SM3";
static const char acName1C12[] = "Sync Manager 2 PDO Assignment";
static const char acName1C12_00[] = "Max SubIndex";
static const char acName1C12_01[] = "PDO Mapping";
static const char acName1C13[] = "Sync Manager 3 PDO Assignment";
static const char acName1C13_00[] = "Max SubIndex";
static const char acName1C13_01[] = "PDO Mapping";
static const char acName6040[] = "Controlword";
static const char acName6041[] = "Statusword";
static const char acName6060[] = "Modes of Operation";
static const char acName6061[] = "Modes of Operation Display";
static const char acName6064[] = "Position Actual Value";
static const char acName6001[] = "LAN9252 Inputs";
static const char acName6001_00[] = "Max SubIndex";
static const char acName6001_01[] = "LAN9252 GPI";
static const char acName7001[] = "LAN9252 Outputs";
static const char acName7001_00[] = "Max SubIndex";
static const char acName7001_01[] = "LAN9252 GPIO Direction";
static const char acName7001_02[] = "LAN9252 GPO";
static const char acName7001_03[] = "MCU LED";

const _objd SDO1000[] = {
    {0x0, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1000, 0x01901389, NULL},
};
const _objd SDO1008[] = {
    {0x0, DTYPE_VISIBLE_STRING, 88, ATYPE_RO, acName1008, 0, "motor_driver"},
};
const _objd SDO1009[] = {
    {0x0, DTYPE_VISIBLE_STRING, 0, ATYPE_RO, acName1009, 0, "1.0"},
};
const _objd SDO100A[] = {
    {0x0, DTYPE_VISIBLE_STRING, 0, ATYPE_RO, acName100A, 0, "1.0"},
};
const _objd SDO1018[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1018_00, 4, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_01, 0x1337, NULL},
    {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_02, 0x00009252, NULL},
    {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_03, 0, NULL},
    {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_04, 0x00000000, NULL},
};
const _objd SDO1600[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1600_00, 5, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1600_01, 0x60400010, NULL},
    {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1600_02, 0x60600008, NULL},
    {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1600_03, 0x70010110, NULL},
    {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1600_04, 0x70010210, NULL},
    {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1600_05, 0x70010308, NULL},
};
const _objd SDO1A00[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A00_00, 4, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_01, 0x60410010, NULL},
    {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_02, 0x60610008, NULL},
    {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_03, 0x60010110, NULL},
    {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_04, 0x60640020, NULL},
};
const _objd SDO1C00[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_00, 4, NULL},
    {0x01, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_01, 1, NULL},
    {0x02, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_02, 2, NULL},
    {0x03, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_03, 3, NULL},
    {0x04, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_04, 4, NULL},
};
const _objd SDO1C12[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C12_00, 1, NULL},
    {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C12_01, 0x1600, NULL},
};
const _objd SDO1C13[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C13_00, 1, NULL},
    {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_01, 0x1A00, NULL},
};
const _objd SDO6040[] = {
    {0x00, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName6040, 0, &Obj.Parameters.ControlWord},
};
const _objd SDO6041[] = {
    {0x00, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName6041, 0, &Obj.Parameters.StatusWord},
};
const _objd SDO6060[] = {
    {0x00, DTYPE_INTEGER8, 8, ATYPE_RW, acName6060, 0, &Obj.Parameters.OperationMode},
};
const _objd SDO6061[] = {
    {0x00, DTYPE_INTEGER8, 8, ATYPE_RO, acName6061, 0, &Obj.Parameters.OperationModeDisplay},
};
const _objd SDO6064[] = {
    {0x00, DTYPE_INTEGER32, 32, ATYPE_RO, acName6064, 0, &Obj.Parameters.PositionActualValue},
};
const _objd SDO6001[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6001_00, 1, NULL},
    {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName6001_01, 0, &Obj.Parameters.Lan9252Gpi},
};
const _objd SDO7001[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName7001_00, 3, NULL},
    {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName7001_01, 0, &Obj.Parameters.Lan9252GpioDirection},
    {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName7001_02, 0, &Obj.Parameters.Lan9252Gpo},
    {0x03, DTYPE_UNSIGNED8, 8, ATYPE_RW, acName7001_03, 0, &Obj.Parameters.McuLed},
};

const _objectlist SDOobjects[] = {
    {0x1000, OTYPE_VAR, 0, 0, acName1000, SDO1000},
    {0x1008, OTYPE_VAR, 0, 0, acName1008, SDO1008},
    {0x1009, OTYPE_VAR, 0, 0, acName1009, SDO1009},
    {0x100A, OTYPE_VAR, 0, 0, acName100A, SDO100A},
    {0x1018, OTYPE_RECORD, 4, 0, acName1018, SDO1018},
    {0x1600, OTYPE_RECORD, 5, 0, acName1600, SDO1600},
    {0x1A00, OTYPE_RECORD, 4, 0, acName1A00, SDO1A00},
    {0x1C00, OTYPE_ARRAY, 4, 0, acName1C00, SDO1C00},
    {0x1C12, OTYPE_ARRAY, 1, 0, acName1C12, SDO1C12},
    {0x1C13, OTYPE_ARRAY, 1, 0, acName1C13, SDO1C13},
    {0x6040, OTYPE_VAR, 0, 0, acName6040, SDO6040},
    {0x6041, OTYPE_VAR, 0, 0, acName6041, SDO6041},
    {0x6060, OTYPE_VAR, 0, 0, acName6060, SDO6060},
    {0x6061, OTYPE_VAR, 0, 0, acName6061, SDO6061},
    {0x6064, OTYPE_VAR, 0, 0, acName6064, SDO6064},
    {0x6001, OTYPE_RECORD, 1, 0, acName6001, SDO6001},
    {0x7001, OTYPE_RECORD, 3, 0, acName7001, SDO7001},
    {0xffff, 0xff, 0xff, 0xff, NULL, NULL},
};
