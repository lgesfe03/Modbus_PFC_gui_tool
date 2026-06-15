import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
import struct
from typing import Callable, List, Optional
from enum import Enum, auto
import serial
from serial.tools import list_ports

DESTINATE_DEVICE_OPTIONS = [['Single PFC 0x03', 0x03], ['ControlBoard 0x04', 0x04], ['PSU 0x00', 0x00]]
Read_Addr_FW_version = "03 03 00 01 00 04"
Read_Addr_Output_Current = "03 03 00 61 00 10"
Read_Addr_CLA_heartbeat = "03 03 00 18 00 04"
Read_Addr_PWM_duty = "03 03 00 25 00 01"
Write_Addr_PWM_duty = "03 06 04 0C 00 02 00 00"
Read_Addr_ADC1 = "03 03 00 0C 00 0A"
Read_Addr_ADC2 = "03 03 00 0D 00 08"
Read_Addr_GPIO = "03 03 00 15 00 01"
Read_Addr_system_status = "03 03 00 1C 00 05"
Write_Addr_Output_Current = "03 06 04 48 00 02 00 01"
Write_Addr_Output_Voltage = "03 06 04 46 00 02 00 01"
Write_Addr_GPIO = "03 06 03 FC 00 01 00"
Read_Addr_Leg = "03 03 00 19 00 04"
Write_Addr_Leg = "03 06 04 00 00 04 00 00 00 00"
Read_Addr_status_fault_mode = "03 03 00 41 00 04"
Read_Addr_status_error_mode = "03 03 00 42 00 04"
Read_Addr_status_warning_mode = "03 03 00 43 00 04"
Read_Addr_status_working_mode = "03 03 00 45 00 01"
Write_Addr_Protect_Reset = "03 06 04 7F 00 01 00"
Write_Addr_Working_Mode = "03 06 04 2C 00 01 00"
Read_Addr_Output_Voltage = "03 03 00 5F 00 10"
Read_Addr_Input_Voltage = "03 03 00 60 00 02"
Read_Addr_Input_Current = "03 03 00 62 00 02"
Read_Addr_Output_Volt_Over_Setting = "03 03 00 87 00 08"
Read_Addr_Output_Curr_Over_Setting = "03 03 00 88 00 08"
Read_Addr_Temperature_Over_Setting = "03 03 00 89 00 08"

# Input limit
INPUT_CURRENT_MIN = 0
INPUT_CURRENT_MAX = 130
INPUT_VOLTAGE_MIN = 0
INPUT_VOLTAGE_MAX = 4800
# Lookup tables (same values as in the C code)
lut_adc = [521, 726, 1018, 1422, 1938, 2518, 3078, 3527, 3819, 3979, 4053]
lut_temp = [1500, 1310, 1120, 930, 740, 550, 360, 170, -20, -210, -400]
LUT_SIZE = len(lut_adc)

class _Working_Mode(Enum):
    A2D_SELF_TEST_MODE = 0
    A2D_WAIT_MODE = auto()
    A2D_IDLE_MODE = auto()
    A2D_INIT_MODE = auto()
    A2D_START_MODE = auto()
    A2D_RUN_MODE = auto()
    A2D_STOP_MODE = auto()
    A2D_FAULT_MODE = auto()
    A2D_TEST_INIT_MODE = auto()
    D2D_TEST_RUN_MODE = auto()

class _Relay_Mode(Enum):
    PFC_RELAY_OFF_MODE = 0
    PFC_RELAY_READY_WAIT_MODE= auto()
    PFC_RELAY_CLOSE_DELAY_MODE= auto()
    PFC_RELAY_CLOSE_SETTLE_MODE= auto()
    PFC_RELAY_ON_MODE= auto()
    PFC_RELAY_OPENING_MODE= auto()
    
FAULT_MASKS = {
    "PFC_IL1_TZ_OCP":   0x00000001,  # Bit 0
    "PFC_IL1_SW_RCP":   0x00000002,  # Bit 1
    "PFC_IL2_TZ_OCP":   0x00000004,  # Bit 2
    "PFC_IL2_SW_RCP":   0x00000008,  # Bit 3
    "PFC_VBUS_TZ_OVP":  0x00000010,  # Bit 4
    "PFC_VAC_OVP":      0x00000020,  # Bit 5
    "PFC_VAC_UFP":      0x00000040,  # Bit 6
    "PFC_VAC_OFP":      0x00000080,  # Bit 7
    "PFC_OTP_1":        0x00000100,# Bit 8
    "PFC_OTP_2":        0x00000200,# Bit 9
    "PFC_OTP_3":        0x00000400,# Bit 10
    "PFC_OTP_4":        0x00000800,# Bit 11
    "PFC_VAC_UVP":      0x00001000,# Bit 12
    "PFC_VBUS_SW_OVP":  0x00002000,# Bit 13
    "FAU_15":           0x00004000,# Bit 14
    "PFC_VBUS_UVP":     0x00008000,# Bit 15
}

ERROR_MASK = {
    "ERR_01_SCP":                      0x00000001,    #Bit 0
    "ERR_02_OPP_LV1":                  0x00000002,    #Bit 1
    "ERR_03_OPP_LV2":                  0x00000004,    #Bit 2
    "ERR_04_OTP":                      0x00000008,    #Bit 3
    "ERR_05_EEP_READ_CALI_NG":         0x00000010,    #Bit 4
    "ERR_06_EEP_READ_CONFIG_NG":       0x00000020,    #Bit 5
    "ERR_07_EEP_READ_EVENT_NG":        0x00000040,    #Bit 6
    "ERR_08_PSKILL":                   0x00000080,    #Bit 7

    "ERR_09_OCP":                      0x00000100,    #Bit 8
    "ERR_10_PFC_OVP":                  0x00000200,    #Bit 9
    "ERR_11_Check_CS_IOUT_AD":         0x00000400,    #Bit 10
    "ERR_12_Check_I_SHARE_IN_AD":      0x00000800,    #Bit 11
    "ERR_13_Check_PFC_VOUT_AD":        0x00001000,    #Bit 12
    "ERR_14_Check_VOUT_AD":            0x00002000,    #Bit 13
    "ERR_15_Check_REF_0V3_AD":         0x00004000,    #Bit 14
    "ERR_16_Check_NTC_AD_00":          0x00008000,    #Bit 15

    "ERR_17_Check_NTC_AD_01":          0x00010000,    #Bit 16
    "ERR_18_Check_NTC_AD_02":          0x00020000,    #Bit 17
    "ERR_19_Check_NTC_AD_03":          0x00040000,    #Bit 18
    "ERR_20_Check_NTC_AD_04":          0x00080000,    #Bit 19
    "ERR_21_Check_NTC_AD_05":          0x00100000,    #Bit 20
    "ERR_22_Check_ACDC_Status":        0x00200000,    #Bit 21
    "ERR_23_Check_IOUT_Sense1_AD":     0x00400000,    #Bit 22
    "ERR_24_Check_IOUT_Sense2_AD":     0x00800000,    #Bit 23

    "ERR_25_Check_REF_1V65_AD_1":      0x01000000,    #Bit 24
    "ERR_26_Check_REF_1V65_AD_2":      0x02000000,    #Bit 25
    "ERR_27_OPP_LV3":                  0x04000000,    #Bit 26
    "ERR_28_OPP_LV4":                  0x08000000,    #Bit 27
    "ERR_29_OPP_LV5":                  0x10000000,    #Bit 28
    "ERR_30_SR_MOS_SCP":               0x20000000,    #Bit 29
    "ERR_31":                          0x40000000,    #Bit 30
    "ERR_32":                          0x80000000,    #Bit 31
}

WARNING_MASK = {
    "WAR_01_UVP":                      0x00000001,    #Bit 0
    "WAR_02_FAN_NG":                   0x00000002,    #Bit 1
    "WAR_03_TO_MCU_UART_NG":           0x00000004,    #Bit 2
    "WAR_04_EEP_WRITE_NG":             0x00000008,    #Bit 3
    "WAR_05_TO_CB1_UART_NG":           0x00000010,    #Bit 2
    "WAR_06_EEP_READ_LENGTH_NG":       0x00000020,    #Bit 5
    "WAR_07_EEP_READ_DATA_NG":         0x00000040,    #Bit 6
    "WAR_08_EEP_CHECKSUM_NG":          0x00000080,    #Bit 7

    "WAR_09_EEP_READ_DATA_SHORT":      0x00000100,    #Bit 8
    "WAR_10_FAN1_NG":                  0x00000200,    #Bit 9
    "WAR_11_FAN2_NG":                  0x00000400,    #Bit 10
    "WAR_12_EEP_READ_CONFIG_NG":       0x00000800,    #Bit 11
    "WAR_13_EEP_READ_CALI_NG":         0x00001000,    #Bit 12
    "WAR_14_EEP_READ_EVENT_NG":        0x00002000,    #Bit 13
    "WAR_15":                          0x00004000,    #Bit 14
    "WAR_16":                          0x00008000,    #Bit 15

    "WAR_17":                          0x00010000,    #Bit 16
    "WAR_18":                          0x00020000,    #Bit 17
    "WAR_19":                          0x00040000,    #Bit 18
    "WAR_20":                          0x00080000,    #Bit 19
    "WAR_21":                          0x00100000,    #Bit 20
    "WAR_22":                          0x00200000,    #Bit 21
    "WAR_23":                          0x00400000,    #Bit 22
    "WAR_24":                          0x00800000,    #Bit 23

    "WAR_25":                          0x01000000,    #Bit 24
    "WAR_26":                          0x02000000,    #Bit 25
    "WAR_27":                          0x04000000,    #Bit 26
    "WAR_28":                          0x08000000,    #Bit 27
    "WAR_29":                          0x10000000,    #Bit 28
    "WAR_30":                          0x20000000,    #Bit 29
    "WAR_31":                          0x40000000,    #Bit 30
    "WAR_32":                          0x80000000,    #Bit 31
}

def decode_faults(fault_code: int) -> list:
    """Return list of active fault names for given fault_code."""
    return [name for name, mask in FAULT_MASKS.items() if (fault_code & mask) != 0]

def decode_errors(error_code: int) -> list:
    return [name for name, mask in ERROR_MASK.items() if (error_code & mask) != 0]

def decode_warnings(warning_code: int) -> list:
    return [name for name, mask in WARNING_MASK.items() if (warning_code & mask) != 0]


def debug_print_tx(frame: bytes) -> None:
    print(f"Tx frame: {format_hex(frame)}")
def debug_print_rx(frame: bytes) -> None:
    print(f"Rx frame: {format_hex(frame)}")
def build_modbus_crc(data: bytes) -> bytes:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return bytes((crc & 0xFF, (crc >> 8) & 0xFF))


def format_hex(data: bytes) -> str:
    if not data:
        return ""
    return " ".join(f"{byte:02X}" for byte in data)

def parse_version_read_response(data: bytes) -> tuple[str, str]:
    if len(data) < 6+4:
        return "N/A", "N/A"
    version_byte0 = str(data[9])
    version_byte1 = str(data[8])
    version_byte2 = str(data[7])
    version_byte3 = str(data[6])
    return version_byte0, version_byte1, version_byte2, version_byte3
def parse_current_read_response(data: bytes) -> tuple[str, str]:
    if len(data) < 6+6:
        return "N/A", "N/A"
    float_bytes = data[8:12]
    current_ref_value = struct.unpack(">f", float_bytes)[0]
    TTPLPFC_ac_cur_ref_pu = f"{current_ref_value:.6f}".rstrip("0").rstrip(".")

    float_bytes = data[12:16]
    current_ref_value = struct.unpack(">f", float_bytes)[0]
    TTPLPFC_ac_cur_ref_inst_pu  = f"{current_ref_value:.6f}".rstrip("0").rstrip(".")

    current_cmd = str((data[6] << 8) + data[7])
    return TTPLPFC_ac_cur_ref_pu, TTPLPFC_ac_cur_ref_inst_pu, current_cmd
def parse_pwm_duty_read_response(data: bytes) -> tuple[str, str]:
    if len(data) < 6+2:
        return "N/A", "N/A"
    if len(data) < 6+8:
        pwm_duty_FAH = str(data[7])
        return pwm_duty_FAH, "N/A", "N/A", "N/A"
    pwm_duty_FAH = str(data[13])
    pwm_duty_FAL = str(data[11])
    pwm_duty_FBH = str(data[9])
    pwm_duty_FBL = str(data[7])
    return pwm_duty_FAH, pwm_duty_FAL, pwm_duty_FBH, pwm_duty_FBL
def convert_vac_voltage(u16_adc):
    multiply_offset = u16_adc*0.2585 - 530.2
    return str(f"{(multiply_offset):.3f}")
def convert_il_amp(u16_adc):
    multiply_offset = u16_adc*0.0222- 45.537
    return str(f"{(multiply_offset):.3f}")
def convert_vbus_voltage(u16_adc):
    multiply_offset = u16_adc*0.12924
    return str(f"{(multiply_offset):.3f}")
def convert_1v65_voltage(u16_adc):
    multiply_offset = u16_adc/4095*3.3
    return str(f"{(multiply_offset):.3f}")
def parse_adc1_read_response(data: bytes) -> tuple[str, str]:
    if len(data) < 6+10:
        return "N/A", "N/A"
    u16_adc_vac = data[6]<<8 | data[7]
    str_vac_voltage = convert_vac_voltage(u16_adc_vac)
    
    u16_adc_il1 = data[8]<<8 | data[9]
    str_il1_amp = convert_il_amp(u16_adc_il1)
    u16_adc_il2 = data[10]<<8 | data[11]
    str_il2_amp = convert_il_amp(u16_adc_il2)

    u16_adc_vbus = data[12]<<8 | data[13]
    str_vbus_voltage = convert_vbus_voltage(u16_adc_vbus)

    u16_adc_1v65 = data[14]<<8 | data[15]
    str_1v65_voltage = convert_1v65_voltage(u16_adc_1v65)
    return str_1v65_voltage, str_vbus_voltage, str_il2_amp, str_il1_amp, str_vac_voltage
def linear_interpolate(x: int, x0: int, x1: int, y0: float, y1: float) -> float:
    """Performs linear interpolation between (x0, y0) and (x1, y1) for x."""
    if x1 == x0:
        return float(y0)  # avoid division by zero; should not happen with valid LUT
    return y0 + (float(x - x0) * (y1 - y0)) / (x1 - x0)

def get_temperature(adc_value: int) -> float:
    """
    Convert ADC value to temperature using the LUT and linear interpolation.
    Returns temperature in 0.1 °C units (same as lut_temp).Returns -999.0 on error.
    """
    if adc_value <= lut_adc[0]:
        return float(lut_temp[0])   # under minimum
    if adc_value >= lut_adc[-1]:
        return float(lut_temp[-1])  # over maximum

    for i in range(LUT_SIZE - 1):
        if lut_adc[i] <= adc_value <= lut_adc[i + 1]:
            return linear_interpolate(adc_value, lut_adc[i], lut_adc[i + 1],
                                      float(lut_temp[i]), float(lut_temp[i + 1]))

    return -999.0  # should not be reached
def convert_ntc_01_degreeC(u16_adc):
    temp_01c = get_temperature(u16_adc)    # temperature in 0.1 °C units
    temp_c = temp_01c / 10.0           # convert to °C
    return str(f"{(temp_c):.1f}")
def parse_adc2_read_response(data: bytes) -> tuple[str, str]:
    if len(data) < 6+8:
        return "N/A", "N/A"
    u16_adc_PFC_S_TEMP = data[6]<<8 | data[7]
    str_PFC_S_TEMP = convert_ntc_01_degreeC(u16_adc_PFC_S_TEMP)
    
    u16_adc_Inlet_TEMP = data[8]<<8 | data[9]
    str_Inlet_TEMP = convert_ntc_01_degreeC(u16_adc_Inlet_TEMP)

    u16_adc_LLC_TEMP = data[10]<<8 | data[11]
    str_LLC_TEMP = convert_ntc_01_degreeC(u16_adc_LLC_TEMP)

    u16_adc_PFC_F_TEMP = data[12]<<8 | data[13]
    str_PFC_F_TEMP = convert_ntc_01_degreeC(u16_adc_PFC_F_TEMP)
    return str_PFC_F_TEMP, str_LLC_TEMP, str_Inlet_TEMP, str_PFC_S_TEMP

def parse_gpio_read_response(data: bytes) -> tuple[str, str]:
    if len(data) < 6+1:
        return "N/A", "N/A"
    
    bit_Fan1_RPM = (data[6] >> 0) & 0x01
    bit_DI_LLC_PwrGood = (data[6] >> 1) & 0x01
    bit_DO_RELAY = (data[6] >> 2) & 0x01
    bit_DO_AC_LOSS = (data[6] >> 3) & 0x01
    bit_DO_NotifyLLC = (data[6] >> 4) & 0x01
    return str(bit_Fan1_RPM), str(bit_DI_LLC_PwrGood), str(bit_DO_RELAY), str(bit_DO_AC_LOSS), str(bit_DO_NotifyLLC)
def parse_system_status_read_response(data: bytes) -> tuple[str, str]:
    if len(data) < 6+5:
        return "N/A", "N/A"
    system_status_relay = data[6]
    system_status = data[7]
    system_status_HwControlStatus  = data[8]
    system_status_aux  = data[9]
    system_status_NormalTripSource  = data[10]
    return str(system_status_relay), str(system_status), str(system_status_HwControlStatus), str(system_status_aux), str(system_status_NormalTripSource)
def parse_leg_read_response(data: bytes) -> tuple[str, str]:
    if len(data) < 6+4:
        return "N/A", "N/A"
    HFLegA_EN = (data[6] >> 0) & 0x01
    HFLegB_EN = (data[6] >> 1) & 0x01
    OPL_LFLeg_EN = (data[6] >> 2) & 0x01
    OPL_HFLeg_SR_EN = (data[6] >> 3) & 0x01
    return HFLegA_EN, HFLegB_EN, OPL_LFLeg_EN, OPL_HFLeg_SR_EN

def parse_u16_read_response(data: bytes) -> tuple[str, str]:
    if len(data) < 6+4:
        return "N/A", "N/A"
    u16 = data[6]<<8 | data[7]
    return u16
def parse_u16_index_read_response(data: bytes, idx: int) -> tuple[str, str]:
    if len(data) < 6+4:
        return "N/A", "N/A"
    u16 = data[idx]<<8 | data[idx+1]
    return u16

def parse_u32_read_response(data: bytes) -> tuple[str, str]:
    if len(data) < 6+4:
        return "N/A", "N/A"
    u32 = data[6]<<24 |data[7]<<16 |data[8]<<8 | data[9]
    return u32

def parse_working_mode_read_response(data: bytes) -> tuple[str, str]:
    if len(data) < 6+1:
        return "N/A", "N/A"
    working_mode = data[6]
    return working_mode
    
class ModbusGuiApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Modbus PFC GUI Tool")
        self.root.geometry("1024x888")

        self.serial_port: Optional[serial.Serial] = None
        self.port_var = tk.StringVar()
        self.device_byte0_var = tk.StringVar()
        self.status_var = tk.StringVar(value="Disconnected")
        self.variable_init()

        self._build_ui()
        self.refresh_ports()

    def variable_init(self) -> None:
        self.row_accumulate = 0
        self.column_accumulate = 0

        self.response_fw_version_read_all_var = tk.StringVar(value="")
        self.input_current_w_var = tk.StringVar(value="0")
        self.input_output_voltage_w_var = tk.StringVar(value="0")
        self.response_current_r_float1_var = tk.StringVar(value="N/A")
        self.response_current_r_float2_var = tk.StringVar(value="N/A")
        self.response_output_voltage_r_float1_var = tk.StringVar(value="N/A")
        self.response_output_voltage_r_float2_var = tk.StringVar(value="N/A")
        self.response_cla_heartbeat_r_u32_var = tk.StringVar(value="N/A")
        self.response_current_r_cmd_var = tk.StringVar(value="N/A")
        self.response_voltage_r_cmd_var = tk.StringVar(value="N/A")
        self.response_pwm_FAH_duty_r_var = tk.StringVar(value="")
        self.response_pwm_FAL_duty_r_var = tk.StringVar(value="")
        self.response_pwm_FBH_duty_r_var = tk.StringVar(value="")
        self.response_pwm_FBL_duty_r_var = tk.StringVar(value="")
        self.input_pwm_duty_w_var = tk.StringVar(value="0")

        self.response_adc1_v165_r_var = tk.StringVar(value="")
        self.response_adc1_vbus_r_var = tk.StringVar(value="")
        self.response_adc1_il1_r_var = tk.StringVar(value="")
        self.response_adc1_il2_r_var = tk.StringVar(value="")
        self.response_adc1_vac_r_var = tk.StringVar(value="")

        self.response_adc2_PFC_S_TEMP_r_var = tk.StringVar(value="")
        self.response_adc2_LLC_TEMP_r_var = tk.StringVar(value="")
        self.response_adc2_Inlet_TEMP_r_var = tk.StringVar(value="")
        self.response_adc2_PFC_F_TEMP_r_var = tk.StringVar(value="")

        self.response_gpio_Fan1_RPM_r_var = tk.StringVar(value="")
        self.response_gpio_DI_LLC_r_var = tk.StringVar(value="")
        self.response_gpio_DO_RELAY_r_var = tk.StringVar(value="")
        self.response_gpio_DO_AC_LOSS_r_var = tk.StringVar(value="")
        self.response_gpio_DO_NotifyLLC_r_var = tk.StringVar(value="")
        self.gpio_do_relay_w_var = tk.IntVar(value=0)
        self.gpio_do_ac_loss_w_var = tk.IntVar(value=0)
        self.gpio_do_notifyllc_w_var = tk.IntVar(value=0)

        self.response_system_status_relay_r_var = tk.StringVar(value="Relay:")
        self.response_system_system_status_r_var = tk.StringVar(value="")
        self.response_system_status_HwControlStatus_r_var = tk.StringVar(value="")
        self.response_system_status_aux_r_var = tk.StringVar(value="")
        self.response_system_status_NormalTripSource_r_var = tk.StringVar(value="")

        self.response_leg_HFLegA_EN_r_var = tk.StringVar(value="")
        self.response_leg_HFLegB_EN_r_var = tk.StringVar(value="")
        self.response_leg_OPL_LFLeg_EN_r_var = tk.StringVar(value="")
        self.response_leg_OPL_HFLeg_SR_EN_r_var = tk.StringVar(value="")
        self.leg_HFLegA_EN_w_var = tk.IntVar(value=0)
        self.leg_HFLegB_EN_w_var = tk.IntVar(value=0)
        self.leg_OPL_LFLeg_EN_w_var = tk.IntVar(value=0)
        self.leg_OPL_HFLeg_SR_EN_w_var = tk.IntVar(value=0)

        self.response_Fault_Code_r_var = tk.StringVar(value="")
        self.response_Error_Code_r_var = tk.StringVar(value="")
        self.response_Warning_Code_r_var = tk.StringVar(value="")
        self.response_Working_Mode_r_var = tk.StringVar(value="")
        self.input_protect_reset_w_var = tk.IntVar(value=0)
        self.input_working_mode_w_var = tk.StringVar(value="0")
        self.response_voltage_in_r_var = tk.StringVar(value="")
        self.response_current_in_r_var = tk.StringVar(value="")

        self.response_voltage_over_r_var = tk.StringVar(value="")
        self.response_current_over_r_var = tk.StringVar(value="")
        self.response_temperature_over_r_var = tk.StringVar(value="")
    def variable_reset(self) -> None:
        self.response_fw_version_read_all_var.set("")
        # self.input_current_w_var.set("")
        self.response_current_r_float1_var.set("")
        self.response_current_r_float2_var.set("")
        self.response_output_voltage_r_float1_var.set("")
        self.response_output_voltage_r_float2_var.set("")
        self.response_cla_heartbeat_r_u32_var.set("")
        self.response_current_r_cmd_var.set("")
        self.response_voltage_r_cmd_var.set("")
        self.response_pwm_FAH_duty_r_var.set("")
        self.response_pwm_FAL_duty_r_var.set("")
        self.response_pwm_FBH_duty_r_var.set("")
        self.response_pwm_FBL_duty_r_var.set("")
        # self.input_pwm_duty_w_var.set("0")
        self.response_adc1_v165_r_var.set("")
        self.response_adc1_vbus_r_var.set("")
        self.response_adc1_il1_r_var.set("")
        self.response_adc1_il2_r_var.set("")
        self.response_adc1_vac_r_var.set("")
        self.response_adc2_PFC_S_TEMP_r_var.set("")
        self.response_adc2_LLC_TEMP_r_var.set("")
        self.response_adc2_Inlet_TEMP_r_var.set("")
        self.response_adc2_PFC_F_TEMP_r_var.set("")
        self.response_gpio_Fan1_RPM_r_var.set("")
        self.response_gpio_DI_LLC_r_var.set("")
        self.response_gpio_DO_RELAY_r_var.set("")
        self.response_gpio_DO_AC_LOSS_r_var.set("")
        self.response_gpio_DO_NotifyLLC_r_var.set("")
        # self.gpio_do_relay_w_var.set(0)
        # self.gpio_do_ac_loss_w_var.set(0)
        # self.gpio_do_notifyllc_w_var.set(0)
        self.response_system_status_relay_r_var.set("Relay:")
        self.response_system_system_status_r_var.set("")
        self.response_system_status_HwControlStatus_r_var.set("")
        self.response_system_status_aux_r_var.set("")
        self.response_system_status_NormalTripSource_r_var.set("")
        self.response_leg_HFLegA_EN_r_var.set("")
        self.response_leg_HFLegB_EN_r_var.set("")
        self.response_leg_OPL_LFLeg_EN_r_var.set("")
        self.response_leg_OPL_HFLeg_SR_EN_r_var.set("")
        # self.leg_HFLegA_EN_w_var.set(0)
        # self.leg_HFLegB_EN_w_var.set(0)
        # self.leg_OPL_LFLeg_EN_w_var.set(0)
        # self.leg_OPL_HFLeg_SR_EN_w_var.set(0)
        self.response_Fault_Code_r_var.set("")
        self.response_Error_Code_r_var.set("")
        self.response_Warning_Code_r_var.set("")
        self.response_Working_Mode_r_var.set("")
        # self.input_protect_reset_w_var.set(0)
        # self.input_working_mode_w_var.set("0")
        self.response_voltage_in_r_var.set("")
        self.response_current_in_r_var.set("")
        
        self.response_voltage_over_r_var.set("")
        self.response_current_over_r_var.set("")
        self.response_temperature_over_r_var.set("")

    def row_accumulator_add(self) -> None:
        self.row_accumulate += 1
    def row_accumulator_get(self) -> None:
        return self.row_accumulate
    def row_accumulator_clear(self) -> None:
        self.row_accumulate = 0
    def column_accumulator_get(self) -> None:
        self.column_accumulate += 1
        return self.column_accumulate - 1
    def column_accumulator_clear(self) -> None:
        self.column_accumulate = 0

    def _build_ui(self) -> None:
        outer_root = ttk.Frame(self.root, padding=12)
        outer_root.pack(fill="both", expand=True)

        top = ttk.LabelFrame(outer_root, text="Connection", padding=12)

        top.pack(fill="x")

        ttk.Label(top, text="COM Port").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, state="readonly", width=22)
        self.port_combo.grid(row=0, column=1, padx=(8, 12), sticky="w")

        ttk.Button(top, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=(0, 8))
        ttk.Button(top, text="Connect", command=self.connect_port).grid(row=0, column=3, padx=(0, 8))
        ttk.Button(top, text="Disconnect", command=self.disconnect_port).grid(row=0, column=4)

        ttk.Label(top, text="DeviceBytes0").grid(row=0, column=5, sticky="w")
        self.device_combo = ttk.Combobox(top, textvariable=self.device_byte0_var, state="readonly", width=22, values=[item[0] for item in DESTINATE_DEVICE_OPTIONS])
        self.device_combo.current(0) # default select first item
        self.device_combo.grid(row=0, column=6, padx=(8, 12), sticky="w")
        # self.device_combo.bind("<<ComboboxSelected>>", self.on_device_select)


        ttk.Button(top, text="ClearMsg", command=self.clear_messages).grid(row=0, column=7)
        ttk.Label(top, textvariable=self.status_var, foreground="#005f8d").grid(
            row=1, column=0, columnspan=5, sticky="w", pady=(10, 0)
        )
        # Tabs setting
        notebook = ttk.Notebook(outer_root)
        notebook.pack(fill="both", expand=True, pady=(12, 0))

        tab_basic = ttk.Frame(notebook)
        tab_lab = ttk.Frame(notebook)
        
        notebook.add(tab_basic, text="Basic")
        notebook.add(tab_lab, text="Lab")

    # Tab basic
        root = tab_basic

        # Scroll function
        scroll_canvas = tk.Canvas(root, highlightthickness=0, borderwidth=0)
        scroll_bar = ttk.Scrollbar(root, orient="vertical", command=scroll_canvas.yview)
        scroll_canvas.configure(yscrollcommand=scroll_bar.set)
        scroll_canvas.pack(side="left", fill="both", expand=True, pady=(12, 0))
        scroll_bar.pack(side="right", fill="y", pady=(12, 0))

        scroll_content = ttk.Frame(scroll_canvas)
        content_window = scroll_canvas.create_window((0, 0), window=scroll_content, anchor="nw")

        def _update_scrollregion(_event=None) -> None:
            scroll_canvas.configure(scrollregion=scroll_canvas.bbox("all"))

        def _sync_width(event) -> None:
            scroll_canvas.itemconfigure(content_window, width=event.width)

        def _on_mousewheel(event) -> str:
            delta = event.delta
            if delta == 0:
                return "break"
            scroll_canvas.yview_scroll(int(-1 * (delta / 120)), "units")
            return "break"

        def _on_mousewheel_up(_event) -> str:
            scroll_canvas.yview_scroll(-1, "units")
            return "break"

        def _on_mousewheel_down(_event) -> str:
            scroll_canvas.yview_scroll(1, "units")
            return "break"

        scroll_content.bind("<Configure>", _update_scrollregion)
        scroll_canvas.bind("<Configure>", _sync_width)
        scroll_canvas.bind_all("<MouseWheel>", _on_mousewheel)
        scroll_canvas.bind_all("<Button-4>", _on_mousewheel_up)
        scroll_canvas.bind_all("<Button-5>", _on_mousewheel_down)

        root = scroll_content
    # get version
        self.row_accumulator_clear()
        self.column_accumulator_clear()
        f_get_version = ttk.LabelFrame(root, text="FW_version", padding=12)
        f_get_version.pack(fill="x", pady=(12, 0))
        ttk.Button(f_get_version, text="R_Version", command=self.send_r_version_command, width=12).grid(
            row=0, column=self.column_accumulator_get(), sticky="w"
        )
        ttk.Label(f_get_version, text="Version.").grid(
            row=0, column=self.column_accumulator_get(), sticky="w", pady=(8, 0))
        ttk.Entry(f_get_version, textvariable=self.response_fw_version_read_all_var, width=18, state="readonly").grid(
            row=0, column=self.column_accumulator_get(), padx=(12, 8), pady=(8, 0), sticky="w"
        )
        # get CLA heartbeat
        ttk.Button(f_get_version, text="R_CLA_heartbeat", command=self.send_r_cla_heartbeat_command, width=12).grid(
            row=0, column=self.column_accumulator_get(), sticky="w"
        )
        ttk.Label(f_get_version, text="test_cla_heartbeat").grid(
            row=0, column=self.column_accumulator_get(), sticky="w", pady=(8, 0))
        ttk.Entry(f_get_version, textvariable=self.response_cla_heartbeat_r_u32_var, width=18, state="readonly").grid(
            row=0, column=self.column_accumulator_get(), padx=(12, 8), pady=(8, 0), sticky="w"
        )
        f_get_version.columnconfigure(10, weight=1)
    # Lab6 Voltage related
        self.column_accumulator_clear()
        f_voltage_lab6 = ttk.LabelFrame(root, text="Lab6 Voltage", padding=12)
        f_voltage_lab6.pack(fill="x", pady=(12, 0))        
        # get voltage
        ttk.Button(f_voltage_lab6, text="R_voltage", command=self.send_r_voltage_out_command, width=12).grid(
            row=0, column=self.column_accumulator_get(), sticky="w"
        )
        ttk.Label(f_voltage_lab6, text="TTPLPFC_vBusRef_pu").grid(
            row=0, column=self.column_accumulator_get(), sticky="w", pady=(8, 0))
        ttk.Entry(f_voltage_lab6, textvariable=self.response_output_voltage_r_float1_var, width=18, state="readonly").grid(
            row=0, column=self.column_accumulator_get(), padx=(12, 8), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_voltage_lab6, text="TTPLPFC_vBus_sensed_Volts").grid(
            row=0, column=self.column_accumulator_get(), sticky="w", pady=(8, 0))
        ttk.Entry(f_voltage_lab6, textvariable=self.response_output_voltage_r_float2_var, width=18, state="readonly").grid(
            row=0, column=self.column_accumulator_get(), padx=(12, 8), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_voltage_lab6, text="voltage_cmd_from_modbus").grid(
            row=0, column=self.column_accumulator_get(), sticky="w", pady=(8, 0))
        ttk.Entry(f_voltage_lab6, textvariable=self.response_voltage_r_cmd_var, width=18, state="readonly").grid(
            row=0, column=self.column_accumulator_get(), padx=(8, 0), pady=(8, 0), sticky="w"
        )
        # set voltage
        self.column_accumulator_clear()
        ttk.Button(f_voltage_lab6, text="W_voltage", command=self.send_w_output_voltage_command, width=12).grid(
            row=1, column=0, sticky="w"
        )
        self.voltage_spin = ttk.Spinbox(f_voltage_lab6, from_=INPUT_VOLTAGE_MIN, to=INPUT_VOLTAGE_MAX, textvariable=self.input_output_voltage_w_var, width=10)
        self.voltage_spin.grid(
            row=1, column=1, padx=(8, 12), sticky="w")
        f_voltage_lab6.columnconfigure(10, weight=1)
    # get ADCs
        f_adc_r = ttk.LabelFrame(root, text="ADC1 Read", padding=12)
        f_adc_r.pack(fill="x", pady=(12, 0))
        ttk.Button(f_adc_r, text="R_ADC1", command=self.send_r_adc1_command, width=12).grid(
            row=0, column=0, sticky="w"
        )
        #ADC1
        ttk.Label(f_adc_r, text="Vac").grid(
            row=0, column=1, sticky="w", pady=(8, 0))
        ttk.Entry(f_adc_r, textvariable=self.response_adc1_vac_r_var, width=12, state="readonly").grid(
            row=0, column=2, padx=(8, 0), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_adc_r, text="iL1").grid(
            row=0, column=3, sticky="w", pady=(8, 0))
        ttk.Entry(f_adc_r, textvariable=self.response_adc1_il1_r_var, width=12, state="readonly").grid(
            row=0, column=4, padx=(8, 0), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_adc_r, text="iL2").grid(
            row=0, column=5, sticky="w", pady=(8, 0))
        ttk.Entry(f_adc_r, textvariable=self.response_adc1_il2_r_var, width=12, state="readonly").grid(
            row=0, column=6, padx=(8, 0), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_adc_r, text="Vbus").grid(
            row=0, column=7, sticky="w", pady=(8, 0))
        ttk.Entry(f_adc_r, textvariable=self.response_adc1_vbus_r_var, width=12, state="readonly").grid(
            row=0, column=8, padx=(8, 0), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_adc_r, text="1.65V").grid(
            row=0, column=9, sticky="w", pady=(8, 0))
        ttk.Entry(f_adc_r, textvariable=self.response_adc1_v165_r_var, width=12, state="readonly").grid(
            row=0, column=10, padx=(12, 8), pady=(8, 0), sticky="w"
        )
        #ADC2
        ttk.Button(f_adc_r, text="R_ADC2", command=self.send_r_adc2_command, width=12).grid(
            row=1, column=0, sticky="w"
        )

        ttk.Label(f_adc_r, text="PFC_S_TEMP").grid(
            row=1, column=1, sticky="w", pady=(8, 0))
        ttk.Entry(f_adc_r, textvariable=self.response_adc2_PFC_S_TEMP_r_var, width=12, state="readonly").grid(
            row=1, column=2, padx=(8, 0), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_adc_r, text="LLC_TEMP").grid(
            row=1, column=3, sticky="w", pady=(8, 0))
        ttk.Entry(f_adc_r, textvariable=self.response_adc2_LLC_TEMP_r_var, width=12, state="readonly").grid(
            row=1, column=4, padx=(8, 0), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_adc_r, text="Inlet_TEMP").grid(
            row=1, column=5, sticky="w", pady=(8, 0))
        ttk.Entry(f_adc_r, textvariable=self.response_adc2_Inlet_TEMP_r_var, width=12, state="readonly").grid(
            row=1, column=6, padx=(8, 0), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_adc_r, text="PFC_F_TEMP").grid(
            row=1, column=7, sticky="w", pady=(8, 0))
        ttk.Entry(f_adc_r, textvariable=self.response_adc2_PFC_F_TEMP_r_var, width=12, state="readonly").grid(
            row=1, column=8, padx=(8, 0), pady=(8, 0), sticky="w"
        )

        f_adc_r.columnconfigure(11, weight=1)
    # get GPIO
        f_gpio = ttk.LabelFrame(root, text="GPIO Related", padding=12)
        f_gpio.pack(fill="x", pady=(12, 0))
        ttk.Button(f_gpio, text="R_GPIO", command=self.send_r_gpio_command, width=12).grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(f_gpio, text="DO_NotifyLLC").grid(
            row=0, column=1, sticky="w", pady=(8, 0))
        ttk.Entry(f_gpio, textvariable=self.response_gpio_DO_NotifyLLC_r_var, width=12, state="readonly").grid(
            row=0, column=2, padx=(12, 8), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_gpio, text="DO_AC_LOSS").grid(
            row=0, column=3, sticky="w", pady=(8, 0))
        ttk.Entry(f_gpio, textvariable=self.response_gpio_DO_AC_LOSS_r_var, width=12, state="readonly").grid(
            row=0, column=4, padx=(8, 0), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_gpio, text="DO_RELAY").grid(
            row=0, column=5, sticky="w", pady=(8, 0))
        ttk.Entry(f_gpio, textvariable=self.response_gpio_DO_RELAY_r_var, width=12, state="readonly").grid(
            row=0, column=6, padx=(8, 0), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_gpio, text="DI_LLC").grid(
            row=0, column=7, sticky="w", pady=(8, 0))
        ttk.Entry(f_gpio, textvariable=self.response_gpio_DI_LLC_r_var, width=12, state="readonly").grid(
            row=0, column=8, padx=(8, 0), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_gpio, text="Fan1_RPM").grid(
            row=0, column=9, sticky="w", pady=(8, 0))
        ttk.Entry(f_gpio, textvariable=self.response_gpio_Fan1_RPM_r_var, width=12, state="readonly").grid(
            row=0, column=10, padx=(8, 0), pady=(8, 0), sticky="w"
        )

    # set GPIO
        ttk.Button(f_gpio, text="W_GPIO", command=self.send_w_gpio_command, width=12).grid(
            row=1, column=0, sticky="w")
        ttk.Label(f_gpio, text="DO_NotifyLLC").grid(
            row=1, column=1, sticky="w", pady=(8, 0))
        ttk.Checkbutton(f_gpio, variable=self.gpio_do_notifyllc_w_var, onvalue=1, offvalue=0,).grid(
            row=1, column=2, sticky="w", pady=(8, 0))
        
        ttk.Label(f_gpio, text="DO_AC_LOSS").grid(
            row=1, column=3, sticky="w", pady=(8, 0))
        ttk.Checkbutton(f_gpio,variable=self.gpio_do_ac_loss_w_var,onvalue=1,offvalue=0,).grid(
            row=1, column=4, sticky="w", pady=(8, 0))
        
        ttk.Label(f_gpio, text="DO_RELAY").grid(
            row=1, column=5, sticky="w", pady=(8, 0))
        ttk.Checkbutton(f_gpio,variable=self.gpio_do_relay_w_var,onvalue=1,offvalue=0,).grid(
            row=1, column=6, sticky="w", pady=(8, 0))
        f_gpio.columnconfigure(10, weight=1)
    # status related
        f_status = ttk.LabelFrame(root, text="Status Related", padding=12)
        f_status.pack(fill="x", pady=(12, 0))
        self.row_accumulator_clear()
        self.column_accumulator_clear()
        ttk.Button(f_status, text="R_FAULT", command=self.send_r_fault_code_command, width=12).grid(
            row=self.row_accumulator_get(), column=self.column_accumulator_get(), sticky="w"
        )
        ttk.Entry(f_status, textvariable=self.response_Fault_Code_r_var, width=26, state="readonly").grid(
            row=self.row_accumulator_get(), column=self.column_accumulator_get(), padx=(12, 8), pady=(8, 0), sticky="w"
        )

        ttk.Button(f_status, text="R_ERROR", command=self.send_r_error_code_command, width=12).grid(
            row=self.row_accumulator_get(), column=self.column_accumulator_get(), sticky="w"
        )
        ttk.Entry(f_status, textvariable=self.response_Error_Code_r_var, width=12, state="readonly").grid(
            row=self.row_accumulator_get(), column=self.column_accumulator_get(), padx=(8, 0), pady=(8, 0), sticky="w"
        )
        ttk.Button(f_status, text="R_WARNING", command=self.send_r_warning_code_command, width=12).grid(
            row=self.row_accumulator_get(), column=self.column_accumulator_get(), sticky="w"
        )
        ttk.Entry(f_status, textvariable=self.response_Warning_Code_r_var, width=12, state="readonly").grid(
            row=self.row_accumulator_get(), column=self.column_accumulator_get(), padx=(8, 0), pady=(8, 0), sticky="w"
        )

        self.column_accumulator_clear()
        self.row_accumulator_add()
        ttk.Button(f_status, text="R_SystemStatus", command=self.send_r_system_status_command, width=12).grid(
            row=self.row_accumulator_get(), column=self.column_accumulator_get(), sticky="w"
        )
        # ttk.Label(f_status, text="Relay").grid(
        #     row=self.row_accumulator_get(), column=self.column_accumulator_get(), sticky="w", pady=(8, 0))
        ttk.Entry(f_status, textvariable=self.response_system_status_relay_r_var, width=26, state="readonly").grid(
            row=self.row_accumulator_get(), column=self.column_accumulator_get(), padx=(8, 0), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_status, text="system_system_status").grid(
            row=self.row_accumulator_get(), column=self.column_accumulator_get(), sticky="w", pady=(8, 0))
        ttk.Entry(f_status, textvariable=self.response_system_system_status_r_var, width=12, state="readonly").grid(
            row=self.row_accumulator_get(), column=self.column_accumulator_get(), padx=(8, 0), pady=(8, 0), sticky="w"
        )        
        ttk.Label(f_status, text="HwControlStatus").grid(
            row=self.row_accumulator_get(), column=self.column_accumulator_get(), sticky="w", pady=(8, 0))
        ttk.Entry(f_status, textvariable=self.response_system_status_HwControlStatus_r_var, width=12, state="readonly").grid(
            row=self.row_accumulator_get(), column=self.column_accumulator_get(), padx=(8, 0), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_status, text="aux").grid(
            row=self.row_accumulator_get(), column=self.column_accumulator_get(), sticky="w", pady=(8, 0))
        ttk.Entry(f_status, textvariable=self.response_system_status_aux_r_var, width=12, state="readonly").grid(
            row=self.row_accumulator_get(), column=self.column_accumulator_get(), padx=(8, 0), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_status, text="NormalTripSource").grid(
            row=self.row_accumulator_get(), column=self.column_accumulator_get(), sticky="w", pady=(8, 0))
        ttk.Entry(f_status, textvariable=self.response_system_status_NormalTripSource_r_var, width=12, state="readonly").grid(
            row=self.row_accumulator_get(), column=self.column_accumulator_get(), padx=(8, 0), pady=(8, 0), sticky="w"
        )

        self.column_accumulator_clear()
        self.row_accumulator_add()
        ttk.Button(f_status, text="R_Work_mode", command=self.send_r_working_code_command, width=14).grid(
            row=self.row_accumulator_get(), column=self.column_accumulator_get(), sticky="w"
        )
        ttk.Entry(f_status, textvariable=self.response_Working_Mode_r_var, width=20, state="readonly").grid(
            row=self.row_accumulator_get(), column=self.column_accumulator_get(), padx=(8, 0), pady=(8, 0), sticky="w"
        )
        #write working mode
        ttk.Button(f_status, text="W_Work_mode", command=self.send_w_working_mode_command, width=12).grid(
            row=self.row_accumulator_get(), column=self.column_accumulator_get(), sticky="w"
        )
        self.work_mode_spin = ttk.Spinbox(f_status, from_=0, to=9, textvariable=self.input_working_mode_w_var, width=10)
        self.work_mode_spin.grid(
            row=self.row_accumulator_get(), column=self.column_accumulator_get(), padx=(8, 12), sticky="w")
        #write protect reset
        ttk.Button(f_status, text="W_Protect_reset", command=self.send_w_protect_reset_command, width=14).grid(
            row=self.row_accumulator_get(), column=self.column_accumulator_get(), sticky="w")
        ttk.Checkbutton(f_status, variable=self.input_protect_reset_w_var, onvalue=1, offvalue=0,).grid(
            row=self.row_accumulator_get(), column=self.column_accumulator_get(), sticky="w", pady=(8, 0))
        
        self.column_accumulator_clear()
        self.row_accumulator_add()
        #read Voltage, Current data which converted by PFC
        ttk.Button(f_status, text="R_V_out_100mV", command=self.send_r_voltage_out_command, width=12).grid(
            row=self.row_accumulator_get(), column=self.column_accumulator_get(), sticky="w"
        )
        ttk.Entry(f_status, textvariable=self.response_voltage_r_cmd_var, width=12, state="readonly").grid(
            row=self.row_accumulator_get(), column=self.column_accumulator_get(), padx=(12, 8), pady=(8, 0), sticky="w"
        )

        ttk.Button(f_status, text="R_V_in_100mV", command=self.send_r_voltage_in_command, width=14).grid(
            row=self.row_accumulator_get(), column=self.column_accumulator_get(), sticky="w"
        )
        ttk.Entry(f_status, textvariable=self.response_voltage_in_r_var, width=12, state="readonly").grid(
            row=self.row_accumulator_get(), column=self.column_accumulator_get(), padx=(8, 0), pady=(8, 0), sticky="w"
        )
        ttk.Button(f_status, text="R_C_in_10mA", command=self.send_r_current_in_command, width=12).grid(
            row=self.row_accumulator_get(), column=self.column_accumulator_get(), sticky="w"
        )
        ttk.Entry(f_status, textvariable=self.response_current_in_r_var, width=12, state="readonly").grid(
            row=self.row_accumulator_get(), column=self.column_accumulator_get(), padx=(8, 0), pady=(8, 0), sticky="w"
        )       
        f_status.columnconfigure(12, weight=1)

    # Protect related 
        f_Protect = ttk.LabelFrame(root, text="Protect Related", padding=12)
        f_Protect.pack(fill="x", pady=(12, 0))
        ttk.Button(f_Protect, text="R_V_Over_1V", command=self.send_r_voltage_over_command, width=12).grid(
            row=0, column=0, sticky="w"
        )
        ttk.Entry(f_Protect, textvariable=self.response_voltage_over_r_var, width=12, state="readonly").grid(
            row=0, column=1, padx=(12, 8), pady=(8, 0), sticky="w"
        )

        ttk.Button(f_Protect, text="R_C_Over_1A", command=self.send_r_current_over_command, width=12).grid(
            row=0, column=2, sticky="w"
        )
        ttk.Entry(f_Protect, textvariable=self.response_current_over_r_var, width=12, state="readonly").grid(
            row=0, column=3, padx=(8, 0), pady=(8, 0), sticky="w"
        )
        ttk.Button(f_Protect, text="R_T_over_0.1deg", command=self.send_r_temperature_over_command, width=12).grid(
            row=0, column=4, sticky="w"
        )
        ttk.Entry(f_Protect, textvariable=self.response_temperature_over_r_var, width=12, state="readonly").grid(
            row=0, column=5, padx=(8, 0), pady=(8, 0), sticky="w"
        )
        f_Protect.columnconfigure(10, weight=1)
    # Tab Lab ###############################
        root = tab_lab
        # leg related 
        f_leg = ttk.LabelFrame(root, text="Leg Related", padding=12)
        f_leg.pack(fill="x", pady=(12, 0))
        ttk.Button(f_leg, text="R_leg", command=self.send_r_leg_command, width=12).grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(f_leg, text="HFLegA_EN").grid(
            row=0, column=1, sticky="w", pady=(8, 0))
        ttk.Entry(f_leg, textvariable=self.response_leg_HFLegA_EN_r_var, width=18, state="readonly").grid(
            row=0, column=2, padx=(12, 8), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_leg, text="HFLegB_EN").grid(
            row=0, column=3, sticky="w", pady=(8, 0))
        ttk.Entry(f_leg, textvariable=self.response_leg_HFLegB_EN_r_var, width=18, state="readonly").grid(
            row=0, column=4, padx=(8, 0), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_leg, text="OPL_LFLeg_EN").grid(
            row=0, column=5, sticky="w", pady=(8, 0))
        ttk.Entry(f_leg, textvariable=self.response_leg_OPL_LFLeg_EN_r_var, width=18, state="readonly").grid(
            row=0, column=6, padx=(8, 0), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_leg, text="OPL_HFLeg_SR_EN").grid(
            row=0, column=7, sticky="w", pady=(8, 0))
        ttk.Entry(f_leg, textvariable=self.response_leg_OPL_HFLeg_SR_EN_r_var, width=18, state="readonly").grid(
            row=0, column=8, padx=(8, 0), pady=(8, 0), sticky="w"
        )
        #set leg
        ttk.Button(f_leg, text="W_leg", command=self.send_w_leg_command, width=12).grid(
            row=1, column=0, sticky="w")
        ttk.Label(f_leg, text="HFLegA_EN").grid(
            row=1, column=1, sticky="w", pady=(8, 0))
        ttk.Checkbutton(f_leg, variable=self.leg_HFLegA_EN_w_var, onvalue=1, offvalue=0,).grid(
            row=1, column=2, sticky="w", pady=(8, 0))
        
        ttk.Label(f_leg, text="HFLegB_EN").grid(
            row=1, column=3, sticky="w", pady=(8, 0))
        ttk.Checkbutton(f_leg,variable=self.leg_HFLegB_EN_w_var,onvalue=1,offvalue=0,).grid(
            row=1, column=4, sticky="w", pady=(8, 0))
        
        ttk.Label(f_leg, text="OPL_LFLeg_EN").grid(
            row=1, column=5, sticky="w", pady=(8, 0))
        ttk.Checkbutton(f_leg,variable=self.leg_OPL_LFLeg_EN_w_var,onvalue=1,offvalue=0,).grid(
            row=1, column=6, sticky="w", pady=(8, 0))
        
        ttk.Label(f_leg, text="OPL_HFLeg_SR_EN").grid(
            row=1, column=7, sticky="w", pady=(8, 0))
        ttk.Checkbutton(f_leg,variable=self.leg_OPL_HFLeg_SR_EN_w_var,onvalue=1,offvalue=0,).grid(
            row=1, column=8, sticky="w", pady=(8, 0))
        f_leg.columnconfigure(10, weight=1)

        # PWM duty related
        f_pwm_duty = ttk.LabelFrame(root, text="Lab3 PWM duty", padding=12)
        f_pwm_duty.pack(fill="x", pady=(12, 0))
        #get pwm duty
        ttk.Button(f_pwm_duty, text="R_PWM duty", command=self.send_r_pwm_duty_command, width=12).grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(f_pwm_duty, text="PWM-FAH").grid(
            row=0, column=1, sticky="w", pady=(8, 0))
        ttk.Entry(f_pwm_duty, textvariable=self.response_pwm_FAH_duty_r_var, width=18, state="readonly").grid(
            row=0, column=2, padx=(8, 0), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_pwm_duty, text="PWM-FAL").grid(
            row=0, column=3, sticky="w", pady=(8, 0))
        ttk.Entry(f_pwm_duty, textvariable=self.response_pwm_FAL_duty_r_var, width=18, state="readonly").grid(
            row=0, column=4, padx=(8, 0), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_pwm_duty, text="PWM-FBH").grid(
            row=0, column=5, sticky="w", pady=(8, 0))
        ttk.Entry(f_pwm_duty, textvariable=self.response_pwm_FBH_duty_r_var, width=18, state="readonly").grid(
            row=0, column=6, padx=(8, 0), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_pwm_duty, text="PWM-FBL").grid(
            row=0, column=7, sticky="w", pady=(8, 0))
        ttk.Entry(f_pwm_duty, textvariable=self.response_pwm_FBL_duty_r_var, width=18, state="readonly").grid(
            row=0, column=8, padx=(8, 0), pady=(8, 0), sticky="w"
        )
        #set pwm duty
        ttk.Button(f_pwm_duty, text="W_PWM_duty", command=self.send_w_pwm_duty_command, width=12).grid(
            row=1, column=0, sticky="w"
        )
        self.pwm_duty_spin = ttk.Spinbox(f_pwm_duty, from_=0, to=255, textvariable=self.input_pwm_duty_w_var, width=10)
        self.pwm_duty_spin.grid(
            row=1, column=1, padx=(8, 12), sticky="w")
        
        f_pwm_duty.columnconfigure(10, weight=1)
        
    # Lab4 Current related
        self.column_accumulator_clear()
        f_current_lab4 = ttk.LabelFrame(root, text="Lab4 Current", padding=12)
        f_current_lab4.pack(fill="x", pady=(12, 0))        
        # get current
        ttk.Button(f_current_lab4, text="R_current", command=self.send_r_current_command, width=12).grid(
            row=0, column=self.column_accumulator_get(), sticky="w"
        )
        ttk.Label(f_current_lab4, text="TTPLPFC_ac_cur_ref_pu").grid(
            row=0, column=self.column_accumulator_get(), sticky="w", pady=(8, 0))
        ttk.Entry(f_current_lab4, textvariable=self.response_current_r_float1_var, width=18, state="readonly").grid(
            row=0, column=self.column_accumulator_get(), padx=(12, 8), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_current_lab4, text="TTPLPFC_ac_cur_ref_inst_pu").grid(
            row=0, column=self.column_accumulator_get(), sticky="w", pady=(8, 0))        
        ttk.Entry(f_current_lab4, textvariable=self.response_current_r_float2_var, width=18, state="readonly").grid(
            row=0, column=self.column_accumulator_get(), padx=(12, 8), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_current_lab4, text="current_cmd_from_modbus").grid(
            row=0, column=self.column_accumulator_get(), sticky="w", pady=(8, 0))
        ttk.Entry(f_current_lab4, textvariable=self.response_current_r_cmd_var, width=18, state="readonly").grid(
            row=0, column=self.column_accumulator_get(), padx=(8, 0), pady=(8, 0), sticky="w"
        )
        # set current
        ttk.Button(f_current_lab4, text="W_TTPLPFC_ac_cur_ref_inst_pu", command=self.send_w_current_command, width=30).grid(
            row=1, column=0, sticky="w"
        )
        self.current_spin = ttk.Spinbox(f_current_lab4, from_=INPUT_CURRENT_MIN, to=INPUT_CURRENT_MAX, textvariable=self.input_current_w_var, width=10)
        self.current_spin.grid(
            row=1, column=1, padx=(8, 12), sticky="w")
        f_current_lab4.columnconfigure(10, weight=1)
    # Lab5 Current related
        self.column_accumulator_clear()
        f_current_lab5 = ttk.LabelFrame(root, text="Lab5 Current", padding=12)
        f_current_lab5.pack(fill="x", pady=(12, 0))        
        # get current
        ttk.Button(f_current_lab5, text="R_current", command=self.send_r_current_command, width=12).grid(
            row=0, column=self.column_accumulator_get(), sticky="w"
        )
        ttk.Label(f_current_lab5, text="TTPLPFC_ac_cur_ref_pu").grid(
            row=0, column=self.column_accumulator_get(), sticky="w", pady=(8, 0))
        ttk.Entry(f_current_lab5, textvariable=self.response_current_r_float1_var, width=18, state="readonly").grid(
            row=0, column=self.column_accumulator_get(), padx=(12, 8), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_current_lab5, text="TTPLPFC_ac_cur_ref_inst_pu").grid(
            row=0, column=self.column_accumulator_get(), sticky="w", pady=(8, 0))
        ttk.Entry(f_current_lab5, textvariable=self.response_current_r_float2_var, width=18, state="readonly").grid(
            row=0, column=self.column_accumulator_get(), padx=(12, 8), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_current_lab5, text="current_cmd_from_modbus").grid(
            row=0, column=self.column_accumulator_get(), sticky="w", pady=(8, 0))
        ttk.Entry(f_current_lab5, textvariable=self.response_current_r_cmd_var, width=18, state="readonly").grid(
            row=0, column=self.column_accumulator_get(), padx=(8, 0), pady=(8, 0), sticky="w"
        )
        # set current
        self.column_accumulator_clear()
        ttk.Button(f_current_lab5, text="W_TTPLPFC_ac_cur_ref_pu", command=self.send_w_current_command, width=30).grid(
            row=1, column=0, sticky="w"
        )
        self.current_spin = ttk.Spinbox(f_current_lab5, from_=INPUT_CURRENT_MIN, to=INPUT_CURRENT_MAX, textvariable=self.input_current_w_var, width=10)
        self.current_spin.grid(
            row=1, column=1, padx=(8, 12), sticky="w")
        f_current_lab5.columnconfigure(10, weight=1)
    def refresh_ports(self) -> None:
        ports = [port.device for port in list_ports.comports()]
        ports = sorted(ports)
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])
        elif not ports:
            self.port_var.set("")
        self.status_var.set("Ports refreshed")

    def connect_port(self) -> None:
        port_name = self.port_var.get().strip()
        if not port_name:
            messagebox.showwarning("No port", "Please select a COM port.")
            return

        self.disconnect_port()

        try:
            self.serial_port = serial.Serial(
                port=port_name,
                baudrate=115200,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,
                write_timeout=1,
            )
        except serial.SerialException as exc:
            self.serial_port = None
            messagebox.showerror("Connect failed", str(exc))
            self.status_var.set("Connect failed")
            return

        self.status_var.set(f"Connected: {port_name}")

    def disconnect_port(self) -> None:
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
            finally:
                self.serial_port = None
                self.status_var.set("Disconnected")
        else:
            self.serial_port = None
    def clear_messages(self) -> None:
        self.variable_reset()
    def fill_bytes0_device(self, request: bytearray) -> None:
        value_selected = self.device_combo.get()
        value_maps = {item[0]: item[1] for item in DESTINATE_DEVICE_OPTIONS}
        value_maps_selected = value_maps.get(value_selected)

        request[0] = int(value_maps_selected)
    def send_r_version_command(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not connected", "Please connect to a COM port first.")
            return
        request = bytearray.fromhex(Read_Addr_FW_version)
        self.fill_bytes0_device(request)
        frame = bytes(request) + build_modbus_crc(bytes(request))
        debug_print_tx(frame)
        threading.Thread(
            target=self._send_frame_worker,
            args=(frame, "Command sent", self.handle_parse_version_read_response),
            daemon=True,
        ).start()

    def send_w_current_command(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not connected", "Please connect to a COM port first.")
            return

        try:
            current_value = int(self.input_current_w_var.get().strip())
        except ValueError:
            messagebox.showwarning("Invalid value", f"must be an integer between {INPUT_CURRENT_MIN} and {INPUT_CURRENT_MAX}.")
            return

        if not INPUT_CURRENT_MIN <= current_value <= INPUT_CURRENT_MAX:
            messagebox.showwarning("Invalid value", f"must be an integer between {INPUT_CURRENT_MIN} and {INPUT_CURRENT_MAX}.")
            return

        request = bytearray.fromhex(Write_Addr_Output_Current)
        self.fill_bytes0_device(request)
        # Fill the 8th byte (index 7) before appending CRC, per SetCurrentCmd[7].
        request[7] = current_value
        frame = bytes(request) + build_modbus_crc(bytes(request))
        debug_print_tx(frame)
        threading.Thread(
            target=self._send_frame_worker,
            args=(frame, "W_current sent", self._handle_parse_current_write_response),
            daemon=True,
        ).start()
    def send_w_output_voltage_command(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not connected", "Please connect to a COM port first.")
            return

        try:
            voltage_value = int(self.input_output_voltage_w_var.get().strip())
        except ValueError:
            messagebox.showwarning("Invalid value", f"must be an integer between {INPUT_VOLTAGE_MIN} and {INPUT_VOLTAGE_MAX}.")
            return

        if not INPUT_VOLTAGE_MIN <= voltage_value <= INPUT_VOLTAGE_MAX:
            messagebox.showwarning("Invalid value", f"must be an integer between {INPUT_VOLTAGE_MIN} and {INPUT_VOLTAGE_MAX}.")
            return

        request = bytearray.fromhex(Write_Addr_Output_Voltage)
        self.fill_bytes0_device(request)
        # Fill the 8th byte (index 7) before appending CRC, per SetvoltageCmd[7].
        request[6] = (voltage_value >> 8) & 0xFF
        request[7] = voltage_value & 0xFF
        frame = bytes(request) + build_modbus_crc(bytes(request))
        debug_print_tx(frame)
        threading.Thread(
            target=self._send_frame_worker,
            args=(frame, "W_voltage sent", self._handle_parse_voltage_write_response),
            daemon=True,
        ).start()

    def send_w_pwm_duty_command(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not connected", "Please connect to a COM port first.")
            return

        try:
            duty_value = int(self.input_pwm_duty_w_var.get().strip())
        except ValueError:
            messagebox.showwarning("Invalid value", "must be an integer between 0 and 100.")
            return

        if not 0 <= duty_value <= 100:
            messagebox.showwarning("Invalid value", "must be an integer between 0 and 100.")
            return

        request = bytearray.fromhex(Write_Addr_PWM_duty)
        self.fill_bytes0_device(request)
        request[7] = duty_value
        frame = bytes(request) + build_modbus_crc(bytes(request))
        debug_print_tx(frame)
        threading.Thread(
            target=self._send_frame_worker,
            args=(frame, "W_pwm_duty sent", self._handle_parse_pwm_duty_write_response),
            daemon=True,
        ).start()
    
    def send_r_current_command(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not connected", "Please connect to a COM port first.")
            return

        request = bytearray.fromhex(Read_Addr_Output_Current)
        self.fill_bytes0_device(request)
        frame = bytes(request) + build_modbus_crc(bytes(request))
        debug_print_tx(frame)
        threading.Thread(
            target=self._send_frame_worker,
            args=(frame, "R_current sent", self._handle_current_read_response),
            daemon=True,
        ).start()
    def send_r_cla_heartbeat_command(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not connected", "Please connect to a COM port first.")
            return

        request = bytearray.fromhex(Read_Addr_CLA_heartbeat)
        self.fill_bytes0_device(request)
        frame = bytes(request) + build_modbus_crc(bytes(request))
        debug_print_tx(frame)
        threading.Thread(
            target=self._send_frame_worker,
            args=(frame, "R_current sent", self._handle_cla_heartbeat_read_response),
            daemon=True,
        ).start()
    def send_r_pwm_duty_command(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not connected", "Please connect to a COM port first.")
            return

        request = bytearray.fromhex(Read_Addr_PWM_duty)
        self.fill_bytes0_device(request)
        frame = bytes(request) + build_modbus_crc(bytes(request))

        debug_print_tx(frame)
        threading.Thread(
            target=self._send_frame_worker,
            args=(frame, "R_PWM duty sent", self._handle_pwm_duty_read_response),
            daemon=True,
        ).start()
    def send_r_adc1_command(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not connected", "Please connect to a COM port first.")
            return

        request = bytearray.fromhex(Read_Addr_ADC1)
        self.fill_bytes0_device(request)
        frame = bytes(request) + build_modbus_crc(bytes(request))

        debug_print_tx(frame)
        threading.Thread(
            target=self._send_frame_worker,
            args=(frame, "R_ADC1 sent", self._handle_adc1_read_response),
            daemon=True,
        ).start()
    def send_r_adc2_command(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not connected", "Please connect to a COM port first.")
            return

        request = bytearray.fromhex(Read_Addr_ADC2)
        self.fill_bytes0_device(request)
        frame = bytes(request) + build_modbus_crc(bytes(request))

        debug_print_tx(frame)
        threading.Thread(
            target=self._send_frame_worker,
            args=(frame, "R_ADC2 sent", self._handle_adc2_read_response),
            daemon=True,
        ).start()
    def send_r_gpio_command(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not connected", "Please connect to a COM port first.")
            return

        request = bytearray.fromhex(Read_Addr_GPIO)
        self.fill_bytes0_device(request)
        frame = bytes(request) + build_modbus_crc(bytes(request))

        debug_print_tx(frame)
        threading.Thread(
            target=self._send_frame_worker,
            args=(frame, "R_GPIO sent", self._handle_gpio_read_response),
            daemon=True,
        ).start()

    def send_w_gpio_command(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not connected", "Please connect to a COM port first.")
            return

        gpio_value = (
            (self.gpio_do_relay_w_var.get() & 0x01)
            | ((self.gpio_do_ac_loss_w_var.get() & 0x01) << 1)
            | ((self.gpio_do_notifyllc_w_var.get() & 0x01) << 2)
        )

        request = bytearray.fromhex(Write_Addr_GPIO)
        self.fill_bytes0_device(request)
        request[6] = gpio_value
        frame = bytes(request) + build_modbus_crc(bytes(request))
        debug_print_tx(frame)
        threading.Thread(
            target=self._send_frame_worker,
            args=(frame, "W_GPIO sent", self._handle_gpio_write_response),
            daemon=True,
        ).start()
    def send_r_system_status_command(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not connected", "Please connect to a COM port first.")
            return

        request = bytearray.fromhex(Read_Addr_system_status)
        self.fill_bytes0_device(request)
        frame = bytes(request) + build_modbus_crc(bytes(request))

        debug_print_tx(frame)
        threading.Thread(
            target=self._send_frame_worker,
            args=(frame, "R_GPIO sent", self._handle_system_status_read_response),
            daemon=True,
        ).start()
    def send_r_leg_command(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not connected", "Please connect to a COM port first.")
            return

        request = bytearray.fromhex(Read_Addr_Leg)
        self.fill_bytes0_device(request)
        frame = bytes(request) + build_modbus_crc(bytes(request))

        debug_print_tx(frame)
        threading.Thread(
            target=self._send_frame_worker,
            args=(frame, "R_GPIO sent", self._handle_leg_read_response),
            daemon=True,
        ).start()
    
    def send_w_leg_command(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not connected", "Please connect to a COM port first.")
            return

        leg_value = (
            (self.leg_HFLegA_EN_w_var.get() & 0x01)
            | ((self.leg_HFLegB_EN_w_var.get() & 0x01) << 1)
            | ((self.leg_OPL_LFLeg_EN_w_var.get() & 0x01) << 2)
            | ((self.leg_OPL_HFLeg_SR_EN_w_var.get() & 0x01) << 3)
        )

        request = bytearray.fromhex(Write_Addr_Leg)
        self.fill_bytes0_device(request)
        request[6] = leg_value
        frame = bytes(request) + build_modbus_crc(bytes(request))
        debug_print_tx(frame)
        threading.Thread(
            target=self._send_frame_worker,
            args=(frame, "W_GPIO sent", self._handle_leg_write_response),
            daemon=True,
        ).start()
    def send_w_protect_reset_command(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not connected", "Please connect to a COM port first.")
            return

        protect_reset_value = (
            (self.input_protect_reset_w_var.get() & 0x01)
        )

        request = bytearray.fromhex(Write_Addr_Protect_Reset)
        self.fill_bytes0_device(request)
        request[6] = protect_reset_value
        frame = bytes(request) + build_modbus_crc(bytes(request))
        debug_print_tx(frame)
        threading.Thread(
            target=self._send_frame_worker,
            args=(frame, "W_GPIO sent", self._handle_protect_reset_write_response),
            daemon=True,
        ).start()
    def send_w_working_mode_command(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not connected", "Please connect to a COM port first.")
            return

        try:
            work_mode_value = int(self.input_working_mode_w_var.get().strip())
        except ValueError:
            messagebox.showwarning("Invalid value", "must be an integer between 0 and 9.")
            return

        if not 0 <= work_mode_value <= 9:
            messagebox.showwarning("Invalid value", "must be an integer between 0 and 9.")
            return

        request = bytearray.fromhex(Write_Addr_Working_Mode)
        self.fill_bytes0_device(request)
        # Fill the 8th byte (index 7) before appending CRC
        request[6] = work_mode_value
        frame = bytes(request) + build_modbus_crc(bytes(request))
        debug_print_tx(frame)
        threading.Thread(
            target=self._send_frame_worker,
            args=(frame, "W_working_mode sent", self._handle_working_mode_write_response),
            daemon=True,
        ).start()
    def send_r_fault_code_command(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not connected", "Please connect to a COM port first.")
            return
        request = bytearray.fromhex(Read_Addr_status_fault_mode)
        self.fill_bytes0_device(request)
        frame = bytes(request) + build_modbus_crc(bytes(request))

        debug_print_tx(frame)
        threading.Thread(
            target=self._send_frame_worker,
            args=(frame, "R_fault_code sent", self._handle_fault_code_read_response),
            daemon=True,
        ).start()
    def send_r_error_code_command(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not connected", "Please connect to a COM port first.")
            return
        request = bytearray.fromhex(Read_Addr_status_error_mode)
        self.fill_bytes0_device(request)
        frame = bytes(request) + build_modbus_crc(bytes(request))

        debug_print_tx(frame)
        threading.Thread(
            target=self._send_frame_worker,
            args=(frame, "R_fault_code sent", self._handle_error_code_read_response),
            daemon=True,
        ).start()
    def send_r_warning_code_command(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not connected", "Please connect to a COM port first.")
            return
        request = bytearray.fromhex(Read_Addr_status_warning_mode)
        self.fill_bytes0_device(request)
        frame = bytes(request) + build_modbus_crc(bytes(request))

        debug_print_tx(frame)
        threading.Thread(
            target=self._send_frame_worker,
            args=(frame, "R_fault_code sent", self._handle_warning_code_read_response),
            daemon=True,
        ).start()
    def send_r_working_code_command(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not connected", "Please connect to a COM port first.")
            return
        request = bytearray.fromhex(Read_Addr_status_working_mode)
        self.fill_bytes0_device(request)
        frame = bytes(request) + build_modbus_crc(bytes(request))

        debug_print_tx(frame)
        threading.Thread(
            target=self._send_frame_worker,
            args=(frame, "R_fault_code sent", self._handle_workingmode_read_response),
            daemon=True,
        ).start()
    def send_r_status_command(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not connected", "Please connect to a COM port first.")
            return
        array_cmd = [Read_Addr_status_fault_mode, 
                     Read_Addr_status_error_mode, 
                     Read_Addr_status_warning_mode, 
                     Read_Addr_status_working_mode]
        array_handle = [self._handle_fault_code_read_response, 
                        self._handle_error_code_read_response, 
                        self._handle_warning_code_read_response, 
                        self._handle_workingmode_read_response]

        for _cmd, _handle in zip(array_cmd, array_handle):
            request = bytearray.fromhex(_cmd)
            self.fill_bytes0_device(request)
            frame = bytes(request) + build_modbus_crc(bytes(request))

            debug_print_tx(frame)
            threading.Thread(
                target=self._send_frame_worker,
                args=(frame, "R_status sent", _handle),
                daemon=True,
            ).start()
            time.sleep(0.2)
    def send_r_voltage_out_command(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not connected", "Please connect to a COM port first.")
            return
        request = bytearray.fromhex(Read_Addr_Output_Voltage)
        self.fill_bytes0_device(request)
        frame = bytes(request) + build_modbus_crc(bytes(request))
        debug_print_tx(frame)
        threading.Thread(
            target=self._send_frame_worker,
            args=(frame, "R_voltage_out sent", self._handle_voltage_out_read_response),
            daemon=True,
        ).start()
    def send_r_voltage_in_command(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not connected", "Please connect to a COM port first.")
            return
        request = bytearray.fromhex(Read_Addr_Input_Voltage)
        self.fill_bytes0_device(request)
        frame = bytes(request) + build_modbus_crc(bytes(request))
        debug_print_tx(frame)
        threading.Thread(
            target=self._send_frame_worker,
            args=(frame, "R_voltage_in sent", self._handle_voltage_in_read_response),
            daemon=True,
        ).start()
    def send_r_current_in_command(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not connected", "Please connect to a COM port first.")
            return
        request = bytearray.fromhex(Read_Addr_Input_Current)
        self.fill_bytes0_device(request)
        frame = bytes(request) + build_modbus_crc(bytes(request))
        debug_print_tx(frame)
        threading.Thread(
            target=self._send_frame_worker,
            args=(frame, "R_current_in sent", self._handle_current_in_read_response),
            daemon=True,
        ).start()
    def send_r_voltage_over_command(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not connected", "Please connect to a COM port first.")
            return
        request = bytearray.fromhex(Read_Addr_Output_Volt_Over_Setting)
        self.fill_bytes0_device(request)
        frame = bytes(request) + build_modbus_crc(bytes(request))
        debug_print_tx(frame)
        threading.Thread(
            target=self._send_frame_worker,
            args=(frame, "R_volt_out sent", self._handle_voltage_over_read_response),
            daemon=True,
        ).start()
    def send_r_current_over_command(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not connected", "Please connect to a COM port first.")
            return
        request = bytearray.fromhex(Read_Addr_Output_Curr_Over_Setting)
        self.fill_bytes0_device(request)
        frame = bytes(request) + build_modbus_crc(bytes(request))
        debug_print_tx(frame)
        threading.Thread(
            target=self._send_frame_worker,
            args=(frame, "R_volt_out sent", self._handle_current_over_read_response),
            daemon=True,
        ).start()
    def send_r_temperature_over_command(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not connected", "Please connect to a COM port first.")
            return
        request = bytearray.fromhex(Read_Addr_Temperature_Over_Setting)
        self.fill_bytes0_device(request)
        frame = bytes(request) + build_modbus_crc(bytes(request))
        debug_print_tx(frame)
        threading.Thread(
            target=self._send_frame_worker,
            args=(frame, "R_volt_out sent", self._handle_temperature_over_read_response),
            daemon=True,
        ).start()
    def _send_frame_worker(
        self,
        frame: bytes,
        status_text: str,
        response_handler: Callable[[bytes], None],
    ) -> None:
        if not self.serial_port:
            return

        try:
            self.serial_port.reset_input_buffer()
            self.serial_port.write(frame)
            self.serial_port.flush()

            response = self._read_response()
            response_handler(response)
            self.root.after(0, lambda: self.status_var.set(status_text))
        except serial.SerialException as exc:
            self.root.after(0, lambda: messagebox.showerror("Serial error", str(exc)))
            self.root.after(0, lambda: self.status_var.set("Serial error"))        
        
    def handle_parse_version_read_response(self, response: bytes) -> None:
        response_text = format_hex(response) if response else "(no response)"
        debug_print_rx(response)
        version_byte0, version_byte1, version_byte2, version_byte3 = parse_version_read_response(response)
        version_concat = version_byte3 + "." + version_byte2 + "." + version_byte1 + "." + version_byte0
        self.root.after(0, lambda: self.response_fw_version_read_all_var.set(version_concat))
    
    def _handle_parse_current_write_response(self, response: bytes) -> None:
        response_text = format_hex(response) if response else "(no response)"
        debug_print_rx(response)
    def _handle_parse_voltage_write_response(self, response: bytes) -> None:
        response_text = format_hex(response) if response else "(no response)"
        debug_print_rx(response)
    def _handle_parse_pwm_duty_write_response(self, response: bytes) -> None:
        response_text = format_hex(response) if response else "(no response)"
        debug_print_rx(response)
    def _handle_current_read_response(self, response: bytes) -> None:
        response_text = format_hex(response) if response else "(no response)"
        debug_print_rx(response)
        TTPLPFC_ac_cur_ref_pu, TTPLPFC_ac_cur_ref_inst_pu, current_cmd = parse_current_read_response(response)
        self.root.after(0, lambda: self.response_current_r_float1_var.set(TTPLPFC_ac_cur_ref_pu))
        self.root.after(0, lambda: self.response_current_r_float2_var.set(TTPLPFC_ac_cur_ref_inst_pu))
        self.root.after(0, lambda: self.response_current_r_cmd_var.set(current_cmd))
    def _handle_cla_heartbeat_read_response(self, response: bytes) -> None:
        response_text = format_hex(response) if response else "(no response)"
        debug_print_rx(response)
        u32_cla_heartbeat = parse_u32_read_response(response)
        self.root.after(0, lambda: self.response_cla_heartbeat_r_u32_var.set(u32_cla_heartbeat))
    def _handle_voltage_out_read_response(self, response: bytes) -> None:
        response_text = format_hex(response) if response else "(no response)"
        debug_print_rx(response)
        float1, float2, u16_1 = parse_current_read_response(response)
        self.root.after(0, lambda: self.response_output_voltage_r_float1_var.set(float1))
        self.root.after(0, lambda: self.response_output_voltage_r_float2_var.set(float2))
        self.root.after(0, lambda: self.response_voltage_r_cmd_var.set(u16_1))
    def _handle_voltage_in_read_response(self, response: bytes) -> None:
        response_text = format_hex(response) if response else "(no response)"
        debug_print_rx(response)
        u16 = parse_u16_read_response(response)
        self.root.after(0, lambda: self.response_voltage_in_r_var.set(u16))        
    def _handle_current_in_read_response(self, response: bytes) -> None:
        response_text = format_hex(response) if response else "(no response)"
        debug_print_rx(response)
        u16 = parse_u16_read_response(response)
        self.root.after(0, lambda: self.response_current_in_r_var.set(u16))

    def _handle_voltage_over_read_response(self, response: bytes) -> None:
        response_text = format_hex(response) if response else "(no response)"
        debug_print_rx(response)
        u16 = parse_u16_index_read_response(response, 8)
        self.root.after(0, lambda: self.response_voltage_over_r_var.set(u16))
    def _handle_current_over_read_response(self, response: bytes) -> None:
        response_text = format_hex(response) if response else "(no response)"
        debug_print_rx(response)
        u16 = parse_u16_index_read_response(response, 8)
        self.root.after(0, lambda: self.response_current_over_r_var.set(u16))
    def _handle_temperature_over_read_response(self, response: bytes) -> None:
        response_text = format_hex(response) if response else "(no response)"
        debug_print_rx(response)
        u16 = parse_u16_index_read_response(response, 8)
        self.root.after(0, lambda: self.response_temperature_over_r_var.set(u16))


    def _handle_pwm_duty_read_response(self, response: bytes) -> None:
        response_text = format_hex(response) if response else "(no response)"
        debug_print_rx(response)
        PWM_FAH_duty, PWM_FAL_duty,  PWM_FBH_duty, PWM_FBL_duty= parse_pwm_duty_read_response(response)
        self.root.after(0, lambda: self.response_pwm_FAH_duty_r_var.set(PWM_FAH_duty))
        self.root.after(0, lambda: self.response_pwm_FAL_duty_r_var.set(PWM_FAL_duty))
        self.root.after(0, lambda: self.response_pwm_FBH_duty_r_var.set(PWM_FBH_duty))
        self.root.after(0, lambda: self.response_pwm_FBL_duty_r_var.set(PWM_FBL_duty))

    def _handle_adc1_read_response(self, response: bytes) -> None:
        response_text = format_hex(response) if response else "(no response)"
        debug_print_rx(response)
        adc1_v165, adc1_vbus, adc1_il2, adc1_il1, adc1_vac = parse_adc1_read_response(response)
        self.root.after(0, lambda: self.response_adc1_v165_r_var.set(adc1_v165))
        self.root.after(0, lambda: self.response_adc1_vbus_r_var.set(adc1_vbus))
        self.root.after(0, lambda: self.response_adc1_il1_r_var.set(adc1_il1))
        self.root.after(0, lambda: self.response_adc1_il2_r_var.set(adc1_il2))
        self.root.after(0, lambda: self.response_adc1_vac_r_var.set(adc1_vac))
    def _handle_adc2_read_response(self, response: bytes) -> None:
        response_text = format_hex(response) if response else "(no response)"
        debug_print_rx(response)
        PFC_F_TEMP, LLC_TEMP, Inlet_TEMP, PFC_S_TEMP = parse_adc2_read_response(response)
        self.root.after(0, lambda: self.response_adc2_PFC_S_TEMP_r_var.set(PFC_F_TEMP))
        self.root.after(0, lambda: self.response_adc2_LLC_TEMP_r_var.set(LLC_TEMP))
        self.root.after(0, lambda: self.response_adc2_Inlet_TEMP_r_var.set(Inlet_TEMP))
        self.root.after(0, lambda: self.response_adc2_PFC_F_TEMP_r_var.set(PFC_S_TEMP))

    def _handle_gpio_read_response(self, response: bytes) -> None:
        response_text = format_hex(response) if response else "(no response)"
        Fan1_RPM, DI_LLC_PwrGood, DO_RELAY, DO_AC_LOSS, DO_NotifyLLC = parse_gpio_read_response(response)
        debug_print_rx(response)
        self.root.after(0, lambda: self.response_gpio_Fan1_RPM_r_var.set(Fan1_RPM))
        self.root.after(0, lambda: self.response_gpio_DI_LLC_r_var.set(DI_LLC_PwrGood))
        self.root.after(0, lambda: self.response_gpio_DO_RELAY_r_var.set(DO_RELAY))
        self.root.after(0, lambda: self.response_gpio_DO_AC_LOSS_r_var.set(DO_AC_LOSS))
        self.root.after(0, lambda: self.response_gpio_DO_NotifyLLC_r_var.set(DO_NotifyLLC))
    def _handle_system_status_read_response(self, response: bytes) -> None:
        response_text = format_hex(response) if response else "(no response)"
        system_status_relay, system_status, system_status_HwControlStatus, system_status_aux, system_status_NormalTripSource = parse_system_status_read_response(response)
        debug_print_rx(response)

        int_system_status_relay = int(system_status_relay)
        enum_relay_mode = _Relay_Mode(int_system_status_relay)
        self.root.after(0, lambda: self.response_system_status_relay_r_var.set(f"{enum_relay_mode.name}({int_system_status_relay})"))

        self.root.after(0, lambda: self.response_system_system_status_r_var.set(system_status))
        self.root.after(0, lambda: self.response_system_status_HwControlStatus_r_var.set(system_status_HwControlStatus))
        self.root.after(0, lambda: self.response_system_status_aux_r_var.set(system_status_aux))
        self.root.after(0, lambda: self.response_system_status_NormalTripSource_r_var.set(system_status_NormalTripSource))
    def _handle_gpio_write_response(self, response: bytes) -> None:
        response_text = format_hex(response) if response else "(no response)"
        debug_print_rx(response)

    def _handle_leg_read_response(self, response: bytes) -> None:
        response_text = format_hex(response) if response else "(no response)"
        HFLegA_EN, HFLegB_EN, OPL_LFLeg_EN, OPL_HFLeg_SR_EN = parse_leg_read_response(response)
        debug_print_rx(response)
        self.root.after(0, lambda: self.response_leg_HFLegA_EN_r_var.set(HFLegA_EN))
        self.root.after(0, lambda: self.response_leg_HFLegB_EN_r_var.set(HFLegB_EN))
        self.root.after(0, lambda: self.response_leg_OPL_LFLeg_EN_r_var.set(OPL_LFLeg_EN))
        self.root.after(0, lambda: self.response_leg_OPL_HFLeg_SR_EN_r_var.set(OPL_HFLeg_SR_EN))
    def _handle_leg_write_response(self, response: bytes) -> None:
        response_text = format_hex(response) if response else "(no response)"
        debug_print_rx(response)
    def _handle_protect_reset_write_response(self, response: bytes) -> None:
        response_text = format_hex(response) if response else "(no response)"
        debug_print_rx(response)
    def _handle_working_mode_write_response(self, response: bytes) -> None:
        response_text = format_hex(response) if response else "(no response)"
        debug_print_rx(response)
    def _handle_fault_code_read_response(self, response: bytes) -> None:
        response_text = format_hex(response) if response else "(no response)"
        Fault_Code = parse_u32_read_response(response)
        debug_print_rx(response)
        parse_Fault_Code = decode_faults(Fault_Code)
        self.root.after(0, lambda: self.response_Fault_Code_r_var.set(f"{parse_Fault_Code}({Fault_Code})"))

    def _handle_error_code_read_response(self, response: bytes) -> None:
        response_text = format_hex(response) if response else "(no response)"
        Error_Code = parse_u32_read_response(response)
        debug_print_rx(response)
        parse_Error_Code = decode_errors(Error_Code)
        self.root.after(0, lambda: self.response_Error_Code_r_var.set(f"{parse_Error_Code}({Error_Code})"))

    def _handle_warning_code_read_response(self, response: bytes) -> None:
        response_text = format_hex(response) if response else "(no response)"
        Warning_Code = parse_u32_read_response(response)
        debug_print_rx(response)
        parse_Warning_Code = decode_warnings(Warning_Code)
        self.root.after(0, lambda: self.response_Warning_Code_r_var.set(f"{parse_Warning_Code}({Warning_Code})"))

    def _handle_workingmode_read_response(self, response: bytes) -> None:
        response_text = format_hex(response) if response else "(no response)"
        Working_Mode = parse_working_mode_read_response(response)
        debug_print_rx(response)
        enum_working_mode = _Working_Mode(Working_Mode)
        self.root.after(0, lambda: self.response_Working_Mode_r_var.set(f"{enum_working_mode.name}({Working_Mode})"))

    def _read_response(self) -> bytes:
        if not self.serial_port:
            return b""

        chunks: List[bytes] = []
        deadline = time.monotonic() + 2.0
        idle_since: Optional[float] = None

        while time.monotonic() < deadline:
            waiting = self.serial_port.in_waiting
            if waiting:
                chunks.append(self.serial_port.read(waiting))
                idle_since = time.monotonic()
            else:
                if idle_since is not None and time.monotonic() - idle_since >= 0.2:
                    break
                time.sleep(0.02)

        return b"".join(chunks)


def main() -> None:
    root = tk.Tk()
    app = ModbusGuiApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
