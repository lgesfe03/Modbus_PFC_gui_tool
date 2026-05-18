import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
import struct
from typing import Callable, List, Optional

import serial
from serial.tools import list_ports

# destinate_device_options = ['single PFC 03', 'ControlBoard 04', 'PSU 00']
destinate_device_options = [0x03, 0x04, 0x00]
Read_Addr_FW_version = "03 03 00 01 00 04"
Read_Addr_Current = "03 03 00 61 00 06"
Read_Addr_PWM_duty = "03 03 00 25 00 01"
Read_Addr_ADC1 = "03 03 00 0C 00 0A"
Read_Addr_GPIO = "03 03 00 15 00 01"
Write_Addr_Current = "03 06 04 48 00 02 00 01"
Write_Addr_GPIO = "03 06 03 FC 00 01 00"

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
    if len(data) < 10:
        return "N/A", "N/A"
    version_byte0 = str(data[9])
    version_byte1 = str(data[8])
    version_byte2 = str(data[7])
    version_byte3 = str(data[6])
    return version_byte0, version_byte1, version_byte2, version_byte3
def parse_current_read_response(data: bytes) -> tuple[str, str]:
    if len(data) < 12:
        return "N/A", "N/A"

    float_bytes = data[6:10]
    current_ref_value = struct.unpack(">f", float_bytes)[0]
    current_ref = f"{current_ref_value:.6f}".rstrip("0").rstrip(".")
    current_cmd = str(data[11])
    return current_ref, current_cmd
def parse_pwm_duty_read_response(data: bytes) -> tuple[str, str]:
    if len(data) < 6+2:
        return "N/A", "N/A"

    pwm_duty_FAH = str(data[7])
    return pwm_duty_FAH
    pwm_duty_FAL = str(data[8])
    pwm_duty_FBH = str(data[9])
    pwm_duty_FBL = str(data[10])
    return pwm_duty_FAH, pwm_duty_FAL, pwm_duty_FBH, pwm_duty_FBL
def convert_vac_voltage(u16_adc):
    return str(u16_adc*0.2585 - 530.2)
def convert_il_amp(u16_adc):
    return str(u16_adc*0.0222- 45.537)
def convert_vbus_voltage(u16_adc):
    return str(u16_adc*0.12924)
def convert_1v65_voltage(u16_adc):
    return str(u16_adc/4095*3.3)
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
def parse_gpio_read_response(data: bytes) -> tuple[str, str]:
    if len(data) < 6+1:
        return "N/A", "N/A"
    
    bit_Fan1_RPM = (data[6] >> 0) & 0x01
    bit_DI_LLC_PwrGood = (data[6] >> 1) & 0x01
    bit_DO_RELAY = (data[6] >> 2) & 0x01
    bit_DO_AC_LOSS = (data[6] >> 3) & 0x01
    bit_DO_NotifyLLC = (data[6] >> 4) & 0x01
    return str(bit_Fan1_RPM), str(bit_DI_LLC_PwrGood), str(bit_DO_RELAY), str(bit_DO_AC_LOSS), str(bit_DO_NotifyLLC)
class ModbusGuiApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Modbus PFC GUI Tool")
        self.root.geometry("1024x888")

        self.serial_port: Optional[serial.Serial] = None
        self.port_var = tk.StringVar()
        self.device_byte0_var = tk.StringVar()
        self.status_var = tk.StringVar(value="Disconnected")
        self.response_fw_version_read_all_var = tk.StringVar(value="")
        self.input_from_gui_current_w_var = tk.StringVar(value="0")
        self.response_current_r_float_var = tk.StringVar(value="N/A")
        self.response_current_r_cmd_var = tk.StringVar(value="N/A")
        self.response_pwm_FAH_duty_r_var = tk.StringVar(value="")
        self.response_pwm_FAL_duty_r_var = tk.StringVar(value="")
        self.response_pwm_FBH_duty_r_var = tk.StringVar(value="")
        self.response_pwm_FBL_duty_r_var = tk.StringVar(value="")
        self.response_adc1_v165_r_var = tk.StringVar(value="")
        self.response_adc1_vbus_r_var = tk.StringVar(value="")
        self.response_adc1_il1_r_var = tk.StringVar(value="")
        self.response_adc1_il2_r_var = tk.StringVar(value="")
        self.response_adc1_vac_r_var = tk.StringVar(value="")
        self.response_gpio_Fan1_RPM_r_var = tk.StringVar(value="")
        self.response_gpio_DI_LLC_r_var = tk.StringVar(value="")
        self.response_gpio_DO_RELAY_r_var = tk.StringVar(value="")
        self.response_gpio_DO_AC_LOSS_r_var = tk.StringVar(value="")
        self.response_gpio_DO_NotifyLLC_r_var = tk.StringVar(value="")
        self.gpio_do_relay_w_var = tk.IntVar(value=0)
        self.gpio_do_ac_loss_w_var = tk.IntVar(value=0)
        self.gpio_do_notifyllc_w_var = tk.IntVar(value=0)


        self._build_ui()
        self.refresh_ports()

    def _build_ui(self) -> None:
        root = ttk.Frame(self.root, padding=12)
        root.pack(fill="both", expand=True)

        top = ttk.LabelFrame(root, text="Connection", padding=12)
        top.pack(fill="x")

        ttk.Label(top, text="COM Port").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, state="readonly", width=22)
        self.port_combo.grid(row=0, column=1, padx=(8, 12), sticky="w")

        ttk.Button(top, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=(0, 8))
        ttk.Button(top, text="Connect", command=self.connect_port).grid(row=0, column=3, padx=(0, 8))
        ttk.Button(top, text="Disconnect", command=self.disconnect_port).grid(row=0, column=4)

        ttk.Label(top, text="DeviceBytes0").grid(row=0, column=5, sticky="w")
        self.device_combo = ttk.Combobox(top, textvariable=self.device_byte0_var, state="readonly", width=22, values=destinate_device_options)
        self.device_combo.current(0)
        self.device_combo.grid(row=0, column=6, padx=(8, 12), sticky="w")

        ttk.Label(top, textvariable=self.status_var, foreground="#005f8d").grid(
            row=1, column=0, columnspan=5, sticky="w", pady=(10, 0)
        )
    # get version
        f_get_version = ttk.LabelFrame(root, text="FW_version", padding=12)
        f_get_version.pack(fill="x", pady=(12, 0))
        ttk.Button(f_get_version, text="R_Version", command=self.send_r_version_command, width=12).grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(f_get_version, text="Version.").grid(row=0, column=1, sticky="w", pady=(8, 0))
        ttk.Entry(f_get_version, textvariable=self.response_fw_version_read_all_var, width=18, state="readonly").grid(
            row=0, column=2, padx=(12, 8), pady=(8, 0), sticky="w"
        )
        f_get_version.columnconfigure(10, weight=1)
    # set current
        f_current = ttk.LabelFrame(root, text="Current Related", padding=12)
        f_current.pack(fill="x", pady=(12, 0))
        ttk.Button(f_current, text="W_current", command=self.send_w_current_command, width=12).grid(
            row=1, column=0, sticky="w"
        )
        self.current_spin = ttk.Spinbox(f_current, from_=0, to=255, textvariable=self.input_from_gui_current_w_var, width=10)
        self.current_spin.grid(
            row=1, column=1, padx=(8, 12), sticky="w")
    # get current
        ttk.Button(f_current, text="R_current", command=self.send_r_current_command, width=12).grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(f_current, text="TTPLPFC_ac_cur_ref_inst_pu").grid(
            row=0, column=1, sticky="w", pady=(8, 0))
        ttk.Entry(f_current, textvariable=self.response_current_r_float_var, width=18, state="readonly").grid(
            row=0, column=2, padx=(12, 8), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_current, text="current_cmd_from_modbus").grid(
            row=0, column=3, sticky="w", pady=(8, 0))
        ttk.Entry(f_current, textvariable=self.response_current_r_cmd_var, width=18, state="readonly").grid(
            row=0, column=4, padx=(8, 0), pady=(8, 0), sticky="w"
        )
        f_current.columnconfigure(10, weight=1)
    # get PWM duty
        f_r_pwm_duty = ttk.LabelFrame(root, text="PWM Read", padding=12)
        f_r_pwm_duty.pack(fill="x", pady=(12, 0))
        ttk.Button(f_r_pwm_duty, text="R_PWM duty", command=self.send_r_pwm_duty_command, width=12).grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(f_r_pwm_duty, text="PWM-FAH").grid(
            row=0, column=1, sticky="w", pady=(8, 0))
        ttk.Entry(f_r_pwm_duty, textvariable=self.response_pwm_FAH_duty_r_var, width=18, state="readonly").grid(
            row=0, column=2, padx=(8, 0), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_r_pwm_duty, text="PWM-FAL").grid(
            row=0, column=3, sticky="w", pady=(8, 0))
        ttk.Entry(f_r_pwm_duty, textvariable=self.response_pwm_FAL_duty_r_var, width=18, state="readonly").grid(
            row=0, column=4, padx=(8, 0), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_r_pwm_duty, text="PWM-FBH").grid(
            row=0, column=5, sticky="w", pady=(8, 0))
        ttk.Entry(f_r_pwm_duty, textvariable=self.response_pwm_FBH_duty_r_var, width=18, state="readonly").grid(
            row=0, column=6, padx=(8, 0), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_r_pwm_duty, text="PWM-FBL").grid(
            row=0, column=7, sticky="w", pady=(8, 0))
        ttk.Entry(f_r_pwm_duty, textvariable=self.response_pwm_FBL_duty_r_var, width=18, state="readonly").grid(
            row=0, column=8, padx=(8, 0), pady=(8, 0), sticky="w"
        )
        f_r_pwm_duty.columnconfigure(10, weight=1)
    # get ADC1s
        f_adc_r = ttk.LabelFrame(root, text="ADC1 Read", padding=12)
        f_adc_r.pack(fill="x", pady=(12, 0))
        ttk.Button(f_adc_r, text="R_ADC1", command=self.send_r_adc1_command, width=12).grid(
            row=0, column=0, sticky="w"
        )

        ttk.Label(f_adc_r, text="Vac").grid(
            row=0, column=1, sticky="w", pady=(8, 0))
        ttk.Entry(f_adc_r, textvariable=self.response_adc1_vac_r_var, width=18, state="readonly").grid(
            row=0, column=2, padx=(8, 0), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_adc_r, text="iL1").grid(
            row=0, column=3, sticky="w", pady=(8, 0))
        ttk.Entry(f_adc_r, textvariable=self.response_adc1_il1_r_var, width=18, state="readonly").grid(
            row=0, column=4, padx=(8, 0), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_adc_r, text="iL2").grid(
            row=0, column=5, sticky="w", pady=(8, 0))
        ttk.Entry(f_adc_r, textvariable=self.response_adc1_il2_r_var, width=18, state="readonly").grid(
            row=0, column=6, padx=(8, 0), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_adc_r, text="Vbus").grid(
            row=0, column=7, sticky="w", pady=(8, 0))
        ttk.Entry(f_adc_r, textvariable=self.response_adc1_vbus_r_var, width=18, state="readonly").grid(
            row=0, column=8, padx=(8, 0), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_adc_r, text="1.65V").grid(
            row=0, column=9, sticky="w", pady=(8, 0))
        ttk.Entry(f_adc_r, textvariable=self.response_adc1_v165_r_var, width=18, state="readonly").grid(
            row=0, column=10, padx=(12, 8), pady=(8, 0), sticky="w"
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
        ttk.Entry(f_gpio, textvariable=self.response_gpio_DO_NotifyLLC_r_var, width=18, state="readonly").grid(
            row=0, column=2, padx=(12, 8), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_gpio, text="DO_AC_LOSS").grid(
            row=0, column=3, sticky="w", pady=(8, 0))
        ttk.Entry(f_gpio, textvariable=self.response_gpio_DO_AC_LOSS_r_var, width=18, state="readonly").grid(
            row=0, column=4, padx=(8, 0), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_gpio, text="DO_RELAY").grid(
            row=0, column=5, sticky="w", pady=(8, 0))
        ttk.Entry(f_gpio, textvariable=self.response_gpio_DO_RELAY_r_var, width=18, state="readonly").grid(
            row=0, column=6, padx=(8, 0), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_gpio, text="DI_LLC").grid(
            row=0, column=7, sticky="w", pady=(8, 0))
        ttk.Entry(f_gpio, textvariable=self.response_gpio_DI_LLC_r_var, width=18, state="readonly").grid(
            row=0, column=8, padx=(8, 0), pady=(8, 0), sticky="w"
        )
        ttk.Label(f_gpio, text="Fan1_RPM").grid(
            row=0, column=9, sticky="w", pady=(8, 0))
        ttk.Entry(f_gpio, textvariable=self.response_gpio_Fan1_RPM_r_var, width=18, state="readonly").grid(
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

    # history
        hint = ttk.Label(
            root,
            text="Historical messages",
            foreground="#020101",
        )
        hint.pack(anchor="w", pady=(12, 0))

    def refresh_ports(self) -> None:
        ports = [port.device for port in list_ports.comports()]
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
    def fill_bytes0_device(self, request: bytearray) -> None:
        request[0] = int(self.device_combo.get())
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
            current_value = int(self.input_from_gui_current_w_var.get().strip())
        except ValueError:
            messagebox.showwarning("Invalid current", "Current must be an integer between 0 and 255.")
            return

        if not 0 <= current_value <= 255:
            messagebox.showwarning("Invalid current", "Current must be an integer between 0 and 255.")
            return

        request = bytearray.fromhex(Write_Addr_Current)
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

    def send_r_current_command(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not connected", "Please connect to a COM port first.")
            return

        request = bytearray.fromhex(Read_Addr_Current)
        self.fill_bytes0_device(request)
        frame = bytes(request) + build_modbus_crc(bytes(request))
        debug_print_tx(frame)
        threading.Thread(
            target=self._send_frame_worker,
            args=(frame, "R_current sent", self._handle_current_read_response),
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

    def _handle_current_read_response(self, response: bytes) -> None:
        response_text = format_hex(response) if response else "(no response)"
        debug_print_rx(response)
        float_value, current_cmd = parse_current_read_response(response)
        self.root.after(0, lambda: self.response_current_r_float_var.set(float_value))
        self.root.after(0, lambda: self.response_current_r_cmd_var.set(current_cmd))

    def _handle_pwm_duty_read_response(self, response: bytes) -> None:
        response_text = format_hex(response) if response else "(no response)"
        debug_print_rx(response)
        PWM_FAH_duty = parse_pwm_duty_read_response(response)
        self.root.after(0, lambda: self.response_pwm_FAH_duty_r_var.set(PWM_FAH_duty))
        # self.root.after(0, lambda: self.response_pwm_FAL_duty_r_var.set(PWM_FAL_duty))
        # self.root.after(0, lambda: self.response_pwm_FBH_duty_r_var.set(PWM_FBH_duty))
        # self.root.after(0, lambda: self.response_pwm_FBL_duty_r_var.set(PWM_FBL_duty))

    def _handle_adc1_read_response(self, response: bytes) -> None:
        response_text = format_hex(response) if response else "(no response)"
        debug_print_rx(response)
        adc1_v165, adc1_vbus, adc1_il2, adc1_il1, adc1_vac = parse_adc1_read_response(response)
        self.root.after(0, lambda: self.response_adc1_v165_r_var.set(adc1_v165))
        self.root.after(0, lambda: self.response_adc1_vbus_r_var.set(adc1_vbus))
        self.root.after(0, lambda: self.response_adc1_il1_r_var.set(adc1_il1))
        self.root.after(0, lambda: self.response_adc1_il2_r_var.set(adc1_il2))
        self.root.after(0, lambda: self.response_adc1_vac_r_var.set(adc1_vac))

    def _handle_gpio_read_response(self, response: bytes) -> None:
        response_text = format_hex(response) if response else "(no response)"
        Fan1_RPM, DI_LLC_PwrGood, DO_RELAY, DO_AC_LOSS, DO_NotifyLLC = parse_gpio_read_response(response)
        debug_print_rx(response)
        self.root.after(0, lambda: self.response_gpio_Fan1_RPM_r_var.set(Fan1_RPM))
        self.root.after(0, lambda: self.response_gpio_DI_LLC_r_var.set(DI_LLC_PwrGood))
        self.root.after(0, lambda: self.response_gpio_DO_RELAY_r_var.set(DO_RELAY))
        self.root.after(0, lambda: self.response_gpio_DO_AC_LOSS_r_var.set(DO_AC_LOSS))
        self.root.after(0, lambda: self.response_gpio_DO_NotifyLLC_r_var.set(DO_NotifyLLC))

    def _handle_gpio_write_response(self, response: bytes) -> None:
        response_text = format_hex(response) if response else "(no response)"
        debug_print_rx(response)

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
