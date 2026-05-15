import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
import struct
from typing import Callable, List, Optional

import serial
from serial.tools import list_ports

Read_Addr_FW_version = "03 03 00 01 00 04"
Read_Addr_Current = "03 03 00 61 00 06"


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


def parse_current_read_response(data: bytes) -> tuple[str, str]:
    if len(data) < 12:
        return "N/A", "N/A"

    float_bytes = data[6:10]
    current_ref_value = struct.unpack(">f", float_bytes)[0]
    current_ref = f"{current_ref_value:.6f}".rstrip("0").rstrip(".")
    current_cmd = str(data[11])
    return current_ref, current_cmd


class ModbusGuiApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Modbus PFC GUI Tool")
        self.root.geometry("720x260")

        self.serial_port: Optional[serial.Serial] = None
        self.port_var = tk.StringVar()
        self.status_var = tk.StringVar(value="Disconnected")
        self.response_var = tk.StringVar(value="")
        self.current_var = tk.StringVar(value="0")
        self.current_read_raw_var = tk.StringVar(value="")
        self.current_read_float_var = tk.StringVar(value="N/A")
        self.current_read_cmd_var = tk.StringVar(value="N/A")

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

        ttk.Label(top, textvariable=self.status_var, foreground="#005f8d").grid(
            row=1, column=0, columnspan=5, sticky="w", pady=(10, 0)
        )

        action = ttk.LabelFrame(root, text="Action", padding=12)
        action.pack(fill="x", pady=(12, 0))

        ttk.Label(action, text="Response").grid(row=0, column=0, sticky="w")
        self.response_entry = ttk.Entry(action, textvariable=self.response_var, width=62, state="readonly")
        self.response_entry.grid(row=0, column=1, padx=(8, 12), sticky="we")

        ttk.Button(action, text="A", command=self.send_a_command, width=8).grid(row=0, column=2, sticky="e")

        action.columnconfigure(1, weight=1)

        current = ttk.LabelFrame(root, text="Current Setting", padding=12)
        current.pack(fill="x", pady=(12, 0))

        ttk.Label(current, text="Current (uint8)").grid(row=0, column=0, sticky="w")
        self.current_spin = ttk.Spinbox(current, from_=0, to=255, textvariable=self.current_var, width=10)
        self.current_spin.grid(row=0, column=1, padx=(8, 12), sticky="w")

        ttk.Label(current, text="Response").grid(row=0, column=2, sticky="w")
        self.current_response_entry = ttk.Entry(current, textvariable=self.response_var, width=40, state="readonly")
        self.current_response_entry.grid(row=0, column=3, padx=(8, 12), sticky="we")

        ttk.Button(current, text="W_current", command=self.send_w_current_command, width=12).grid(
            row=0, column=4, sticky="e"
        )

        current.columnconfigure(3, weight=1)

        current_read = ttk.LabelFrame(root, text="Current Read", padding=12)
        current_read.pack(fill="x", pady=(12, 0))

        ttk.Button(current_read, text="R_current", command=self.send_r_current_command, width=12).grid(
            row=0, column=0, sticky="w"
        )

        ttk.Label(current_read, text="Response").grid(row=0, column=1, sticky="w", padx=(12, 0))
        self.current_read_raw_entry = ttk.Entry(
            current_read, textvariable=self.current_read_raw_var, width=42, state="readonly"
        )
        self.current_read_raw_entry.grid(row=0, column=2, padx=(8, 12), sticky="we")

        ttk.Label(current_read, text="TTPLPFC_ac_cur_ref_inst_pu").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(current_read, textvariable=self.current_read_float_var, width=18, state="readonly").grid(
            row=1, column=1, padx=(12, 8), pady=(8, 0), sticky="w"
        )

        ttk.Label(current_read, text="current_cmd_from_modbus").grid(row=1, column=2, sticky="w", pady=(8, 0))
        ttk.Entry(current_read, textvariable=self.current_read_cmd_var, width=18, state="readonly").grid(
            row=1, column=3, padx=(8, 0), pady=(8, 0), sticky="w"
        )

        current_read.columnconfigure(2, weight=1)

        hint = ttk.Label(
            root,
            text="所有發送 以及 回應的訊息",
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

    def send_a_command(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not connected", "Please connect to a COM port first.")
            return

        request = bytes.fromhex(Read_Addr_FW_version)
        frame = request + build_modbus_crc(request)
        threading.Thread(
            target=self._send_frame_worker,
            args=(frame, "Command sent", self._handle_default_response),
            daemon=True,
        ).start()

    def send_w_current_command(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not connected", "Please connect to a COM port first.")
            return

        try:
            current_value = int(self.current_var.get().strip())
        except ValueError:
            messagebox.showwarning("Invalid current", "Current must be an integer between 0 and 255.")
            return

        if not 0 <= current_value <= 255:
            messagebox.showwarning("Invalid current", "Current must be an integer between 0 and 255.")
            return

        request = bytearray.fromhex("03 06 04 48 00 02 00 01")
        # Fill the 8th byte (index 7) before appending CRC, per SetCurrentCmd[7].
        request[7] = current_value
        frame = bytes(request) + build_modbus_crc(bytes(request))
        threading.Thread(
            target=self._send_frame_worker,
            args=(frame, "W_current sent", self._handle_default_response),
            daemon=True,
        ).start()

    def send_r_current_command(self) -> None:
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Not connected", "Please connect to a COM port first.")
            return

        request = bytes.fromhex(Read_Addr_Current)
        frame = request + build_modbus_crc(request)
        threading.Thread(
            target=self._send_frame_worker,
            args=(frame, "R_current sent", self._handle_current_read_response),
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

    def _handle_default_response(self, response: bytes) -> None:
        response_text = format_hex(response) if response else "(no response)"
        self.root.after(0, lambda: self.response_var.set(response_text))

    def _handle_current_read_response(self, response: bytes) -> None:
        response_text = format_hex(response) if response else "(no response)"
        float_value, current_cmd = parse_current_read_response(response)
        self.root.after(0, lambda: self.current_read_raw_var.set(response_text))
        self.root.after(0, lambda: self.current_read_float_var.set(float_value))
        self.root.after(0, lambda: self.current_read_cmd_var.set(current_cmd))

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
