# Modbus_PFC_gui_tool
for PFC debug gui thorugh modbus command

## Run

```bash
pip install -r requirements.txt
python main.py
```

## Features

- COM port dropdown
- Connect / Disconnect
- Send Modbus RTU request `03 03 00 01 00 04` with CRC auto-calculated
- Show the received response next to button `A`
- Enter `Current` as a `uint8` and send `03 06 04 48 00 02 00 01` with byte 7 replaced by the input value
- Read current with `R_current` using `03 03 00 61 00 06`, then decode `byte[6:9]` as `float32` and `byte[11]` as `uint8`
- Write GPIO with `W_GPIO` using `04 06 03 FC 00 01 00 00`, where `byte[7]` is built from `DO_RELAY` bit0, `DO_AC_LOSS` bit1, and `DO_NotifyLLC` bit2
