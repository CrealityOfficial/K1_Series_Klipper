# klippy/extras/ms32008.py
# MS32008 Klipper driver
#
# Features:
#  - init_chip() (Init_MS32008 equivalent)
#  - soft_reset(), hard_sleep(), soft_standby(), event_clr(), watch_sel()
#  - set_fsw(), ext_clk(), osc_off()
#  - channel functions for CHA/CHB:
#       set_pd_out(), set_record_rev(), set_dir(), set_stop_pos(),
#       set_ms_mode(), set_pps(), set_step_num(), set_current(),
#       ch_conf_load(), ch_force_stop(), read_status(), read_pulse_record()
#  - DC motor control: dc_ctrl()
#  - G-code commands to call common actions and for debug (MS32008_*).
#
# Usage in printer.cfg:
# [ms32008]
# i2c_bus: i2c1         # default i2c bus name Klipper exposes; adjust if needed
# i2c_address: 0x16     # change if your board uses different address
#
# Example G-codes:
#   MS32008_INIT
#   MS32008_SET PPS=400 CH=A
#   MS32008_MOVE CH=A PPS=400 STEPS=1000 DIR=CW
#   MS32008_READ REG=0x0F COUNT=1
#

from __future__ import annotations
import time
import logging

from . import bus


FCLK_FREQUENCY = 20000000  # Hz (from FCLK define)

# Simple channel masks
CHA = 0x10
CHB = 0x20

Registers = {
    # control / config
    "CONF0":    0x00,  # contains nRST, standby, stm_fsw multipliers, useExtClk, oscOFF
    "CONF1":    0x01,  # ACH_confLoad, BCH_confLoad
    "CONF2":    0x02,  # ACH_forceStop, BCH_forceStop
    "DCMOTOR":  0x03,  # DCMotor control (Hiz, A2B, B2A, Brake)

    # single-bit clear/status
    "CLR":      0x0D,  # uvloClr, otsClr
    "WATCH":    0x0E,  # watch enable and watch modes
    "CHIPFLAG": 0x0F,  # chip flag (reserved in header)

    # channel control and misc
    "CH_CTRL":  0x10,  # CHx power driver enable / record reverse
    "CH_DIR_MS":0x11,  # direction, force-stop-pos, microstep mode
    "ACH_FREQ_L": 0x12, # placeholder names for multi-byte registers
    "ACH_FREQ_H": 0x13,
    "ACH_PULSE_L":0x14,
    "ACH_PULSE_H":0x15,
    "ACH_AMP":   0x16,  # amplitude
    "READ_STATUS": 0x1D, # read status (status bits defined)
    "ACH_PULSE_RECORD_H": 0x1E,
    "ACH_PULSE_RECORD_L": 0x1F
}

# Registers that are commonly read / polled
ReadRegisters = [
    "CONF0", "CONF1", "CONF2", "DCMOTOR",
    "CLR", "WATCH", "CHIPFLAG",
    "CH_CTRL", "CH_DIR_MS", "ACH_FREQ_L", "ACH_FREQ_H",
    "ACH_PULSE_L", "ACH_PULSE_H", "ACH_AMP",
    "READ_STATUS", "ACH_PULSE_RECORD_H", "ACH_PULSE_RECORD_L"
]

# Fields — bit masks for each register (as in tmc2208.py style)
Fields = {}

# CONF0 (00H) bits
Fields["CONF0"] = {
    "nRST_ENABLE":        0x00,   # nRST enable (active default)
    "nRST_DISABLE":       0x01,   # nRST disable
    "standby_DISABLE":    0x00,   # standby off
    "standby_ENABLE":     0x02,   # standby on (awake)
    # stm_fsw multiplier (bits 2..4 used, values as in .h)
    "stm_fsw_MUL1":       0x00,   # 2
    "stm_fsw_MUL2":       0x04,   # 4
    "stm_fsw_MUL3":       0x08,   # 6
    "stm_fsw_MUL4":       0x0C,   # 8
    "stm_fsw_MUL5":       0x10,   # 10
    "stm_fsw_MUL6P5":     0x14,   # 13
    "stm_fsw_MUL8":       0x18,   # 16
    "stm_fsw_MUL10":      0x1C,   # 20
    "useExtClk_DISABLE":  0x00,   # use external clock disabled
    "useExtClk_ENABLE":   0x40,   # use external clock enabled (FCLK input)
    "oscOFF_DISABLE":     0x00,   # oscillator off disabled
    "oscOFF_ENABLE":      0x80,   # oscillator off enabled
}

# CONF1 (01H)
Fields["CONF1"] = {
    "ACH_confLoad":       0x80,   # load config for channel A and execute
    "BCH_confLoad":       0x40,   # load config for channel B and execute
}

# CONF2 (02H)
Fields["CONF2"] = {
    "ACH_forceStop":      0x80,   # A channel force stop (use for immediate stop)
    "BCH_forceStop":      0x40,   # B channel force stop
}

# DCMOTOR (03H)
Fields["DCMOTOR"] = {
    "DCMotor_Hiz":        0x00,   # Hi-Z (idle)
    "DCMotor_A2B":        0x01,   # rotate A->B (CW / forward)
    "DCMotor_B2A":        0x02,   # rotate B->A (CCW / reverse)
    "DCMotor_Brake":      0x03,   # brake
}

# CLR (0DH) — clears
Fields["CLR"] = {
    "uvloClr":            0x80,   # clear undervoltage latch (write to clear)
    "otsClr":             0x40,   # clear over-temp/shutdown latch
}

# WATCH (0EH) — watchdog & status selection
Fields["WATCH"] = {
    "watchEN_ENABLE":     0x80,   # enable watchdog
    "watchEN_DISABLE":    0x00,   # disable watchdog

    # watch selection codes (lower bits)
    "watch_ACH_FG":       0x00,   # ACH FG (1 of 1/4 measurement etc.)
    "watch_ACH_Runing":   0x02,   # ACH running
    "watch_ACH_cacheBusy":0x03,   # ACH cache busy
    "watch_BCH_FG":       0x04,   # BCH FG
    "watch_BCH_Runing":   0x06,   # BCH running
    "watch_BCH_cacheBusy":0x07,   # BCH cache busy
    "watch_OTP":          0x0D,   # OTP (one-time-program) status
    "watch_UVP":          0x0E,   # UVP (undervoltage protection) status
    "watch_SycClkDiv400": 0x0F,   # synchronous clock divided by 400
}

# CHIPFLAG (0FH) — reserved/flags (header mentioned 'chipFlag' but not detailed)
Fields["CHIPFLAG"] = {
    # no specific defines in header — keep placeholder
    # e.g. "chip_flag": 0x01
}

# CH_CTRL (10H)
Fields["CH_CTRL"] = {
    "CHx_PowDri_ENABLE":  0x40,   # power driver enable
    "CHx_PowDri_DISABLE": 0x00,   # power driver disable
    "CHx_recordRev_ENABLE":0x20,  # enable reverse recording (A channel brake/dir logging)
    "CHx_recordRev_DISABLE":0x00, # disable reverse recording
}

# CH_DIR_MS (11H)
Fields["CH_DIR_MS"] = {
    "CHx_Dir_CW":         0x00,   # CW (forward)
    "CHx_Dir_CCW":        0x01,   # CCW (reverse)
    # Force stop positions (bits 2..3)
    "CHx_ForceStopPosDiv4": 0x00, # 1/4 position (default)
    "CHx_ForceStopPosDiv2": 0x04, # 1/2 position
    "CHx_ForceStopPos2Pha":  0x08, # 2 phase position
    "CHx_ForceStopPos1Pha":  0x0C, # 1 phase position
    # microstep mode (bits 4..7)
    "CHx_msMode1Div256":  0x00,
    "CHx_msMode1Div128":  0x10,
    "CHx_msMode1Div64":   0x20,
    "CHx_msMode1Div32":   0x30,
    "CHx_msMode1Div16":   0x40,
    "CHx_msMode1Div8":    0x50,
    "CHx_msMode1Div4":    0x60,
    "CHx_msMode1Div2":    0x70,
    "CHx_msModeFull":     0x80,
}

# ACH_FREQ (12H/13H) — frequency low/high bytes for channel A
# We'll keep per-byte masks (full register is 8 bits each)
Fields["ACH_FREQ_L"] = {
    "freq_lo":            0xFF,
}
Fields["ACH_FREQ_H"] = {
    "freq_hi":            0xFF,
}

# ACH_PULSE (14H/15H) — pulse low/high bytes
Fields["ACH_PULSE_L"] = {
    "pulse_lo":           0xFF,
}
Fields["ACH_PULSE_H"] = {
    "pulse_hi":           0xFF,
}

# ACH_AMP (16H) — amplitude (0..255)
Fields["ACH_AMP"] = {
    "amp":                0xFF,
}

# READ_STATUS (1DH)
Fields["READ_STATUS"] = {
    "BIT_ChipOTS":        0x01,   # over temperature shutdown
    "BIT_ChipUVLO":       0x02,   # undervoltage lockout
    "BIT_CHx_cacheBusy":  0x04,   # channel cache busy
    "BIT_CHx_Running":    0x08,   # channel running
}

# ACH_PULSE_RECORD (1EH/1FH)
Fields["ACH_PULSE_RECORD_H"] = {
    "pulse_record_hi":    0xFF
}
Fields["ACH_PULSE_RECORD_L"] = {
    "pulse_record_lo":    0xFF
}

def flatten_fields():
    """
    Return a flattened mapping: (reg_name, field_name) -> mask
    Useful for code that wants to lookup by tuple.
    """
    flat = {}
    for reg, fdict in Fields.items():
        for fname, mask in fdict.items():
            flat[(reg, fname)] = mask
    return flat


class MS32008Error(Exception):
    pass

def load_config_prefix(config, name, default=None):
    try:
        return config.get(name, default)
    except Exception:
        return default

class MS32008:
    def __init__(self, config):
        self.printer = config.get_printer()
        # config fields
        # i2c_addr = (0x40 | (0x00 << 1)) >> 1
        i2c_addr = 0x10
        self.i2c = self._get_i2c(config, i2c_addr)

        # create G-code commands for convenience / debugging
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command("MS32008_INIT", self.gcmd_init, desc="Init MS32008")
        self.gcode.register_command("MS32008_READ", self.gcmd_read, desc="Read MS32008 registers")
        self.gcode.register_command("MS32008_WRITE", self.gcmd_write, desc="Write MS32008 registers")
        self.gcode.register_command("MS32008_MOVE", self.gcmd_move, desc="High-level move (set PPS,steps,dir,ch_conf_load)")
        self.gcode.register_command("MS32008_SET", self.gcmd_set, desc="Set CH param like CURRENT/PPS/DIR")
        self.gcode.register_command("MS32008_DC", self.gcmd_dc, desc="Control DC motor channel")

    def _get_i2c(self, config, i2c_addr):
        """
        hold on 
        NOTE: i2c fd
        """
        try:
            i2c = bus.MCU_I2C_from_config(config, default_addr=i2c_addr)
            if hasattr(i2c, 'i2c_write') and hasattr(i2c, 'i2c_read'):
                return i2c
        except Exception:
            pass
        raise MS32008Error(f"Could not find I2C bus object for '{i2c_addr}' - adjust i2c_bus in config")

    # ---------------------------
    # Low-level I2C helpers
    # ---------------------------
    def _i2c_write_reg(self, reg, data):
        """Write 'data' (an iterable of bytes) to register 'reg'."""
        if not isinstance(data, (bytes, bytearray, list, tuple)):
            data = [data]
        # We write as: [reg, data0, data1, ...]
        payload = [reg] + [int(x) & 0xFF for x in data]
        if hasattr(self.i2c, 'i2c_write'):
            self.i2c.i2c_write(payload)
            self.gcode.respond_info(f"[DEBUG] _i2c_write_reg: pyload={payload}")
            return
        raise MS32008Error("I2C write function not found on bus object")

    # def _i2c_read_reg(self, reg, count):
    #     """Read 'count' bytes from register 'reg'."""
    #     # Typical register read: write register address, then read count bytes.
    #     if hasattr(self.i2c, 'i2c_read'):
    #         data = self.i2c.i2c_read([reg], count)
    #         logging.info(data)
    #         return list(data)
    #     raise MS32008Error("I2C read function not found on bus object")
    
    def _i2c_read_reg(self, reg, count):
        if hasattr(self.i2c, 'i2c_read'):
            raw = self.i2c.i2c_read([reg], count)
        else:
            raise MS32008Error("I2C read function not found on bus object")
        if isinstance(raw, dict):
            if raw.get("#name") != "i2c_read_response":
                raise MS32008Error(f"Invalid I2C protocol: {raw}")
            if "response" not in raw:
                raise MS32008Error(f"I2C response missing data field: {raw}")
            payload = raw["response"]   # usually bytes
        else:
            payload = raw
        if isinstance(payload, int):
            buf = bytes([payload])

        elif isinstance(payload, (bytes, bytearray)):
            buf = bytes(payload)

        elif isinstance(payload, str):
            # 1-to-1 byte mapping (safe for raw I2C data)
            buf = payload.encode("latin1")

        elif isinstance(payload, list):
            # flatten list into bytes
            flat = []
            for x in payload:
                if isinstance(x, int):
                    flat.append(x & 0xFF)
                elif isinstance(x, str):
                    flat.extend(x.encode("latin1"))
                elif isinstance(x, (bytes, bytearray)):
                    flat.extend(x)
                else:
                    raise MS32008Error(f"Invalid I2C element type: {type(x)} in payload")
            buf = bytes(flat)

        else:
            raise MS32008Error(f"Unsupported I2C data type: {type(payload)}")
        if len(buf) < count:
            raise MS32008Error(
                f"I2C read returned too few bytes: expected {count}, got {len(buf)}"
            )

        buf = buf[-count:]
        self.gcode.respond_info(f"[DEBUG] _i2c_read_reg: reg={reg} pyload={list(buf)}")
        return list(buf)


    # ---------------------------
    # Functional mappings
    # ---------------------------
    def soft_reset(self):
        # Read reg0, clear nRST, wait, set nRST (same as MS32008_SoftReset)
        r = self._i2c_read_reg(0x00, 1)
        val = r[0] & (~0x01)   # cmd_nRST_ENABLE bit assumed low-level; preserve mask logic
        self._i2c_write_reg(0x00, [val])
        time.sleep(0.005)
        val |= 0x01
        self._i2c_write_reg(0x00, [val])

    def hard_sleep(self, enable: bool):
        return self.soft_standby(enable)

    def soft_standby(self, enable: bool):
        r = self._i2c_read_reg(0x00, 1)
        val = r[0]
        if enable:
            val |= 0x02
        else:
            val &= (~0x02)
        self._i2c_write_reg(0x00, [val])

    def set_fsw(self, fsw_val):
        # update bits [4:2] in reg0 (mask 0x1c)
        r = self._i2c_read_reg(0x00, 1)
        val = r[0] & (~0x1c)
        val |= (fsw_val & 0x1c)
        self._i2c_write_reg(0x00, [val])

    def ext_clk(self, enable: bool):
        r = self._i2c_read_reg(0x00, 1)
        val = r[0]
        if enable:
            val |= 0x20  # useExtClk_ENABLE
        else:
            val &= (~0x20)
        self._i2c_write_reg(0x00, [val])

    def osc_off(self, enable: bool):
        r = self._i2c_read_reg(0x00, 1)
        val = r[0]
        if enable:
            val |= 0x10  # oscOFF_ENABLE
        else:
            val &= (~0x10)
        self._i2c_write_reg(0x00, [val])

    def ch_conf_load(self, ch_mask):
        # write one byte to reg 0x01 (CHxConfLoad)
        self._i2c_write_reg(0x01, [ch_mask])

    def ch_force_stop(self, ch_mask):
        self._i2c_write_reg(0x02, [ch_mask])

    def dc_ctrl(self, mode):
        # write reg 0x03: DCMotor control
        self._i2c_write_reg(0x03, [mode])

    def event_clr(self, mask):
        # reg 0x0d
        self._i2c_write_reg(0x0d, [mask])

    def watch_sel(self, enable: bool, sel):
        # reg 0x0E set watchEN + sel
        if enable:
            self._i2c_write_reg(0x0E, [0x80 | (sel & 0x7F)])
        else:
            self._i2c_write_reg(0x0E, [0x00 | (sel & 0x7F)])

    def read_chip_flag(self):
        return self._i2c_read_reg(0x0f, 1)[0]

    def _ch_base(self, ch):
        # Channel helpers: CHA base at 0x10, CHB base at 0x20
        if ch in ('A', 'CHA', 'a'):
            return 0x10
        elif ch in ('B', 'CHB', 'b'):
            return 0x20
        else:
            raise MS32008Error("Unknown channel '{}'".format(ch))

    def set_ch_pd_out(self, ch, enable: bool):
        base = self._ch_base(ch)
        r = self._i2c_read_reg(base + 0, 1)
        val = r[0]
        if enable:
            val |= 0x40   # CHx_PowDri_ENABLE
        else:
            val &= (~0x40)
        self._i2c_write_reg(base + 0, [val])

    def set_ch_record_rev(self, ch, enable: bool):
        base = self._ch_base(ch)
        r = self._i2c_read_reg(base + 0, 1)
        val = r[0]
        if enable:
            val |= 0x20   # CHx_recordRev_ENABLE
        else:
            val &= (~0x20)
        self._i2c_write_reg(base + 0, [val])

    def set_ch_dir(self, ch, cw: bool):
        base = self._ch_base(ch)
        r = self._i2c_read_reg(base + 1, 1)
        val = r[0]
        if cw:
            val &= (~0x01)   # CHx_Dir_CW -> clear bit
        else:
            val |= 0x01      # CHx_Dir_CCW -> set bit
        self._i2c_write_reg(base + 1, [val])

    def set_ch_stop_pos(self, ch, pos_mask):
        base = self._ch_base(ch)
        r = self._i2c_read_reg(base + 1, 1)
        val = (r[0] & (~0x0c)) | (pos_mask & 0x0c)
        self._i2c_write_reg(base + 1, [val])

    def set_ch_ms_mode(self, ch, ms_mask):
        base = self._ch_base(ch)
        r = self._i2c_read_reg(base + 1, 1)
        val = (r[0] & (~0xf0)) | (ms_mask & 0xf0)
        self._i2c_write_reg(base + 1, [val])

    def set_ch_pps(self, ch, pps):
        # write the value computed by FCLK/(pps<<4).
        fclk = FCLK_FREQUENCY
        u16reg = int(fclk / (int(pps) << 4))
        lo = u16reg & 0x00ff
        hi = (u16reg & 0xff00) >> 8
        base = self._ch_base(ch) | 0x80
        self._i2c_write_reg(base + 2, [lo, hi])

    def set_ch_stepnum(self, ch, step_num):
        # write two bytes little-endian at base+4
        lo = step_num & 0xff
        hi = (step_num >> 8) & 0xff
        base = self._ch_base(ch) | 0x80
        self._i2c_write_reg(base + 4, [lo, hi])

    def set_ch_current(self, ch, amp_percent):
        # write single byte at base+6
        base = self._ch_base(ch)
        v = int(amp_percent) & 0xff
        self._i2c_write_reg(base + 6, [v])

    def read_ch_status(self, ch):
        base = self._ch_base(ch)
        return self._i2c_read_reg(base + 13, 1)[0]

    def read_ch_pulse_record(self, ch):
        base = self._ch_base(ch)
        r = self._i2c_read_reg(base + 14, 2)
        return (r[1] << 8) | r[0]
    
    def force_enable_channel_power(self, ch):
        base = self._ch_base(ch)
        r = self._i2c_read_reg(base, 1)[0]
        r |= 0x40
        self._i2c_write_reg(base, [r])


    # ---------------------------
    # High-level convenience / composite ops
    # ---------------------------
    def init_chip(self,
                  default_pps=400,
                  default_step_fraction_div=10,
                  default_current=120):
        """
        Configure the chip with defaults similar to Init_MS32008() from control.c
        This writes the reg0 defaults, clears events, config channels, sets PPS/stepnum/current
        """
        # Soft reset
        self.soft_reset()
        self.read_chip_flag()
        # reg0 composition:
        # cmd_nRST_DISABLE | standby_DISABLE | stm_fsw_MUL2|useExtClk_ENABLE|oscOFF_DISABLE
        # We mimic same composition from control.c: value assembled into u8Reg[0]
        # reg0 = 0x00 
        # reg0 |= 0x00      # cmd_nRST_DISABLE (we assume bit meaning kept as in mc code)
        # reg0 &= (~0x02)   # standby_DISABLE -> clear standby bit
        # # set stm_fsw_MUL2 -> per C code value; here we set to mask 0x04 as example
        # reg0 |= 0x04
        # # useExtClk_ENABLE
        # reg0 |= 0x20
        # # oscOFF_DISABLE -> ensure bit cleared
        # reg0 &= (~0x10)
        reg0 = 0x01 | 0x00 | 0x04 | 0x00 | 0x00
        self._i2c_write_reg(0x00, [reg0])

        # DC motor to Hi-Z (reg 0x03)
        self.dc_ctrl(0x00)  # DCMotor_Hiz assumed 0x00

        # clear events
        self.event_clr(0x80 | 0x40)  # uvloClr|otsClr in original code (mask 0x03)

        # set channel config bytes at 0x10 & 0x20
        cha_byte0 = 0x40 | 0x00   # CHx_PowDri_ENABLE | CHx_recordRev_DISABLE
        cha_byte1 = 0x60 | 0x00   # msMode 1/256 etc | ForceStopPosDiv4 | Dir_CW
        # user may want different defaults; keep same as control.cs
        self._i2c_write_reg(0x10 | 0x80, [cha_byte0, cha_byte1])
        self._i2c_write_reg(0x20 | 0x80, [cha_byte0, cha_byte1])

        # set default PPS / step numbers / currents for both channels
        # control.c set u16CHApps = 400, step = pps/10
        pps = default_pps
        stepnum = int(pps / default_step_fraction_div)
        # self.watch_sel(enable=True, sel=0x02)
        # self.watch_sel(enable=True, sel=0x00)

        # self.set_ch_pps('A', pps)
        # self.set_ch_pps('B', pps)
        # self.set_ch_stepnum('A', stepnum)
        # self.set_ch_stepnum('B', stepnum)
        # self.set_ch_current('A', default_current)
        # self.set_ch_current('B', default_current)
        # # conf load both channels
        # # self.set_ch_pd_out(ch='A', enable=True)
        # # self.set_ch_pd_out(ch='B', enable=True)
        # self.ch_conf_load(0x80 | 0x40)  # ACH_confLoad|BCH_confLoad (mask 0x03)

    def move_channel(self, ch, pps, steps, dir_cw=True):
        """
        High-level move: set pps, stepnum, dir, then issue conf_load to start.
        This mirrors how control.c sets registers then calls CHxConfLoad.
        """
        self.set_ch_current(ch, 120)
        # self.set_ch_ms_mode(ch, 0x20)
        self.set_ch_pps(ch, pps)
        self.set_ch_stepnum(ch, steps)
        self.set_ch_dir(ch, dir_cw)

        # # 清除 ForceStop 位 -> 启动
        # r = self._i2c_read_reg(0x02, 1)[0]
        # if ch == 'A':
        #     r &= ~0x80   # 清 bit7
        # elif ch == 'B':
        #     r &= ~0x40
        # self._i2c_write_reg(0x02, [r])

        # Trigger config load for that channel only
        if ch in ('A', 'CHA', 'a'):
            # self.ch_force_stop(0x00)
            self.ch_conf_load(0x80)
        else:
            # self.ch_force_stop(0x00)
            self.ch_conf_load(0x40)

    def motor_control(self, ch):
        # ch = 'A' or 'B' or 'Z'
        r = self._i2c_read_reg(0x02, 1)[0]  # 读寄存器0x02
        if ch == 'A':
            r = (r | 0x80) if (r & 0x80) == 0 else (r & 0x7F)
        elif ch == 'B':
            r = (r | 0x40) if (r & 0x40) == 0 else (r & 0xBF)
        elif ch == 'Z':
            r = (r | 0x80) if (r & 0x80) == 0 else (r & 0x7F)
            r = (r | 0x40) if (r & 0x40) == 0 else (r & 0xBF)
        self._i2c_write_reg(0x02, [r])

    # ---------------------------
    # G-code command handlers
    # ---------------------------
    def gcmd_init(self, gcmd):
        self.init_chip()
        gcmd.respond_info("MS32008 initialized")

    def gcmd_read(self, gcmd):
        reg = gcmd.get_int('REG')
        count = gcmd.get_int('COUNT')
        data = self._i2c_read_reg(reg, count)
        hexstr = " ".join("0x%02X" % b for b in data)
        gcmd.respond_info("MS32008 READ: " + hexstr)

    def gcmd_write(self, gcmd):
        reg = gcmd.get_int('REG')
        raw = gcmd.get('DATA')
        if not raw:
            raise gcmd.error("DATA required")
        data = [int(x, 0) for x in raw.split()]
        self._i2c_write_reg(reg | 0x80, data)
        gcmd.respond_info("MS32008 WRITE OK")

    def gcmd_move(self, gcmd):
        ch = gcmd.get('CH', 'A')
        pps = gcmd.get_int('PPS')
        steps = gcmd.get_int('STEPS')
        dirv = gcmd.get('DIR', 'CW')
        cw = (dirv.upper() == 'CW')

        # self.force_enable_channel_power(ch)
        self.move_channel(ch, pps, steps, cw)

        gcmd.respond_info("MS32008 MOVE started")

    def gcmd_set(self, gcmd):
        # Generic setter for CURRENT/PPS/DIR/STEPNUM
        ch = gcmd.get('CH', 'A')
        if gcmd.has('CURRENT'):
            cur = gcmd.get_int('CURRENT')
            self.set_ch_current(ch, cur)
            gcmd.respond_info("CURRENT set")
            return
        if gcmd.has('PPS'):
            pps = gcmd.get_int('PPS')
            self.set_ch_pps(ch, pps)
            gcmd.respond_info("PPS set")
            return
        if gcmd.has('STEP'):
            s = gcmd.get_int('STEP')
            self.set_ch_stepnum(ch, s)
            gcmd.respond_info("STEPNUM set")
            return
        if gcmd.has('DIR'):
            d = gcmd.get('DIR')
            self.set_ch_dir(ch, d.upper() == 'CW')
            gcmd.respond_info("DIR set")
            return
        raise gcmd.error("No known parameter provided")

    def gcmd_dc(self, gcmd):
        mode = gcmd.get_int('MODE')
        self.dc_ctrl(mode)
        gcmd.respond_info("DC control set")

def load_config(config):
    return MS32008(config)
