#!/usr/bin/env python3
"""Hongdinisi 16-series pipetting module (N-C16R2-1000-Z1) over RS485 Modbus RTU: driver + command line.

    python3 pipette.py status                        state, error, tip, remaining ul, pressure, Z position
    python3 pipette.py home | stop | eject           home the piston, abort the motion, eject the tip
    python3 pipette.py aspirate <ul> [ul/s] [pre_ul] aspirate (default 100 ul/s, pre-aspiration 0)
    python3 pipette.py dispense <ul> [ul/s] [blow_ul]dispense (default 100 ul/s, air blow-back 0)
    python3 pipette.py lld [mode] [timeout_s] [mm/s] liquid level detection (mode 0 pressure, 1 capacitive, 2 both)
    python3 pipette.py z home | stop | move <mm> | rel <mm> [mm/s]    Z axis through the pipette (register 0x0312 forwarding)
    python3 pipette.py read <reg_hex> [n] [--holding] | write <reg_hex> <value>   raw registers
Options: --port (default: the FTDI adapter), --addr (Modbus address, default 1), --wait (block until idle).
Units follow the manual: volumes and speeds are sent in 0.1 ul / 0.1 ul/s, Z in 0.1 mm / 0.1 mm/s.
"""
import argparse, glob, struct, sys, time
import serial

STATES = {0: "idle", 1: "aspirating", 2: "dispensing", 3: "ejecting tip", 4: "detecting liquid", 5: "holding",
          6: "homing", 7: "stopped", 88: "not homed", 99: "error"}
CMD = {"home": 1, "stop": 2, "lld": 3, "aspirate": 4, "dispense": 5, "eject": 6}


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return struct.pack("<H", crc)


class Pipette:
    def __init__(self, port=None, addr=1, baud=115200, timeout=0.3):
        port = port or (glob.glob("/dev/serial/by-id/usb-FTDI*") or ["/dev/ttyUSB0"])[0]
        self.ser = serial.Serial(port, baud, timeout=timeout); self.addr = addr

    def _xfer(self, pdu, reply_len):
        frame = bytes([self.addr]) + pdu; frame += crc16(frame)
        self.ser.reset_input_buffer(); self.ser.write(frame)
        r = self.ser.read(reply_len)
        if len(r) < 5: raise RuntimeError("no reply (%d bytes) to %s" % (len(r), frame.hex(" ")))
        if r[1] & 0x80:
            raise RuntimeError("Modbus exception %d for function %02X" % (r[2], r[1] & 0x7F))
        if crc16(r[:-2]) != r[-2:]: raise RuntimeError("bad CRC in reply " + r.hex(" "))
        return r

    def read(self, reg, n=1, holding=False):
        fn = 0x03 if holding else 0x04
        r = self._xfer(struct.pack(">BHH", fn, reg, n), 5 + 2 * n)
        return list(struct.unpack(">%dH" % n, r[3:3 + 2 * n]))

    def write(self, reg, value):
        self._xfer(struct.pack(">BHH", 0x06, reg, value & 0xFFFF), 8)

    def write_many(self, reg, values):
        vals = b"".join(struct.pack(">H", v & 0xFFFF) for v in values)
        self._xfer(struct.pack(">BHHB", 0x10, reg, len(values), 2 * len(values)) + vals, 8)

    def command(self, name):
        self.write(0x0300, CMD[name])

    def state(self):
        return self.read(0x0010)[0]

    def status(self):
        v = self.read(0x0010, 16)
        z = self.read(0x001C, 2)
        return {"state": STATES.get(v[0], v[0]), "error": v[1], "aspirate_flags": v[2], "lld": v[3], "tip": bool(v[4]),
                "max_ul": v[5] / 10, "remaining_ul": v[6] / 10, "pressure_pa": struct.unpack("h", struct.pack("H", v[7]))[0],
                "z_mm": struct.unpack("h", struct.pack("H", z[0]))[0] / 10, "z_status": z[1]}

    def wait_idle(self, timeout=60):
        t0 = time.time()
        while time.time() - t0 < timeout:
            s = self.state()
            if s in (0, 5, 7, 88, 99): return s
            time.sleep(0.1)
        raise RuntimeError("still busy after %.0f s" % timeout)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("words", nargs="*"); ap.add_argument("--port"); ap.add_argument("--addr", type=int, default=1)
    ap.add_argument("--wait", action="store_true"); ap.add_argument("--holding", action="store_true")
    a = ap.parse_args(); w = a.words
    if not w: print(__doc__); return
    p = Pipette(a.port, a.addr); c = w[0]
    if c == "status":
        for k, v in p.status().items(): print("%-15s %s" % (k, v))
        return
    if c in ("home", "stop", "eject"):
        p.command(c); print("OK", c)
    elif c == "aspirate" and len(w) > 1:
        ul, rate, pre = float(w[1]), float(w[2]) if len(w) > 2 else 100.0, float(w[3]) if len(w) > 3 else 0.0
        p.write_many(0x0301, [int(pre * 10), int(ul * 10), int(rate * 10)]); p.command("aspirate")
        print("OK aspirate %.1f ul at %.1f ul/s (pre %.1f ul)" % (ul, rate, pre))
    elif c == "dispense" and len(w) > 1:
        ul, rate, blow = float(w[1]), float(w[2]) if len(w) > 2 else 100.0, float(w[3]) if len(w) > 3 else 0.0
        p.write_many(0x0306, [int(ul * 10), int(rate * 10), int(blow * 10), int(rate * 10)]); p.command("dispense")
        print("OK dispense %.1f ul at %.1f ul/s (blow-back %.1f ul)" % (ul, rate, blow))
    elif c == "lld":
        mode, tmo, spd = int(w[1]) if len(w) > 1 else 0, float(w[2]) if len(w) > 2 else 20.0, float(w[3]) if len(w) > 3 else 10.0
        p.write(0x030C, mode); p.write_many(0x0310, [int(tmo), int(spd * 10)]); p.command("lld")
        print("OK liquid level detection mode %d, timeout %.0f s, Z %.1f mm/s" % (mode, tmo, spd))
    elif c == "z" and len(w) > 1:
        p.write(0x0312, 1)                                      # pipette forwards Z commands
        s = w[1]
        if s == "home": p.write(0x0313, 0x8002); print("OK Z homing")
        elif s == "stop": p.write(0x0313, 0x8004); print("OK Z stop")
        elif s in ("move", "rel") and len(w) > 2:
            mm, spd = float(w[2]), float(w[3]) if len(w) > 3 else 20.0
            p.write(0x0316, int(spd * 10)); p.write(0x0317, 1 if s == "rel" else 0)
            p.write(0x0318, int(round(mm * 10)) & 0xFFFF); p.write(0x0313, 0x8001)
            print("OK Z %s %.1f mm at %.1f mm/s" % (s, mm, spd))
        else: print("usage: z home | stop | move <mm> [mm/s] | rel <mm> [mm/s]"); return
    elif c == "read" and len(w) > 1:
        n = int(w[2]) if len(w) > 2 else 1; vals = p.read(int(w[1], 16), n, a.holding)
        for i, v in enumerate(vals): print("0x%04X = %5d (0x%04X)" % (int(w[1], 16) + i, v, v))
        return
    elif c == "write" and len(w) > 2:
        p.write(int(w[1], 16), int(w[2], 0)); print("OK write"); return
    else:
        print(__doc__); return
    if a.wait:
        print("state:", STATES.get(p.wait_idle(), "?"))


if __name__ == "__main__":
    main()
