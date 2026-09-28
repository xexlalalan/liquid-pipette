#!/usr/bin/env python3
"""/pipette — the pipetting module (Hongdinisi 16-series, RS485 Modbus RTU). Owns the serial port.

Services: /pipette/home, /pipette/stop, /pipette/eject, /pipette/z_home, /pipette/z_stop (Trigger);
          /pipette/aspirate, /pipette/dispense (Volume: ul, rate, extra = pre-aspiration / blow-back);
          /pipette/lld (Lld); /pipette/z_move (MoveZ: mm, absolute).
Topic:    /pipette/status (String JSON, 2 Hz). Motion services block until the pipette is idle again.
"""
import json, threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
from std_msgs.msg import String
from harness_msgs.srv import Volume, Lld, MoveZ
from .pipette import Pipette, STATES


class PipetteNode(Node):
    def __init__(self):
        super().__init__("pipette_node")
        self.declare_parameter("port", ""); self.declare_parameter("addr", 1); self.declare_parameter("z_speed", 20.0)
        self.p = Pipette(self.get_parameter("port").value or None, self.get_parameter("addr").value)
        self.lock = threading.Lock()                 # one Modbus transaction at a time
        cb = ReentrantCallbackGroup()
        self.pub = self.create_publisher(String, "/pipette/status", 10)
        self.create_timer(0.5, self.tick, callback_group=cb)
        for name in ("home", "stop", "eject"):
            self.create_service(Trigger, "/pipette/" + name, self._cmd(name), callback_group=cb)
        self.create_service(Trigger, "/pipette/z_home", self._z(0x8002, "Z homing"), callback_group=cb)
        self.create_service(Trigger, "/pipette/z_stop", self._z(0x8004, "Z stop"), callback_group=cb)
        self.create_service(Volume, "/pipette/aspirate", self.aspirate, callback_group=cb)
        self.create_service(Volume, "/pipette/dispense", self.dispense, callback_group=cb)
        self.create_service(Lld, "/pipette/lld", self.lld, callback_group=cb)
        self.create_service(MoveZ, "/pipette/z_move", self.z_move, callback_group=cb)
        self.get_logger().info("pipette node up on %s" % self.p.ser.port)

    def tick(self):
        try:
            with self.lock: st = self.p.status()
            st["online"] = True
        except Exception as e:
            st = {"online": False, "error": str(e)}
        self.pub.publish(String(data=json.dumps(st)))

    def _finish(self, res, what, wait=True):
        try:
            if wait:
                with self.lock: s = self.p.wait_idle(120)
                res.success = s in (0, 5); res.message = "%s: %s" % (what, STATES.get(s, s))
            else:
                res.success = True; res.message = what
        except Exception as e:
            res.success = False; res.message = str(e)
        return res

    def _cmd(self, name):
        def cb(req, res):
            try:
                with self.lock: self.p.command(name)
            except Exception as e:
                res.success = False; res.message = str(e); return res
            return self._finish(res, name, wait=(name != "stop"))
        return cb

    def _z(self, code, what):
        def cb(req, res):
            try:
                with self.lock: self.p.write(0x0312, 1); self.p.write(0x0313, code)
                res.success = True; res.message = what
            except Exception as e:
                res.success = False; res.message = str(e)
            return res
        return cb

    def aspirate(self, req, res):
        try:
            with self.lock:
                self.p.write_many(0x0301, [int(req.extra * 10), int(req.ul * 10), int((req.rate or 100) * 10)]); self.p.command("aspirate")
        except Exception as e:
            res.success = False; res.message = str(e); return res
        return self._finish(res, "aspirate %.1f ul" % req.ul)

    def dispense(self, req, res):
        try:
            with self.lock:
                r = int((req.rate or 100) * 10)
                self.p.write_many(0x0306, [int(req.ul * 10), r, int(req.extra * 10), r]); self.p.command("dispense")
        except Exception as e:
            res.success = False; res.message = str(e); return res
        return self._finish(res, "dispense %.1f ul" % req.ul)

    def lld(self, req, res):
        try:
            with self.lock:
                self.p.write(0x030C, req.mode); self.p.write_many(0x0310, [int(req.timeout_s or 20), int((req.speed_mm_s or 10) * 10)])
                self.p.command("lld")
        except Exception as e:
            res.success = False; res.message = str(e); return res
        return self._finish(res, "liquid level detection")

    def z_move(self, req, res):
        try:
            with self.lock:
                self.p.write(0x0312, 1); self.p.write(0x0316, int(self.get_parameter("z_speed").value * 10))
                self.p.write(0x0317, 0 if req.absolute else 1); self.p.write(0x0318, int(round(req.mm * 10)) & 0xFFFF)
                self.p.write(0x0313, 0x8001)
            res.success = True; res.message = "Z %s %.1f mm" % ("to" if req.absolute else "by", req.mm)
        except Exception as e:
            res.success = False; res.message = str(e)
        return res


def main():
    rclpy.init(); n = PipetteNode(); ex = MultiThreadedExecutor(num_threads=4); ex.add_node(n)
    try: ex.spin()
    except KeyboardInterrupt: pass
    finally: n.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
