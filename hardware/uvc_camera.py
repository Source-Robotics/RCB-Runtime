"""A USB (UVC) camera through Linux's V4L2 interface, standard library only (2026-11-23, TODO/camera_plan.md P1).

What the camera server (camera_server.py) needs and nothing more:

  find_cameras()          the video capture nodes on USB, with their USB id ("vvvv:pppp") and stable by-id path
  describe(path)          formats, sizes and frame rates the camera offers, and its controls with their current values
  Camera(path, w, h, fps) MJPEG capture through memory-mapped buffers: read() hands back one finished JPEG (a copy) with
                          the driver's timestamp (CLOCK_MONOTONIC) and sequence number; nothing is decoded here
  Camera.controls() / set_control(key, value)
                          the camera's own controls by a stable key: the V4L2 name lower-cased, every run of other
                          characters one '_' ("Exposure Time, Absolute" -> exposure_time_absolute), the names v4l2-ctl
                          prints

Proven on the MSG gripper's camera (USB 0bda:5844, MJPEG 1280x720 at 30 fps) with ~/Desktop/camera_test/box_camera.py
on 2026-11-21 / 23: 30.0 fps for 26 minutes, an unplug and a replug survived (read() raises OSError ENODEV).
The struct sizes are the 64-bit kernel ABI's (the Pi 5); importing this module on a 32-bit Python raises ImportError.
"""
from __future__ import annotations

import errno
import fcntl
import glob
import mmap
import os
import re
import struct

if struct.calcsize("P") != 8:
    raise ImportError("hardware.uvc_camera is written for a 64-bit kernel ABI (the V4L2 struct sizes)")


def _ioc(direction, nr, size):
    return (direction << 30) | (size << 16) | (ord("V") << 8) | nr


_R, _W, _RW = 2, 1, 3
VIDIOC_QUERYCAP = _ioc(_R, 0, 104)              # struct v4l2_capability
VIDIOC_ENUM_FMT = _ioc(_RW, 2, 64)              # struct v4l2_fmtdesc
VIDIOC_S_FMT = _ioc(_RW, 5, 208)                # struct v4l2_format
VIDIOC_REQBUFS = _ioc(_RW, 8, 20)               # struct v4l2_requestbuffers
VIDIOC_QUERYBUF = _ioc(_RW, 9, 88)              # struct v4l2_buffer
VIDIOC_QBUF = _ioc(_RW, 15, 88)
VIDIOC_DQBUF = _ioc(_RW, 17, 88)
VIDIOC_STREAMON = _ioc(_W, 18, 4)
VIDIOC_STREAMOFF = _ioc(_W, 19, 4)
VIDIOC_S_PARM = _ioc(_RW, 22, 204)              # struct v4l2_streamparm
VIDIOC_G_CTRL = _ioc(_RW, 27, 8)                # struct v4l2_control
VIDIOC_S_CTRL = _ioc(_RW, 28, 8)
VIDIOC_QUERYCTRL = _ioc(_RW, 36, 68)            # struct v4l2_queryctrl
VIDIOC_QUERYMENU = _ioc(_RW, 37, 44)            # struct v4l2_querymenu
VIDIOC_ENUM_FRAMESIZES = _ioc(_RW, 74, 44)      # struct v4l2_frmsizeenum
VIDIOC_ENUM_FRAMEINTERVALS = _ioc(_RW, 75, 52)  # struct v4l2_frmivalenum

BUF_TYPE_CAPTURE = 1
MEMORY_MMAP = 1
FIELD_NONE = 1
CAP_VIDEO_CAPTURE = 0x00000001
CAP_META_CAPTURE = 0x00800000
CAP_DEVICE_CAPS = 0x80000000
CTRL_FLAG_NEXT = 0x80000000
CTRL_FLAG_DISABLED = 0x0001
CTRL_FLAG_READ_ONLY = 0x0004
CTRL_FLAG_INACTIVE = 0x0010
CTRL_FLAG_WRITE_ONLY = 0x0040
BUF_FLAG_ERROR = 0x0040
CTRL_CLASS = 6
CTRL_TYPES = {1: "int", 2: "bool", 3: "menu", 4: "button", 5: "int64", 7: "string", 8: "bitmask", 9: "intmenu"}
MJPG = struct.unpack("<I", b"MJPG")[0]


def fourcc(value: int) -> str:
    return struct.pack("<I", value).decode("ascii", "replace")


def _cstr(raw) -> str:
    return bytes(raw).split(b"\0", 1)[0].decode("ascii", "replace")


def control_key(name: str) -> str:
    """The stable key of a V4L2 control name: "Exposure Time, Absolute" -> "exposure_time_absolute"."""
    return re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")


def _enum(fd, request, size, fill):
    """The filled struct for index 0, 1, ... until the driver answers EINVAL (the end of the list)."""
    index = 0
    while True:
        buf = bytearray(size)
        fill(buf, index)
        try:
            fcntl.ioctl(fd, request, buf, True)
        except OSError as e:
            if e.errno == errno.EINVAL:
                return
            raise
        yield buf
        index += 1


# ---------------------------------------------------------------- finding cameras

def _usb_id(video_name: str, sys_root: str = "/sys") -> str | None:
    """"vvvv:pppp" of the USB device a /dev/videoN belongs to, or None (not on USB / not found)."""
    path = os.path.realpath(os.path.join(sys_root, "class", "video4linux", video_name, "device"))
    for _ in range(4):                                   # the interface, then its USB device
        vendor, product = os.path.join(path, "idVendor"), os.path.join(path, "idProduct")
        if os.path.exists(vendor) and os.path.exists(product):
            try:
                with open(vendor) as fv, open(product) as fp:
                    return f"{fv.read().strip().lower()}:{fp.read().strip().lower()}"
            except OSError:
                return None
        path = os.path.dirname(path)
    return None


def _is_video_capture(path: str) -> bool:
    try:
        fd = os.open(path, os.O_RDWR | os.O_NONBLOCK)
    except OSError:
        return False
    try:
        cap = bytearray(104)
        fcntl.ioctl(fd, VIDIOC_QUERYCAP, cap, True)
        caps, dcaps = struct.unpack_from("<II", cap, 84)
        node = dcaps if caps & CAP_DEVICE_CAPS else caps
        return bool(node & CAP_VIDEO_CAPTURE)
    except OSError:
        return False
    finally:
        os.close(fd)


def find_cameras(dev_root: str = "/dev", sys_root: str = "/sys") -> list[dict]:
    """The USB video CAPTURE nodes (metadata nodes left out): [{"path": the stable /dev/v4l/by-id name (or /dev/videoN),
    "node": "/dev/videoN", "usb_id": "vvvv:pppp"}], sorted by path. Nodes this process may not open are left out."""
    by_id = {}
    for link in glob.glob(os.path.join(dev_root, "v4l", "by-id", "*")):
        by_id[os.path.realpath(link)] = link
    found = []
    for node in sorted(glob.glob(os.path.join(dev_root, "video*"))):
        usb = _usb_id(os.path.basename(node), sys_root)
        if usb is None or not _is_video_capture(node):
            continue
        found.append({"path": by_id.get(os.path.realpath(node), node), "node": node, "usb_id": usb})
    return sorted(found, key=lambda c: c["path"])


# ---------------------------------------------------------------- describing a camera

def _modes(fd) -> list[dict]:
    """[{"format": "MJPG", "width", "height", "fps": [..]}] — discrete sizes only (UVC cameras list them)."""
    modes = []
    for fbuf in _enum(fd, VIDIOC_ENUM_FMT, 64, lambda b, i: struct.pack_into("<II", b, 0, i, BUF_TYPE_CAPTURE)):
        pix = struct.unpack_from("<I", fbuf, 44)[0]
        seen = set()
        for sbuf in _enum(fd, VIDIOC_ENUM_FRAMESIZES, 44, lambda b, i, p=pix: struct.pack_into("<II", b, 0, i, p)):
            if struct.unpack_from("<I", sbuf, 8)[0] != 1:      # stepwise / continuous: not a UVC camera's way
                break
            w, h = struct.unpack_from("<II", sbuf, 12)
            if (w, h) in seen:                                 # some cameras list a size twice
                continue
            seen.add((w, h))
            rates = []
            fill = lambda b, i, p=pix, ww=w, hh=h: struct.pack_into("<IIII", b, 0, i, p, ww, hh)  # noqa: E731
            for ibuf in _enum(fd, VIDIOC_ENUM_FRAMEINTERVALS, 52, fill):
                if struct.unpack_from("<I", ibuf, 16)[0] != 1:
                    break
                num, den = struct.unpack_from("<II", ibuf, 20)
                if num and round(den / num, 3) not in rates:
                    rates.append(round(den / num, 3))
            modes.append({"format": fourcc(pix), "width": w, "height": h, "fps": sorted(rates, reverse=True)})
    return modes


def _menu(fd, cid, lo, hi, ctype) -> list:
    out: list = []
    for index in range(lo, hi + 1):
        buf = bytearray(44)
        struct.pack_into("<II", buf, 0, cid, index)
        try:
            fcntl.ioctl(fd, VIDIOC_QUERYMENU, buf, True)
        except OSError:
            continue
        out.append([index, _cstr(buf[8:40]) if ctype == 3 else struct.unpack_from("<q", buf, 8)[0]])
    return out


def _get_ctrl(fd, cid):
    buf = bytearray(8)
    struct.pack_into("<I", buf, 0, cid)
    try:
        fcntl.ioctl(fd, VIDIOC_G_CTRL, buf, True)
    except OSError:
        return None
    return struct.unpack_from("<i", buf, 4)[0]


def _controls(fd) -> list[dict]:
    """Every enabled control: key, name, type, min, max, step, default, value (None when unreadable), menu
    ([[value, name], ...] for menus), read_only, inactive (the camera ignores it now, e.g. the exposure time while
    auto exposure is on), id."""
    out: list[dict] = []
    qid = CTRL_FLAG_NEXT
    while True:
        buf = bytearray(68)
        struct.pack_into("<I", buf, 0, qid)
        try:
            fcntl.ioctl(fd, VIDIOC_QUERYCTRL, buf, True)
        except OSError as e:
            if e.errno == errno.EINVAL:
                return out
            raise
        cid, ctype = struct.unpack_from("<II", buf, 0)
        lo, hi, step, default, flags = struct.unpack_from("<iiiiI", buf, 40)
        qid = cid | CTRL_FLAG_NEXT
        if ctype == CTRL_CLASS or flags & CTRL_FLAG_DISABLED:
            continue
        name = _cstr(buf[8:40])
        out.append({"key": control_key(name), "name": name, "type": CTRL_TYPES.get(ctype, str(ctype)),
                    "min": lo, "max": hi, "step": step, "default": default,
                    "value": None if flags & CTRL_FLAG_WRITE_ONLY else _get_ctrl(fd, cid),
                    "menu": _menu(fd, cid, lo, hi, ctype) if ctype in (3, 9) else [],
                    "read_only": bool(flags & CTRL_FLAG_READ_ONLY), "inactive": bool(flags & CTRL_FLAG_INACTIVE),
                    "id": cid})


def describe(path: str) -> dict:
    """{"card", "driver", "bus", "modes": [...], "controls": [...]} of a video capture node (opened and closed again,
    never streamed). Raises OSError when it cannot be opened."""
    fd = os.open(path, os.O_RDWR | os.O_NONBLOCK)
    try:
        cap = bytearray(104)
        fcntl.ioctl(fd, VIDIOC_QUERYCAP, cap, True)
        return {"card": _cstr(cap[16:48]), "driver": _cstr(cap[0:16]), "bus": _cstr(cap[48:80]),
                "modes": _modes(fd), "controls": _controls(fd)}
    finally:
        os.close(fd)


# ---------------------------------------------------------------- capturing

class Camera:
    """MJPEG capture through V4L2 memory-mapped buffers. Opening sets the mode (the camera may pick the nearest size /
    rate it has: `width`, `height`, `fps` say what it took). read() returns (jpeg, driver timestamp s, sequence, flags)
    or None when no frame is ready; it raises OSError (ENODEV) once the camera is unplugged."""

    def __init__(self, path: str, width: int, height: int, fps: float, buffers: int = 4):
        self.path = path
        self.closed = False
        self.fd = os.open(path, os.O_RDWR | os.O_NONBLOCK)
        self.maps: list = []
        try:
            fmt = bytearray(208)
            struct.pack_into("<I", fmt, 0, BUF_TYPE_CAPTURE)
            struct.pack_into("<IIII", fmt, 8, int(width), int(height), MJPG, FIELD_NONE)
            fcntl.ioctl(self.fd, VIDIOC_S_FMT, fmt, True)
            self.width, self.height, pix = struct.unpack_from("<III", fmt, 8)
            if pix != MJPG:
                raise OSError(errno.EINVAL, f"the camera offers {fourcc(pix)}, not MJPEG, at {width}x{height}")
            parm = bytearray(204)
            struct.pack_into("<I", parm, 0, BUF_TYPE_CAPTURE)
            struct.pack_into("<II", parm, 12, 1000, int(round(float(fps) * 1000)))
            fcntl.ioctl(self.fd, VIDIOC_S_PARM, parm, True)
            num, den = struct.unpack_from("<II", parm, 12)
            self.fps = den / num if num else 0.0
            req = bytearray(20)
            struct.pack_into("<III", req, 0, buffers, BUF_TYPE_CAPTURE, MEMORY_MMAP)
            fcntl.ioctl(self.fd, VIDIOC_REQBUFS, req, True)
            for index in range(struct.unpack_from("<I", req, 0)[0]):
                buf = self._buf(index)
                fcntl.ioctl(self.fd, VIDIOC_QUERYBUF, buf, True)
                offset, length = struct.unpack_from("<I", buf, 64)[0], struct.unpack_from("<I", buf, 72)[0]
                self.maps.append(mmap.mmap(self.fd, length, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE,
                                           offset=offset))
                fcntl.ioctl(self.fd, VIDIOC_QBUF, buf, True)
            fcntl.ioctl(self.fd, VIDIOC_STREAMON, struct.pack("<i", BUF_TYPE_CAPTURE))
        except BaseException:
            self.close()
            raise

    @staticmethod
    def _buf(index: int = 0) -> bytearray:
        buf = bytearray(88)
        struct.pack_into("<II", buf, 0, index, BUF_TYPE_CAPTURE)
        struct.pack_into("<I", buf, 60, MEMORY_MMAP)
        return buf

    def fileno(self) -> int:
        return self.fd

    def read(self):
        buf = self._buf()
        try:
            fcntl.ioctl(self.fd, VIDIOC_DQBUF, buf, True)
        except BlockingIOError:
            return None
        index, _, used, flags = struct.unpack_from("<IIII", buf, 0)
        sec, usec = struct.unpack_from("<qq", buf, 24)
        seq = struct.unpack_from("<I", buf, 56)[0]
        data = self.maps[index][:used]
        fcntl.ioctl(self.fd, VIDIOC_QBUF, self._buf(index), True)
        return data, sec + usec * 1e-6, seq, flags

    def controls(self) -> list[dict]:
        return _controls(self.fd)

    def set_control(self, key: str, value: int) -> None:
        """Set a control by its key. Raises KeyError for a key the camera does not have, ValueError for a value
        outside its range or a read-only control, OSError when the camera refuses it."""
        for c in _controls(self.fd):
            if c["key"] == key:
                break
        else:
            raise KeyError(key)
        if c["read_only"]:
            raise ValueError(f"{key} is read-only")
        value = int(value)
        if not c["min"] <= value <= c["max"]:
            raise ValueError(f"{key} = {value} is outside {c['min']}..{c['max']}")
        buf = bytearray(8)
        struct.pack_into("<Ii", buf, 0, c["id"], value)
        fcntl.ioctl(self.fd, VIDIOC_S_CTRL, buf, True)

    def close(self) -> None:
        if self.closed:
            return
        try:
            fcntl.ioctl(self.fd, VIDIOC_STREAMOFF, struct.pack("<i", BUF_TYPE_CAPTURE))
        except OSError:
            pass
        for m in self.maps:
            m.close()
        self.maps = []
        os.close(self.fd)
        self.closed = True


def is_complete_jpeg(data: bytes) -> bool:
    """Starts with SOI and ends (trailing zero padding aside) with EOI."""
    return data[:2] == b"\xff\xd8" and data.rstrip(b"\0")[-2:] == b"\xff\xd9"
