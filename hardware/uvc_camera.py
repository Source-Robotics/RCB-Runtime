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
  Camera.read_meta()      the camera's packet headers from its UVC metadata node (P2, 2026-11-24): the camera's own
                          clock (PTS, SCR) for utility/uvc_clock.py; parse_meta() reads one metadata buffer
  meta_node_for(path)     the metadata node that belongs to a video node (the same USB interface)

Proven on the MSG gripper's camera (USB 0bda:5844, MJPEG 1280x720 at 30 fps) with ~/Desktop/camera_test/box_camera.py
on 2026-11-21 / 23: 30.0 fps for 26 minutes, an unplug and a replug survived (read() raises OSError ENODEV). Its
metadata (recorded 2026-11-24): every packet header carries PTS + SCR, 242 per frame, one per 125 us microframe.
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

# The UVC metadata node (V4L2_BUF_TYPE_META_CAPTURE, format V4L2_META_FMT_UVC): per packet header the driver stores
# struct uvc_meta_buf — u64 ns (CLOCK_MONOTONIC when it read the header), u16 sof (the host's USB frame number then),
# u8 length (the camera's header length), u8 flags (bmHeaderInfo), then the header's PTS (u32) and SCR (u32 STC + u16
# SOF) when the flags say so. A block is stored only when its SCR differs from the last one (drivers/media/usb/uvc/
# uvc_video.c uvc_video_decode_meta), so a camera without SCR leaves none.
BUF_TYPE_META_CAPTURE = 13
META_FMT_UVC = struct.unpack("<I", b"UVCH")[0]
META_BUFFERS = 8
UVC_FID, UVC_EOF, UVC_PTS, UVC_SCR, UVC_ERR = 0x01, 0x02, 0x04, 0x08, 0x40
_META_HEAD = struct.Struct("<QHBB")
_META_FULL = struct.Struct("<QHBBIIH")             # a block with PTS and SCR: 22 bytes (every block of the MSG camera)


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


def _node_caps(path: str) -> int | None:
    """The node's own V4L2 capabilities (device_caps), or None when it cannot be opened or asked."""
    try:
        fd = os.open(path, os.O_RDWR | os.O_NONBLOCK)
    except OSError:
        return None
    try:
        cap = bytearray(104)
        fcntl.ioctl(fd, VIDIOC_QUERYCAP, cap, True)
        caps, dcaps = struct.unpack_from("<II", cap, 84)
        return dcaps if caps & CAP_DEVICE_CAPS else caps
    except OSError:
        return None
    finally:
        os.close(fd)


def _is_video_capture(path: str) -> bool:
    return bool((_node_caps(path) or 0) & CAP_VIDEO_CAPTURE)


def meta_node_for(video_path: str, dev_root: str = "/dev", sys_root: str = "/sys", caps=None) -> str | None:
    """The UVC metadata node of the camera whose video node is `video_path`: the /dev/videoN on the same USB interface
    that offers metadata capture (the MSG camera: video1 beside video0). None when there is none (another driver, or a
    kernel without UVC metadata). `caps` replaces _node_caps (tests)."""
    caps = caps or _node_caps
    name = os.path.basename(os.path.realpath(video_path))
    cls = os.path.join(sys_root, "class", "video4linux")
    iface = os.path.realpath(os.path.join(cls, name, "device"))
    for node in sorted(glob.glob(os.path.join(dev_root, "video*"))):
        other = os.path.basename(node)
        if other == name or os.path.realpath(os.path.join(cls, other, "device")) != iface:
            continue
        if (caps(node) or 0) & CAP_META_CAPTURE:
            return node
    return None


def parse_meta(data) -> list[tuple]:
    """The packet headers in one UVC metadata buffer: [(ns, host_sof, flags, pts, stc, dev_sof)], in arrival order.

    ns: CLOCK_MONOTONIC in ns when the driver read the header (when its USB transfer completed); host_sof: the host's
    USB frame number at that moment (11 bits); flags: the header's bmHeaderInfo (UVC_*); pts: the camera's
    presentation time stamp (its clock: the start of the frame), or None; stc, dev_sof: the source clock reference —
    the camera's clock and the USB frame number (11 bits) it was sampled at —, or None. A block's size follows from
    its flags (the driver stores the standard fields only), never from its length byte; a truncated block ends the
    list."""
    data = bytes(data)
    n = len(data)
    if n and n % 22 == 0 and all(f & (UVC_PTS | UVC_SCR) == UVC_PTS | UVC_SCR for f in data[11::22]):
        return [(ns, sof, fl, pts, stc, dsof & 0x7FF) for ns, sof, _ln, fl, pts, stc, dsof in _META_FULL.iter_unpack(data)]
    out, pos = [], 0
    while pos + _META_HEAD.size <= n:
        ns, sof, _ln, fl = _META_HEAD.unpack_from(data, pos)
        size = _META_HEAD.size + (4 if fl & UVC_PTS else 0) + (6 if fl & UVC_SCR else 0)
        if pos + size > n:
            break
        o = pos + _META_HEAD.size
        pts = stc = dsof = None
        if fl & UVC_PTS:
            pts = struct.unpack_from("<I", data, o)[0]
            o += 4
        if fl & UVC_SCR:
            stc, dsof = struct.unpack_from("<IH", data, o)
            dsof &= 0x7FF
        out.append((ns, sof, fl, pts, stc, dsof))
        pos += size
    return out


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
    or None when no frame is ready; it raises OSError (ENODEV) once the camera is unplugged.

    With `meta` (P2, 2026-11-24) it also streams the camera's UVC metadata node, set up FIRST: the driver refuses its
    format once the video buffers exist (EBUSY, found 2026-11-23). read_meta() hands back what arrived. No metadata
    node, or one that refuses: the video streams anyway, `meta_problem` says why."""

    def __init__(self, path: str, width: int, height: int, fps: float, buffers: int = 4, meta: bool = False):
        self.path = path
        self.closed = False
        self.meta_fd: int | None = None
        self.meta_maps: list = []
        self.meta_path: str | None = None
        self.meta_problem = "" if meta else "not asked for"
        self.fd = os.open(path, os.O_RDWR | os.O_NONBLOCK)
        self.maps: list = []
        try:
            if meta:
                self._open_meta()
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
    def _buf(index: int = 0, kind: int = BUF_TYPE_CAPTURE) -> bytearray:
        buf = bytearray(88)
        struct.pack_into("<II", buf, 0, index, kind)
        struct.pack_into("<I", buf, 60, MEMORY_MMAP)
        return buf

    def _open_meta(self) -> None:
        """The metadata node streaming, or `meta_problem` set and nothing of it left open."""
        node = meta_node_for(self.path)
        if node is None:
            self.meta_problem = "the camera has no UVC metadata node"
            return
        try:
            fd = self.meta_fd = os.open(node, os.O_RDWR | os.O_NONBLOCK)
            fmt = bytearray(208)
            struct.pack_into("<I", fmt, 0, BUF_TYPE_META_CAPTURE)
            struct.pack_into("<II", fmt, 8, META_FMT_UVC, 0)
            fcntl.ioctl(fd, VIDIOC_S_FMT, fmt, True)
            if struct.unpack_from("<I", fmt, 8)[0] != META_FMT_UVC:
                raise OSError(errno.EINVAL, f"{node} offers {fourcc(struct.unpack_from('<I', fmt, 8)[0])}, not UVCH")
            req = bytearray(20)
            struct.pack_into("<III", req, 0, META_BUFFERS, BUF_TYPE_META_CAPTURE, MEMORY_MMAP)
            fcntl.ioctl(fd, VIDIOC_REQBUFS, req, True)
            for index in range(struct.unpack_from("<I", req, 0)[0]):
                buf = self._buf(index, BUF_TYPE_META_CAPTURE)
                fcntl.ioctl(fd, VIDIOC_QUERYBUF, buf, True)
                offset, length = struct.unpack_from("<I", buf, 64)[0], struct.unpack_from("<I", buf, 72)[0]
                self.meta_maps.append(mmap.mmap(fd, length, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE,
                                                offset=offset))
                fcntl.ioctl(fd, VIDIOC_QBUF, buf, True)
            fcntl.ioctl(fd, VIDIOC_STREAMON, struct.pack("<i", BUF_TYPE_META_CAPTURE))
            self.meta_path = node
        except OSError as e:
            self.meta_problem = f"{node}: {e}"
            self._close_meta()

    def _close_meta(self) -> None:
        fd = self.meta_fd
        if fd is None:
            return
        try:
            fcntl.ioctl(fd, VIDIOC_STREAMOFF, struct.pack("<i", BUF_TYPE_META_CAPTURE))
        except OSError:
            pass
        for m in self.meta_maps:
            m.close()
        self.meta_maps = []
        os.close(fd)
        self.meta_fd = None

    def stop_meta(self, problem: str) -> None:
        """Close the metadata node while the video goes on (it failed alone; review 2026-11-25, F10): read_meta()
        returns nothing from now on, meta_path is None and meta_problem says why."""
        self._close_meta()
        self.meta_path = None
        self.meta_problem = problem

    def read_meta(self) -> list[tuple]:
        """Every metadata buffer the driver has finished, oldest first: [(sequence, error, data)] — the sequence is
        the video frame's it belongs to, error the driver's flag (a buffer that overflowed), data its bytes
        (parse_meta() reads them; utility/uvc_clock.py has a faster reader). Empty without a metadata node or when
        nothing is ready."""
        out: list[tuple] = []
        fd = self.meta_fd
        while fd is not None:
            buf = self._buf(0, BUF_TYPE_META_CAPTURE)
            try:
                fcntl.ioctl(fd, VIDIOC_DQBUF, buf, True)
            except BlockingIOError:
                break
            index, _, used, flags = struct.unpack_from("<IIII", buf, 0)
            seq = struct.unpack_from("<I", buf, 56)[0]
            data = self.meta_maps[index][:used]
            fcntl.ioctl(fd, VIDIOC_QBUF, self._buf(index, BUF_TYPE_META_CAPTURE), True)
            out.append((seq, bool(flags & BUF_FLAG_ERROR), data))
        return out

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
        self._close_meta()
        self.closed = True


# is_complete_jpeg moved to communication/camera_protocol.py on 2026-11-25 (the analysis kit needs it on a PC)
from communication.camera_protocol import is_complete_jpeg  # noqa: E402,F401
