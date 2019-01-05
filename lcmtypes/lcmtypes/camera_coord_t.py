"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class camera_coord_t(object):
    __slots__ = ["x", "y", "z", "v_x", "v_y", "v_z", "mode", "utime"]

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.v_x = 0.0
        self.v_y = 0.0
        self.v_z = 0.0
        self.mode = 0
        self.utime = 0

    def encode(self):
        buf = BytesIO()
        buf.write(camera_coord_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">ddddddqq", self.x, self.y, self.z, self.v_x, self.v_y, self.v_z, self.mode, self.utime))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != camera_coord_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return camera_coord_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = camera_coord_t()
        self.x, self.y, self.z, self.v_x, self.v_y, self.v_z, self.mode, self.utime = struct.unpack(">ddddddqq", buf.read(64))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if camera_coord_t in parents: return 0
        tmphash = (0xe7a63d3acc61be77) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if camera_coord_t._packed_fingerprint is None:
            camera_coord_t._packed_fingerprint = struct.pack(">Q", camera_coord_t._get_hash_recursive([]))
        return camera_coord_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)
