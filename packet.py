# packet_format.py
#
# Satu-satunya tempat mendefinisikan payload UDP
# -------------------------------------------------

import struct
from dataclasses import dataclass

# ── 1. Atur urutan & tipe field di sini ──────────────────────────
#   <  = little-endian
#   i  = 4-byte signed int
#   f  = 4-byte float
#   B  = 1-byte unsigned char, dll.
STRUCT_FMT = "<if"          # contoh: angka (int32) + speed (float32)

# ── 2. Hitung ukuran total byte ─────────────────────────────────
STRUCT_SIZE = struct.calcsize(STRUCT_FMT)

@dataclass
class TxPacket:              # ► data yang KITA kirim ➜ robot
    angka: int
    speed = [0.0, 0.0]  # default speed = [0.0, 0.0] (robot tidak bergerak)
    led :int


@dataclass
class RxPacket:              # ► data yang KITA terima ◀ robot
    angka: int
    speed = [0.0, 0.0]  # default speed = [0.0, 0.0] (robot tidak bergerak)
    led :int


# ─────────────────────────────────────────────────────────────────
# Helper untuk (de)serialisasi
# ─────────────────────────────────────────────────────────────────
def pack_tx(p: TxPacket) -> bytes:
    """TxPacket → bytes"""
    return struct.pack(STRUCT_FMT, p.angka, p.speed)

def unpack_rx(b: bytes) -> RxPacket:
    """bytes → RxPacket"""
    angka, speed = struct.unpack(STRUCT_FMT, b[:STRUCT_SIZE])
    return RxPacket(angka, speed)
