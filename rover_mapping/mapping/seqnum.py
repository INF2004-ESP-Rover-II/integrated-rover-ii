"""Wrap-safe sequence number comparison (Buddy 3's rule: newer = ahead by 1..half-range)."""
from .config import SEQ_BITS


def is_newer(new: int, last: int, bits: int = SEQ_BITS) -> bool:
    mod = 1 << bits
    diff = (new - last) % mod
    return 1 <= diff < mod // 2


def next_seq(seq: int, bits: int = SEQ_BITS) -> int:
    return (seq + 1) % (1 << bits)
