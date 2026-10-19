"""时钟抽象：真实时钟与可注入的测试时钟。

对外时间字符串统一为 UTC ISO 8601 毫秒精度（YYYY-MM-DDTHH:mm:ss.sssZ）。
单元测试通过构造 FakeClock 注入时间，正式入口只使用 Clock。
"""

import time
from datetime import datetime, timedelta, timezone

UTC = timezone.utc


def utc_iso(dt):
    """把 datetime 转成仓库约定的 UTC ISO 8601 毫秒字符串。"""
    text = dt.astimezone(UTC).strftime("%Y-%m-%dT%H:%M:%S.")
    return text + "%03dZ" % (dt.microsecond // 1000)


def parse_utc_iso(text):
    """解析 UTC ISO 8601 字符串，接受结尾 Z 或 +00:00。"""
    if not isinstance(text, str):
        raise ValueError("不是有效的时间字符串: %r" % (text,))
    cleaned = text.strip()
    if cleaned.endswith("Z") or cleaned.endswith("z"):
        cleaned = cleaned[:-1] + "+00:00"
    return datetime.fromisoformat(cleaned).astimezone(UTC)


def run_id_timestamp(dt):
    """run_id 使用的时间戳：YYYYMMDDTHHMMSSffffffZ（微秒）。"""
    return dt.astimezone(UTC).strftime("%Y%m%dT%H%M%S%fZ")


class Clock:
    """真实时钟。"""

    def utc_now(self):
        return datetime.now(UTC)

    def monotonic(self):
        return time.monotonic()

    def sleep(self, seconds):
        if seconds and seconds > 0:
            time.sleep(seconds)


class FakeClock(Clock):
    """测试用注入时钟；绝不由正式入口构造。"""

    def __init__(self, monotonic=1000.0, utc=None):
        self._monotonic = float(monotonic)
        self._utc = utc if utc is not None else datetime(2026, 1, 1, tzinfo=UTC)

    def utc_now(self):
        return self._utc

    def monotonic(self):
        return self._monotonic

    def sleep(self, seconds):
        self.advance(seconds or 0.0)

    def advance(self, seconds):
        seconds = float(seconds)
        if seconds < 0:
            raise ValueError("不能回拨假时钟")
        self._monotonic += seconds
        self._utc = self._utc + timedelta(seconds=seconds)
