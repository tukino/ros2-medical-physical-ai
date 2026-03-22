"""Observability helpers (Day11).

This module is intentionally rclpy-free to keep unit tests lightweight.

Event log format (single-line, grep-friendly):
  event=<name> key=value key=value

- Keys are emitted in sorted order for stable diffs.
- Values are sanitized to avoid embedded newlines.
"""

from __future__ import annotations

from typing import Any


def sanitize_value(value: Any) -> str:
    if value is None:
        return 'null'

    if isinstance(value, bool):
        return 'true' if value else 'false'

    if isinstance(value, (int, float)):
        return str(value)

    text = str(value)

    # Keep logs single-line and shell-friendly.
    text = text.replace('\r', '\\r').replace('\n', '\\n').replace('\t', '\\t')

    # Avoid breaking key=value parsing.
    text = text.replace(' ', '_')

    return text


def format_event(event: str, **fields: Any) -> str:
    if not str(event).strip():
        raise ValueError('event must be non-empty')

    items = [f"event={sanitize_value(event)}"]

    for key in sorted(fields.keys()):
        if key is None:
            continue
        key_str = str(key).strip()
        if not key_str:
            continue
        items.append(f"{key_str}={sanitize_value(fields[key])}")

    return ' '.join(items)


def format_vitals_drop_event(
    *,
    node: str,
    ns: str,
    tick: int,
    elapsed_sec: float,
    drop_rate: float,
) -> str:
    return format_event(
        'vitals.drop',
        node=str(node),
        ns=str(ns),
        tick=int(tick),
        elapsed_sec=float(elapsed_sec),
        drop_rate=float(drop_rate),
    )


def format_vitals_enqueue_delayed_event(
    *,
    node: str,
    ns: str,
    tick: int,
    elapsed_sec: float,
    delay_ms: int,
    jitter_ms: int,
    delay_ticks: int,
    due_tick: int,
    queue_len: int,
    clamped: bool,
) -> str:
    return format_event(
        'vitals.enqueue_delayed',
        node=str(node),
        ns=str(ns),
        tick=int(tick),
        elapsed_sec=float(elapsed_sec),
        delay_ms=int(delay_ms),
        jitter_ms=int(jitter_ms),
        delay_ticks=int(delay_ticks),
        due_tick=int(due_tick),
        queue_len=int(queue_len),
        clamped=bool(clamped),
    )
