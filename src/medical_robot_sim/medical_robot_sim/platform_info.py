from __future__ import annotations

import re
from dataclasses import dataclass


@dataclass(frozen=True)
class JetsonRelease:
    raw: str
    r: str | None
    rev: str | None


_NV_TEGRA_R_RE = re.compile(r"\bR(?P<r>\d+)\b")
_NV_TEGRA_REV_RE = re.compile(r"\bREVISION:\s*(?P<rev>[0-9.]+)")


def parse_nv_tegra_release(text: str) -> JetsonRelease:
    """Parse /etc/nv_tegra_release content.

    The file format varies across L4T releases; we only extract stable tokens.

    Args:
        text: Full file text (possibly multi-line).

    Returns:
        JetsonRelease with best-effort extracted fields.
    """

    raw = (text or "").strip()
    if not raw:
        return JetsonRelease(raw="", r=None, rev=None)

    first_line = raw.splitlines()[0].strip()

    r_match = _NV_TEGRA_R_RE.search(first_line)
    rev_match = _NV_TEGRA_REV_RE.search(first_line)

    return JetsonRelease(
        raw=first_line,
        r=r_match.group("r") if r_match else None,
        rev=rev_match.group("rev") if rev_match else None,
    )
