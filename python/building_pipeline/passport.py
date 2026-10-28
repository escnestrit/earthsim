"""Building passport. Source status and geometry confidence are separate fields (spec s.12)."""
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

class Status(str, Enum):
    SOURCE_VERIFIED = "SOURCE_VERIFIED"
    MODEL_FROM_OFFICIAL_DATA = "MODEL_FROM_OFFICIAL_DATA"
    RECONSTRUCTED_FROM_PHOTOS = "RECONSTRUCTED_FROM_PHOTOS"
    PARTIALLY_RECONSTRUCTED = "PARTIALLY_RECONSTRUCTED"
    PROCEDURAL_APPROXIMATION = "PROCEDURAL_APPROXIMATION"
    LOW_DETAIL_FALLBACK = "LOW_DETAIL_FALLBACK"
    SOURCE_UNAVAILABLE = "SOURCE_UNAVAILABLE"

class Origin(str, Enum):
    TAG = "tag"
    ESTIMATED = "estimated"
    DEFAULT = "default"

@dataclass
class Sourced:
    value: float
    origin: Origin

@dataclass
class BuildingPassport:
    id: str
    footprint: list
    height_m: Sourced
    levels: Optional[Sourced] = None
    category: str = "unknown"
    status: Status = Status.LOW_DETAIL_FALLBACK
    geometry_confidence: str = "none"   # ordinal: none < footprint_only < measured (not a probability)
    sources: list = field(default_factory=list)
    algo_version: str = "0"
    cache_key: Optional[str] = None
