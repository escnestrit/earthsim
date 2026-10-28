"""Cell state machine (spec s.13). Illegal transitions raise; ERROR is retryable."""
from enum import Enum

class S(str, Enum):
    UNREQUESTED = "UNREQUESTED"; FETCHING = "FETCHING"; BASIC_READY = "BASIC_READY"
    REFINING = "REFINING"; DETAILED_READY = "DETAILED_READY"; CACHED = "CACHED"
    ERROR = "ERROR"; NO_SOURCE = "NO_SOURCE"

OK = {
    S.UNREQUESTED: {S.FETCHING, S.CACHED},
    S.FETCHING: {S.BASIC_READY, S.ERROR, S.NO_SOURCE, S.UNREQUESTED},
    S.BASIC_READY: {S.REFINING, S.UNREQUESTED},
    S.REFINING: {S.DETAILED_READY, S.BASIC_READY, S.ERROR, S.UNREQUESTED},
    S.DETAILED_READY: {S.CACHED, S.UNREQUESTED},
    S.CACHED: {S.UNREQUESTED, S.REFINING},
    S.ERROR: {S.FETCHING, S.UNREQUESTED},
    S.NO_SOURCE: {S.FETCHING, S.UNREQUESTED},
}

class Cell:
    def __init__(self, cid): self.cid, self.state, self.retries = cid, S.UNREQUESTED, 0
    def go(self, new):
        if new not in OK[self.state]: raise ValueError(f"{self.cid}: {self.state.value} -> {new.value} illegal")
        if new is S.ERROR: self.retries += 1
        self.state = new
