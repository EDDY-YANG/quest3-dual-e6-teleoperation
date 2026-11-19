"""Offline-only environment. Import this before any project modules."""
import sys
sys.dont_write_bytecode = True
from pathlib import Path
import hashlib
NETWORK_ATTEMPTS=[]
def deny_network(event,args):
    if event in ("socket.connect","socket.connect_ex","socket.sendto"):
        NETWORK_ATTEMPTS.append(event)
        raise RuntimeError("Offline prototype forbids network connections and robot I/O")
sys.addaudithook(deny_network)
ROOT=Path(__file__).resolve().parent
PROJECT=ROOT.parents[1]
sys.path.insert(0,str(PROJECT))
PROFILE=PROJECT/"profiles/offline_hybrid_v4.json"
RECORDINGS=None

def episode_directory(name):
    if not name or Path(name).name != name or name in (".", ".."):
        raise ValueError("Pass an immediate recording directory name")
    path=(RECORDINGS/name).resolve()
    path.relative_to(RECORDINGS)
    return path

def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()
