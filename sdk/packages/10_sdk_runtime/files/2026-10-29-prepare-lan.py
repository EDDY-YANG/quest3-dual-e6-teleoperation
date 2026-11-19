"""Prepare a local short-lived TLS certificate; never install a root CA."""

import argparse
import ipaddress
import json
from datetime import datetime, timedelta, timezone
from pathlib import Path

from cryptography import x509
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.x509.oid import NameOID

parser = argparse.ArgumentParser()
parser.add_argument("--ip", required=True)
parser.add_argument("--prefix", type=int, default=24)
args = parser.parse_args()
address = ipaddress.IPv4Address(args.ip)
if not address.is_private or address.is_loopback:
    raise ValueError("Use the current private WLAN IPv4 address")
root = Path(__file__).resolve().parent
runtime = root / "runtime"
runtime.mkdir(exist_ok=True)
key = ec.generate_private_key(ec.SECP256R1())
now = datetime.now(timezone.utc)
name = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, "Quest Input Local Development")])
cert = (
    x509.CertificateBuilder()
    .subject_name(name)
    .issuer_name(name)
    .public_key(key.public_key())
    .serial_number(x509.random_serial_number())
    .not_valid_before(now - timedelta(minutes=5))
    .not_valid_after(now + timedelta(days=7))
    .add_extension(x509.SubjectAlternativeName([x509.IPAddress(address)]), critical=False)
    .add_extension(x509.BasicConstraints(ca=False, path_length=None), critical=True)
    .sign(key, hashes.SHA256())
)
(runtime / "lan-cert.pem").write_bytes(cert.public_bytes(serialization.Encoding.PEM))
(runtime / "lan-key.pem").write_bytes(
    key.private_bytes(
        serialization.Encoding.PEM, serialization.PrivateFormat.PKCS8, serialization.NoEncryption()
    )
)
config = {
    "bind_ip": str(address),
    "subnet": str(ipaddress.IPv4Network(f"{address}/{args.prefix}", strict=False)),
    "port": 8443,
    "certificate_expires_utc": (now + timedelta(days=7)).isoformat(),
    "certificate_sha256": cert.fingerprint(hashes.SHA256()).hex(),
}
(runtime / "lan-config.json").write_text(json.dumps(config, indent=2), encoding="utf-8")
print(json.dumps(config, indent=2))
