"""Compile, sign and install this checkout's Robot sketch on the lab car."""
import argparse
import hashlib
import http.client
import json
import os
from pathlib import Path
import secrets
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parent
BOARD = "arduino:renesas_uno:unor4wifi"
HOST = "192.168.4.1"
KEY = Path.home() / ".config/cog300-robot/update.pem"
CLI = shutil.which("arduino-cli") or "/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli"
SSL = shutil.which("openssl") or "openssl"


def identity(build, bootstrap):
    if not KEY.exists():
        if not bootstrap:
            raise ValueError("The signing key is missing. Run --bootstrap with the car connected by USB.")
        KEY.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(dir=KEY.parent) as new:
            subprocess.run([SSL, "genpkey", "-algorithm", "ED25519", "-out", new.name], check=True)
            os.link(new.name, KEY)
    public = subprocess.check_output([SSL, "pkey", "-in", KEY, "-pubout", "-outform", "DER"])
    if len(public) != 44 or public[:12].hex() != "302a300506032b6570032100":
        raise ValueError("Expected an Ed25519 signing key.")
    values = ",".join(str(value) for value in public[12:])
    return f'#pragma once\n#define ROBOT_BUILD_ID "{build}"\nstatic const uint8_t ROBOT_UPDATE_KEY[] = {{{values}}};\n'


def info():
    conn = http.client.HTTPConnection(HOST, timeout=4)
    try:
        conn.request("GET", "/firmware")
        response = conn.getresponse()
        if response.status != 200:
            raise ValueError("The car needs its OTA bootstrap firmware first.")
        return json.loads(response.read(512))
    finally:
        conn.close()


def upload(image, build, temp):
    challenge = info()["challenge"]
    if not challenge:
        raise ValueError("Stop the car and release its phone or USB control session first.")
    if len(challenge) != 32 or any(c not in "0123456789abcdef" for c in challenge):
        raise ValueError("Unexpected response from the car.")
    fields = f"{len(image):08x}{hashlib.sha256(image).hexdigest()}"
    message = temp / "manifest"
    message.write_bytes((challenge + fields).encode())
    signature = subprocess.check_output([SSL, "pkeyutl", "-sign", "-rawin", "-inkey", KEY, "-in", message])
    body = (fields + signature.hex()).encode()
    headers = (f"POST /firmware HTTP/1.1\r\nHost: {HOST}\r\nConnection: Upgrade\r\n"
               f"Upgrade: robot-firmware/1\r\nX-Robot-Control: 1\r\nContent-Length: {len(body)}\r\n\r\n")
    with socket.create_connection((HOST, 80), timeout=10) as sock, sock.makefile("rb") as reader:
        sock.sendall(headers.encode() + body)
        status = reader.readline(256)
        if not status.startswith(b"HTTP/1.1 101 "):
            raise ValueError(f"Update rejected ({status.strip()!r}). Check control ownership, key and bridge storage.")
        while True:
            line = reader.readline(512)
            if line == b"\r\n":
                break
            if not line or len(line) == 512:
                raise ValueError("Incomplete update handshake.")
        sock.settimeout(300)
        sock.sendall(image)
        if reader.readline(100).strip() != b"FLASHING":
            raise ValueError("The car did not confirm staged-image verification.")
    print("Image verified. Waiting for the car to restart. Keep power connected.", flush=True)
    deadline = time.monotonic() + 120
    while time.monotonic() < deadline:
        time.sleep(2)
        try:
            if info()["build"] == build:
                print(f"Car is running build {build}.")
                return
        except (OSError, ValueError, KeyError, http.client.HTTPException):
            pass
    raise ValueError("Reboot unconfirmed. Rejoin the car Wi-Fi and check /firmware for build " + build)


def usb_port():
    inventory = json.loads(subprocess.check_output([CLI, "board", "list", "--format", "json"]))
    candidates = [port["port"]["address"] for port in inventory.get("detected_ports", [])
                  if any(board.get("fqbn") == BOARD for board in port.get("matching_boards", []))]
    if len(candidates) != 1:
        raise ValueError("Connect one UNO R4 WiFi or pass its USB port explicitly with --bootstrap.")
    return candidates[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--bootstrap", metavar="USB_PORT", nargs="?", const="auto", help="install the OTA receiver once over USB")
    group.add_argument("--build-only", action="store_true", help="compile and check the image without uploading")
    args = parser.parse_args()
    if not (ROOT / "Robot/WirelessSecrets.h").is_file():
        raise ValueError("Configure Robot/WirelessSecrets.h with the car's access point password first.")
    port = usb_port() if args.bootstrap == "auto" else args.bootstrap
    build = secrets.token_hex(8)
    # A build-only first run can prepare the local signing identity without hardware.
    header = identity(build, args.bootstrap or args.build_only)
    with tempfile.TemporaryDirectory(prefix="cog300-update-") as directory:
        temp = Path(directory)
        sketch = temp / "Robot"
        shutil.copytree(ROOT / "Robot", sketch)
        (sketch / "FirmwareIdentity.h").write_text(header)
        generator = temp / "build_wifi_page.py"
        shutil.copy2(ROOT / generator.name, generator)
        subprocess.run([sys.executable, generator], check=True)
        out = temp / "out"
        subprocess.run([CLI, "compile", "-b", BOARD, "--build-path", temp / "build", "--output-dir", out, sketch], check=True)
        image = (out / "Robot.ino.bin").read_bytes()
        stack, reset = struct.unpack_from("<II", image)
        if not (256 <= len(image) <= 0x3c000 and 0x20000000 < stack <= 0x20008000 and
                reset & 1 and 0x4000 <= reset < 0x4000 + len(image) and build.encode() in image):
            raise ValueError("Image failed the UNO R4 WiFi application and OTA identity checks.")
        if args.bootstrap:
            subprocess.run([CLI, "upload", "-b", BOARD, "-p", port, "--input-dir", out, sketch], check=True)
            print(f"USB bootstrap uploaded, build {build}.")
        elif args.build_only:
            print(f"Checked {len(image)} bytes for build {build}.")
        else:
            upload(image, build, temp)


if __name__ == "__main__":
    try:
        main()
    except (ValueError, OSError, subprocess.CalledProcessError, http.client.HTTPException, KeyError) as error:
        sys.exit(f"Robot update: {error}")
