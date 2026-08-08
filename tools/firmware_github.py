"""Fetch motor-driver firmware images from GitHub releases.

Companion to flash_motor_firmware.py — the "where does the .bin come from"
half.

A release is only usable if it carries a ``firmware.json`` manifest naming the
image and declaring its size and SHA-256; both are verified before the bytes
are handed to the flasher. Downloads are cached under
``temp_data/firmware/<product>/<tag>/`` and re-verified against the manifest on
every cache hit, so a second flash of the same version needs no network at all.

Stdlib only (urllib + hashlib) on purpose: the runtime venv has no `requests`,
and this must not add a dependency to a robot that may never be online.

Offline behaviour matters here. This Pi can run with `ufw default deny
outgoing` (see ~/Desktop/block_internet.md), which DROPS packets rather than
refusing them — a connect would hang, not fail. Every request therefore carries
an explicit timeout and every failure is translated into a FirmwareFetchError
that tells the operator to use a local .bin or a cached release.
"""

import hashlib
import json
import re
import sys
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from project_paths import TEMP_DATA_DIR

GITHUB_API = "https://api.github.com"
MANIFEST_NAME = "firmware.json"
DEFAULT_TIMEOUT_S = 10
FIRMWARE_CACHE_DIR = TEMP_DATA_DIR / "firmware"
UPLOAD_DIR = FIRMWARE_CACHE_DIR / "uploads"

FIRMWARE_PRODUCTS = {
    "stepfoc": {
        "label": "StepFOC stepper",
        "repo": "Source-Robotics/STEPFOC-stepper-controller",
    },
    "spectral-bldc": {
        "label": "Spectral Micro BLDC",
        "repo": "Source-Robotics/Spectral-Micro-BLDC-controller",
    },
}

_SHA256_RE = re.compile(r"^[0-9a-f]{64}$")

OFFLINE_HINT = (
    "Could not reach GitHub — this robot may have outgoing internet blocked. "
    "Use a local .bin (--file), or a release already downloaded into "
    f"{FIRMWARE_CACHE_DIR}."
)


class FirmwareFetchError(Exception):
    """Download, manifest, or verification failure. Carries operator advice."""


def get_product(product_id):
    try:
        return FIRMWARE_PRODUCTS[product_id]
    except KeyError:
        known = ", ".join(sorted(FIRMWARE_PRODUCTS))
        raise FirmwareFetchError(f"Unknown firmware product '{product_id}' (known: {known})")


# ---------------------------------------------------------------- HTTP helpers
def _fetch(url, timeout_s=DEFAULT_TIMEOUT_S, accept="application/vnd.github+json"):
    request = urllib.request.Request(url, headers={
        "Accept": accept,
        "User-Agent": "RCB-Runtime-firmware-flasher",
    })
    try:
        with urllib.request.urlopen(request, timeout=timeout_s) as response:
            return response.read()
    except urllib.error.HTTPError as error:
        if error.code == 404:
            raise FirmwareFetchError(f"Not found on GitHub: {url}")
        if error.code in (403, 429):
            raise FirmwareFetchError(
                "GitHub rate-limited this robot (unauthenticated API calls are "
                "capped per IP). Wait a few minutes or use a local .bin.")
        raise FirmwareFetchError(f"GitHub request failed ({error.code} {error.reason}).")
    except (urllib.error.URLError, TimeoutError, OSError) as error:
        raise FirmwareFetchError(f"{OFFLINE_HINT}  [{error}]")


def _fetch_json(url, timeout_s=DEFAULT_TIMEOUT_S):
    try:
        return json.loads(_fetch(url, timeout_s).decode("utf-8"))
    except (ValueError, UnicodeDecodeError) as error:
        raise FirmwareFetchError(f"GitHub returned invalid JSON from {url}: {error}")


def sha256_hex(data):
    return hashlib.sha256(data).hexdigest()


def _normalize_sha256(value):
    """Accept 'sha256:ABC...', mixed case, or None. Returns lowercase hex or None."""
    if not value:
        return None
    normalized = str(value).strip().lower()
    if normalized.startswith("sha256:"):
        normalized = normalized[len("sha256:"):]
    return normalized if _SHA256_RE.match(normalized) else None


# ---------------------------------------------------------------- release listing
def _summarize_release(release):
    assets = release.get("assets") or []
    names = [str(asset.get("name", "")) for asset in assets]
    return {
        "tag": release.get("tag_name") or "",
        "name": (release.get("name") or release.get("tag_name") or "").strip(),
        "published_at": release.get("published_at"),
        "prerelease": bool(release.get("prerelease")),
        "has_manifest": any(n.lower() == MANIFEST_NAME for n in names),
        "has_bin": any(n.lower().endswith(".bin") for n in names),
        "url": release.get("html_url"),
    }


def list_releases(product_id, timeout_s=DEFAULT_TIMEOUT_S):
    """Newest first. Drafts are dropped; entries lacking manifest+bin are kept
    but flagged so the GUI can grey them out with a reason."""
    repo = get_product(product_id)["repo"]
    releases = _fetch_json(f"{GITHUB_API}/repos/{repo}/releases?per_page=100", timeout_s)
    if not isinstance(releases, list):
        raise FirmwareFetchError(f"Unexpected release listing for {repo}")
    return [_summarize_release(r) for r in releases if not r.get("draft")]


def _find_asset(release, name):
    for asset in release.get("assets") or []:
        if str(asset.get("name", "")).lower() == name.lower():
            return asset
    return None


# ---------------------------------------------------------------- cache
def _safe_component(value, fallback):
    """One filesystem-safe path component from remote-controlled text.

    Dots must stay legal (version tags are 'v1.2.3'), which means a tag of
    '..' would otherwise survive intact and walk up a directory — so an
    all-dots result is replaced outright.
    """
    component = re.sub(r"[^A-Za-z0-9._-]", "_", str(value or fallback))
    if not component.strip("."):
        component = fallback
    return component


def _cache_dir(product_id, tag):
    return (FIRMWARE_CACHE_DIR
            / _safe_component(product_id, "unknown-product")
            / _safe_component(tag, "untagged"))


def _read_cached(product_id, tag, on_log):
    """Return a download dict if a verified copy is already on disk."""
    directory = _cache_dir(product_id, tag)
    manifest_path = directory / MANIFEST_NAME
    if not manifest_path.is_file():
        return None
    try:
        manifest = json.loads(manifest_path.read_text())
    except (OSError, ValueError):
        return None
    filename = manifest.get("firmware") or manifest.get("filename")
    if not filename:
        return None
    binary_path = directory / filename
    if not binary_path.is_file():
        return None
    data = binary_path.read_bytes()
    expected = _normalize_sha256(manifest.get("sha256"))
    actual = sha256_hex(data)
    if expected and expected != actual:
        on_log(f"Cached {tag} failed its checksum — re-downloading.")
        return None
    size = manifest.get("size")
    if isinstance(size, int) and size != len(data):
        on_log(f"Cached {tag} has the wrong size — re-downloading.")
        return None
    on_log(f"Using cached firmware {product_id} {tag} ({binary_path.name}, "
           f"{len(data)} bytes)")
    return {"path": binary_path, "manifest": manifest, "sha256": actual,
            "tag": tag, "product_id": product_id, "cached": True}


def list_cached_firmware():
    """Everything previously downloaded, for offline use. Newest-looking last."""
    entries = []
    if not FIRMWARE_CACHE_DIR.is_dir():
        return entries
    for manifest_path in sorted(FIRMWARE_CACHE_DIR.glob("*/*/" + MANIFEST_NAME)):
        try:
            manifest = json.loads(manifest_path.read_text())
        except (OSError, ValueError):
            continue
        filename = manifest.get("firmware") or manifest.get("filename")
        binary_path = manifest_path.parent / filename if filename else None
        if binary_path is None or not binary_path.is_file():
            continue
        entries.append({
            "product_id": manifest_path.parent.parent.name,
            "tag": manifest_path.parent.name,
            "path": binary_path,
            "manifest": manifest,
            "version": manifest.get("version"),
            "size": binary_path.stat().st_size,
        })
    return entries


# ---------------------------------------------------------------- download
def download_firmware(product_id, tag=None, timeout_s=DEFAULT_TIMEOUT_S, on_log=None):
    """Download (or reuse a cached copy of) one release's firmware image.

    tag=None means the repository's "latest" release. Returns
    {path, manifest, sha256, tag, product_id, cached}. Raises
    FirmwareFetchError with operator-facing advice on any failure.
    """
    log = on_log or (lambda line: None)
    product = get_product(product_id)
    repo = product["repo"]

    if tag:
        cached = _read_cached(product_id, tag, log)
        if cached is not None:
            return cached

    url = (f"{GITHUB_API}/repos/{repo}/releases/tags/{urllib.parse.quote(tag)}"
           if tag else f"{GITHUB_API}/repos/{repo}/releases/latest")
    log(f"Querying {repo} for {'release ' + tag if tag else 'the latest release'}...")
    release = _fetch_json(url, timeout_s)
    resolved_tag = release.get("tag_name") or tag or "untagged"

    # "latest" only resolves to a tag after the query, so the cache check for
    # it has to happen here rather than up front.
    if not tag:
        cached = _read_cached(product_id, resolved_tag, log)
        if cached is not None:
            return cached

    manifest_asset = _find_asset(release, MANIFEST_NAME)
    if manifest_asset is None:
        raise FirmwareFetchError(
            f"Release {resolved_tag} does not include {MANIFEST_NAME}, so its "
            "image cannot be verified. Download the .bin manually and use --file.")
    manifest_bytes = _fetch(manifest_asset["browser_download_url"], timeout_s,
                            accept="application/octet-stream")
    try:
        manifest = json.loads(manifest_bytes.decode("utf-8"))
    except (ValueError, UnicodeDecodeError):
        raise FirmwareFetchError(f"{MANIFEST_NAME} in {resolved_tag} is not valid JSON.")

    filename = manifest.get("firmware") or manifest.get("filename")
    if not filename:
        raise FirmwareFetchError(
            f"{MANIFEST_NAME} in {resolved_tag} does not name a firmware file.")
    binary_asset = _find_asset(release, filename)
    if binary_asset is None:
        raise FirmwareFetchError(f"Release {resolved_tag} is missing {filename}.")

    log(f"Downloading {filename} ({binary_asset.get('size', '?')} bytes)...")
    data = _fetch(binary_asset["browser_download_url"], timeout_s,
                  accept="application/octet-stream")

    expected_size = manifest.get("size")
    if isinstance(expected_size, int) and expected_size != len(data):
        raise FirmwareFetchError(
            f"Firmware size mismatch: manifest says {expected_size} bytes, "
            f"got {len(data)}.")
    expected_hash = _normalize_sha256(manifest.get("sha256"))
    actual_hash = sha256_hex(data)
    if expected_hash and expected_hash != actual_hash:
        raise FirmwareFetchError(
            "Firmware checksum mismatch — refusing to flash a corrupted image.")
    if not expected_hash:
        log(f"WARNING: {MANIFEST_NAME} declares no sha256; integrity unverified.")

    directory = _cache_dir(product_id, resolved_tag)
    directory.mkdir(parents=True, exist_ok=True)
    binary_path = directory / filename
    binary_path.write_bytes(data)
    (directory / MANIFEST_NAME).write_bytes(manifest_bytes)
    log(f"Cached to {binary_path}")

    return {"path": binary_path, "manifest": manifest, "sha256": actual_hash,
            "tag": resolved_tag, "product_id": product_id, "cached": False}


def save_uploaded_firmware(filename, data):
    """Persist a GUI upload so the same image can be re-flashed from the CLI."""
    UPLOAD_DIR.mkdir(parents=True, exist_ok=True)
    safe_name = re.sub(r"[^A-Za-z0-9._-]", "_", filename or "upload.bin")
    path = UPLOAD_DIR / safe_name
    path.write_bytes(data)
    return path


def manifest_app_base_address(manifest):
    """Manifest's declared link address, or None. Feeds the .bin validator."""
    value = manifest.get("app_base_address") or manifest.get("appBaseAddress")
    if value is None:
        return None
    try:
        return int(value, 0) if isinstance(value, str) else int(value)
    except (TypeError, ValueError):
        return None
