"""Create VP9/Opus browser fallbacks without modifying the original MP4 files.

From the repository root: python tools/build_web_videos.py
Existing WebM files are validated and skipped; --force replaces them only after
a newly encoded temporary file passes decoding, duration and audio checks.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def find_ffmpeg(explicit: str | None) -> str:
    candidate = explicit or os.environ.get("FFMPEG_BINARY")
    if candidate:
        executable = shutil.which(candidate) or str(Path(candidate).expanduser())
        if not Path(executable).is_file():
            raise RuntimeError("The supplied FFmpeg executable does not exist.")
        return executable
    candidate = shutil.which("ffmpeg")
    if candidate:
        return candidate
    try:
        import imageio_ffmpeg
    except ImportError as exc:
        raise RuntimeError(
            "Provide --ffmpeg / FFMPEG_BINARY, or install tools/requirements-media.txt."
        ) from exc
    return imageio_ffmpeg.get_ffmpeg_exe()


def inspect_video(path: Path, *, decode: bool = False) -> dict:
    try:
        import av
    except ImportError as exc:
        raise RuntimeError("Install tools/requirements-media.txt to validate media.") from exc
    with av.open(str(path)) as container:
        if len(container.streams.video) != 1 or len(container.streams.audio) > 1:
            raise ValueError(f"Expected one video and at most one audio stream: {path.name}")
        video = container.streams.video[0]
        audio = container.streams.audio[0] if container.streams.audio else None
        video.codec_context.thread_count = 2
        if audio: audio.codec_context.thread_count = 1
        result = {
            "duration_s": container.duration / av.time_base,
            "width": video.codec_context.width,
            "height": video.codec_context.height,
            "video_codec": video.codec_context.name,
            "audio_codec": audio.codec_context.name if audio else None,
        }
        if decode:
            video_frames = audio_samples = 0
            audio_nonzero = False
            first_video_s = last_video_s = None
            for frame in container.decode(*([video,audio] if audio else [video])):
                if isinstance(frame, av.VideoFrame):
                    video_frames += 1
                    timestamp = float(frame.pts * frame.time_base)
                    if first_video_s is None:
                        first_video_s = timestamp
                    last_video_s = timestamp
                else:
                    audio_samples += frame.samples
                    # PCM sample bytes, excluding any decoder padding.
                    sample_bytes = frame.samples * frame.format.bytes
                    if not frame.format.is_planar:
                        sample_bytes *= len(frame.layout.channels)
                    audio_nonzero |= any(
                        any(bytes(plane)[:sample_bytes]) for plane in frame.planes
                    )
            if not video_frames or (audio and (not audio_samples or not audio_nonzero)):
                raise ValueError(f"Missing decoded video or nonzero audio: {path.name}")
            result.update(
                decoded_video_frames=video_frames,
                decoded_audio_samples=audio_samples,
                nonzero_audio=audio_nonzero,
                first_video_s=first_video_s,
                last_video_s=last_video_s,
            )
    return result


def validate_webm(path: Path, source: dict) -> dict:
    result = inspect_video(path, decode=True)
    if result["video_codec"] != "vp9" or result["audio_codec"] != ("opus" if source["audio_codec"] else None):
        raise ValueError(f"Expected VP9/Opus: {path.name}")
    if abs(result["duration_s"] - source["duration_s"]) > 0.2:
        raise ValueError(f"Duration changed by more than 0.2 seconds: {path.name}")
    expected_width = min(480,source["width"]) if source["height"] > source["width"] else source["width"]
    if result["width"] != expected_width or result["height"] % 2:
        raise ValueError(f"Unexpected dimensions: {path.name}")
    # All source clips here are about 30 fps. A full decode must reach the end.
    if result["last_video_s"] < source["duration_s"] - 0.2:
        raise ValueError(f"Decoded video ends early: {path.name}")
    result.update(bytes=path.stat().st_size, sha256=sha256(path))
    return result


def encode_one(entry: dict, ffmpeg: str, force: bool) -> dict:
    original = ROOT / entry["file"]
    destination = original.with_suffix(".webm")
    if sha256(original) != entry["sha256"]:
        raise ValueError(f"Original MP4 does not match provenance: {original.name}")
    source = inspect_video(original)
    if destination.exists() and not force:
        result = validate_webm(destination, source)
        status = "validated_existing"
    else:
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=f".{original.stem}-", suffix=".webm", dir=destination.parent
        )
        os.close(descriptor)
        temporary = Path(temporary_name)
        try:
            command = [
                ffmpeg, "-nostdin", "-hide_banner", "-loglevel", "error", "-y",
                "-i", str(original), "-map", "0:v:0",
                "-map_metadata", "-1",
            ]
            if source["height"] > source["width"] and source["width"] > 480:
                command += ["-vf", "scale=480:-2"]
            command += [
                "-c:v", "libvpx-vp9", "-crf", "33", "-b:v", "0",
                "-cpu-used", "4", "-row-mt", "1", "-threads", "2",
                "-pix_fmt", "yuv420p", "-fps_mode", "passthrough",
            ]
            command += ["-map","0:a:0","-c:a","libopus","-b:a","96k"] if source["audio_codec"] else ["-an"]
            command.append(str(temporary))
            subprocess.run(command, check=True, capture_output=True, text=True)
            result = validate_webm(temporary, source)
            if sha256(original) != entry["sha256"]:
                raise ValueError(f"Original MP4 changed during encoding: {original.name}")
            os.replace(temporary, destination)
            status = "encoded"
        finally:
            temporary.unlink(missing_ok=True)
    if sha256(original) != entry["sha256"]:
        raise ValueError(f"Original MP4 changed: {original.name}")
    return {
        "file": destination.relative_to(ROOT).as_posix(),
        "status": status,
        "original_sha256_verified": entry["sha256"],
        "source_duration_s": source["duration_s"],
        **result,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ffmpeg", help="FFmpeg executable (or use FFMPEG_BINARY).")
    parser.add_argument("--force", action="store_true", help="Re-encode existing WebM files.")
    args = parser.parse_args()
    entries = json.loads((ROOT / "docs/media_manifest.json").read_text(encoding="utf-8"))
    # Check every original before creating any output; provenance is never rewritten.
    for entry in entries:
        if sha256(ROOT / entry["file"]) != entry["sha256"]:
            raise ValueError(f"Original MP4 does not match provenance: {entry['file']}")
    ffmpeg = find_ffmpeg(args.ffmpeg)
    with ThreadPoolExecutor(max_workers=2) as pool:
        futures = [pool.submit(encode_one, entry, ffmpeg, args.force) for entry in entries]
        for future in as_completed(futures):
            print(json.dumps(future.result(), ensure_ascii=True), flush=True)


if __name__ == "__main__":
    main()
