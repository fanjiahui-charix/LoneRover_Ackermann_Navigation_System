#!/usr/bin/env python3
"""Create the accelerated showcase from the unmodified early recording.

Requires Python 3.9+ and FFmpeg (with libx264 and palettegen/paletteuse).
Optional: install imageio-ffmpeg in a virtual environment to supply FFmpeg.

Example:
    python tools/media/prepare_demo.py --source /path/to/07ed818a0b7dcf64463b76ff97776233.mp4

The source is read only. No footage is generated, reordered, or cropped. The
full video timeline is played at 1.90x; the GIF is a seven-second excerpt of
that accelerated video. The original audio is omitted from the showcase.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path


SOURCE_SHA256 = "1e75a83f1aa917ceca4cc9b127facfd75fd0a6707255b45d31cf01c22c31f06d"
SPEED = 1.9
PREVIEW_START = 20.0  # 38.0 s on the source timeline: approach and near-side turn.
PREVIEW_DURATION = 7.0
POSTER_SOURCE_TIME = 45.0
REPO_ROOT = Path(__file__).resolve().parents[2]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def run(command: list[str]) -> subprocess.CompletedProcess:
    result = subprocess.run(command, capture_output=True, text=True, encoding="utf-8", errors="replace")
    if result.returncode:
        raise RuntimeError(f"Command failed ({result.returncode}): {command!r}\n{result.stderr}")
    return result


def find_ffmpeg(explicit: str | None) -> str:
    if explicit:
        return str(Path(explicit).resolve())
    found = shutil.which("ffmpeg")
    if found:
        return found
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except ImportError as exc:
        raise RuntimeError("Supply --ffmpeg or install FFmpeg / imageio-ffmpeg in your environment.") from exc


def probe_video(ffmpeg: str, path: Path) -> dict:
    # Full decode both checks corruption and provides the decoded frame count.
    result = run([
        ffmpeg, "-hide_banner", "-i", str(path), "-map", "0:v:0", "-an",
        "-f", "null", "-", "-progress", "pipe:1", "-nostats",
    ])
    duration_match = re.search(r"Duration: (\d+):(\d+):(\d+(?:\.\d+)?)", result.stderr)
    stream_match = re.search(
        r"Stream #0:\d+[^\n]*Video: ([^ ,]+)[^\n]*?, (\d{2,5})x(\d{2,5})[^\n]*?, ([\d.]+) fps",
        result.stderr,
    )
    frame_matches = re.findall(r"^frame=(\d+)$", result.stdout, re.MULTILINE)
    time_matches = re.findall(r"^out_time_us=(\d+)$", result.stdout, re.MULTILINE)
    if not duration_match or not stream_match or not frame_matches or not time_matches:
        raise RuntimeError(f"Could not inspect video metadata for {path}:\n{result.stderr}")
    hours, minutes, seconds = duration_match.groups()
    codec, width, height, fps = stream_match.groups()
    frames, fps = int(frame_matches[-1]), float(fps)
    return {
        "container_duration_seconds": int(hours) * 3600 + int(minutes) * 60 + float(seconds),
        "decoded_video_frames": frames,
        # GIF timestamps have centisecond rounding and variable frame delays;
        # the container's reported fps is not a reliable duration denominator.
        "video_duration_seconds": int(time_matches[-1]) / 1_000_000,
        "width": int(width),
        "height": int(height),
        "reported_fps": fps,
        "codec": codec,
        "has_audio": bool(re.search(r"Stream #0:\d+[^\n]*Audio:", result.stderr)),
        "decode_succeeded": True,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, default=REPO_ROOT / "docs" / "media")
    parser.add_argument("--ffmpeg", help="Path to an FFmpeg executable")
    args = parser.parse_args()
    source, output = args.source.resolve(), args.output_dir.resolve()
    ffmpeg = find_ffmpeg(args.ffmpeg)
    if sha256(source) != SOURCE_SHA256:
        raise RuntimeError("Source SHA-256 differs from the reviewed early recording.")
    source_info = probe_video(ffmpeg, source)
    output.mkdir(parents=True, exist_ok=True)
    video = output / "early-prototype-demo.mp4"
    gif = output / "early-prototype-preview.gif"
    poster = output / "early-prototype-poster.jpg"
    provenance = output / "early-prototype-provenance.json"
    if source in [video, gif, poster, provenance]:
        raise RuntimeError("The output paths must not overwrite the source.")

    video_filter = f"setpts=(PTS-STARTPTS)/{SPEED},fps=30"
    video_command = [
        ffmpeg, "-hide_banner", "-loglevel", "error", "-y", "-i", str(source),
        "-map", "0:v:0", "-vf", video_filter, "-an", "-c:v", "libx264",
        "-preset", "slow", "-crf", "24", "-pix_fmt", "yuv420p",
        "-movflags", "+faststart", "-map_metadata", "-1",
        "-metadata", "title=Navigation demonstration (1.90x playback)",
        "-metadata", "comment=Full source video timeline accelerated 1.90x; silent.",
        str(video),
    ]
    run(video_command)
    gif_filter = (
        "fps=8,scale=544:-1:flags=lanczos,split[frames][colors];"
        "[colors]palettegen=max_colors=96:stats_mode=diff[palette];"
        "[frames][palette]paletteuse=dither=bayer:bayer_scale=5:diff_mode=rectangle"
    )
    run([
        ffmpeg, "-hide_banner", "-loglevel", "error", "-y",
        "-ss", str(PREVIEW_START), "-i", str(video), "-t", str(PREVIEW_DURATION),
        "-filter_complex", gif_filter, "-an", "-loop", "0", "-final_delay", "12", str(gif),
    ])
    run([
        ffmpeg, "-hide_banner", "-loglevel", "error", "-y",
        "-ss", str(POSTER_SOURCE_TIME / SPEED), "-i", str(video),
        "-frames:v", "1", "-q:v", "2", str(poster),
    ])
    video_info = probe_video(ffmpeg, video)
    gif_info = probe_video(ffmpeg, gif)
    if source_info["has_audio"] and video_info["has_audio"]:
        raise RuntimeError("The showcase unexpectedly retained an audio stream.")
    if not 30 <= video_info["video_duration_seconds"] <= 40:
        raise RuntimeError("Showcase duration is outside the requested 30-40 s range.")
    if sum(path.stat().st_size for path in (video, gif, poster)) >= 12_000_000:
        raise RuntimeError("Generated media exceed the 12 MB repository budget.")
    if sha256(source) != SOURCE_SHA256:
        raise RuntimeError("Source changed while generating showcase media.")
    version = run([ffmpeg, "-version"]).stdout.splitlines()[0]
    manifest = {
        "schema_version": 1,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "stage": "early recording",
        "source": {
            "filename": source.name,
            "sha256": SOURCE_SHA256,
            "bytes": source.stat().st_size,
            **source_info,
            "preserved_unmodified": True,
            "stored_in_repository": False,
        },
        "edit": {
            "playback_speed_factor": SPEED,
            "persistent_label": None,
            "timeline": "Entire source video timeline in original order; accelerated, without cropping or scene cuts.",
            "audio": "Omitted for the silent showcase and GIF; preserved in the external original.",
            "frame_rate_conversion": "30 fps output; temporal frames are dropped by the fps filter during acceleration.",
        },
        "outputs": {
            "video": {"filename": video.name, "bytes": video.stat().st_size, "sha256": sha256(video), **video_info},
            "gif": {
                "filename": gif.name, "bytes": gif.stat().st_size, "sha256": sha256(gif),
                **gif_info,
                "showcase_start_seconds": PREVIEW_START,
                "source_start_seconds": PREVIEW_START * SPEED,
                "source_span_seconds": PREVIEW_DURATION * SPEED,
                "requested_fps": 8,
                "loop": "infinite",
            },
            "poster": {
                "filename": poster.name, "bytes": poster.stat().st_size, "sha256": sha256(poster),
                "source_time_seconds_approx": POSTER_SOURCE_TIME,
                "width": video_info["width"], "height": video_info["height"],
            },
        },
        "reproduction": {
            "script": "tools/media/prepare_demo.py",
            "command": "python tools/media/prepare_demo.py --source /path/to/07ed818a0b7dcf64463b76ff97776233.mp4",
            "ffmpeg": version,
            "video_filter": video_filter,
            "gif_filter": gif_filter,
            "video_encoding": {"encoder": "libx264", "preset": "slow", "crf": 24, "pixel_format": "yuv420p", "faststart": True},
        },
    }
    provenance.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"source": source_info, "video": video_info, "gif": gif_info, "media_bytes": sum(p.stat().st_size for p in (video, gif, poster)), "provenance": str(provenance)}, indent=2))


if __name__ == "__main__":
    main()
