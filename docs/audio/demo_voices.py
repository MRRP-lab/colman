"""Render every US and British Kokoro voice saying its own name into one mp3.

Usage (from the repository root):
    pixi run python docs/audio/demo_voices.py docs/audio/kokoro_voices.mp3

Needs the Kokoro dependencies from pixi.toml and ffmpeg. Model weights download on
first run. The voice list is the Kokoro-82M voice packs for American ("a") and
British ("b") English; set them on the TTS node with -p voice:=NAME -p lang_code:=a|b.
"""

import subprocess
import sys

import numpy as np
import soundfile as sf
from kokoro import KPipeline

SAMPLE_RATE = 24000
GAP_S = 0.7

# lang_code -> label -> voice ids
VOICES = {
    "a": {
        "American female": [
            "af_alloy",
            "af_aoede",
            "af_bella",
            "af_heart",
            "af_jessica",
            "af_kore",
            "af_nicole",
            "af_nova",
            "af_river",
            "af_sarah",
            "af_sky",
        ],
        "American male": [
            "am_adam",
            "am_echo",
            "am_eric",
            "am_fenrir",
            "am_liam",
            "am_michael",
            "am_onyx",
            "am_puck",
            "am_santa",
        ],
    },
    "b": {
        "British female": ["bf_alice", "bf_emma", "bf_isabella", "bf_lily"],
        "British male": ["bm_daniel", "bm_fable", "bm_george", "bm_lewis"],
    },
}


def main():
    out = sys.argv[1]
    gap = np.zeros(int(SAMPLE_RATE * GAP_S), dtype=np.float32)
    chunks = []
    for lang, groups in VOICES.items():
        pipe = KPipeline(lang_code=lang, repo_id="hexgrad/Kokoro-82M")
        for label, voices in groups.items():
            article = "an" if label[0] in "AEIOU" else "a"
            for voice in voices:
                name = voice.split("_", 1)[1].capitalize()
                text = f"Hello, I am {name}. I am {article} {label} voice."
                for _, _, audio in pipe(text, voice=voice):
                    chunks.append(np.asarray(audio, dtype=np.float32))
                chunks.append(gap)
                print("done", voice, flush=True)
    wav = out + ".wav"
    sf.write(wav, np.concatenate(chunks), SAMPLE_RATE)
    subprocess.run(
        ["ffmpeg", "-y", "-loglevel", "error", "-i", wav, "-b:a", "128k", out],
        check=True,
    )


if __name__ == "__main__":
    main()
