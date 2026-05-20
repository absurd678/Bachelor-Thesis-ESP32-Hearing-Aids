from pathlib import Path

try:
    Import("env")
    PROJECT_DIR = Path(env.subst("$PROJECT_DIR"))
except NameError:
    PROJECT_DIR = Path(__file__).resolve().parents[1]
HUFFMAN_HEADER = (
    PROJECT_DIR
    / "managed_components"
    / "espressif__esp-tflite-micro"
    / "tensorflow"
    / "lite"
    / "micro"
    / "kernels"
    / "decode_state_huffman.h"
)

REPLACEMENTS = {
    "0x8000'0000": "0x80000000",
    "0x7800'0000": "0x78000000",
    "0x07FF'FFFF": "0x07FFFFFF",
}


if HUFFMAN_HEADER.exists():
    text = HUFFMAN_HEADER.read_text(encoding="utf-8")
    patched = text
    for old, new in REPLACEMENTS.items():
        patched = patched.replace(old, new)

    if patched != text:
        HUFFMAN_HEADER.write_text(patched, encoding="utf-8", newline="\n")
        print("Patched esp-tflite-micro decode_state_huffman.h for C++11 builds")
