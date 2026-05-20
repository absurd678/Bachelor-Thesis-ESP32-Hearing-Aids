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
FILL_KERNEL = (
    PROJECT_DIR
    / "managed_components"
    / "espressif__esp-tflite-micro"
    / "tensorflow"
    / "lite"
    / "micro"
    / "kernels"
    / "fill.cc"
)

REPLACEMENTS = {
    "0x8000'0000": "0x80000000",
    "0x7800'0000": "0x78000000",
    "0x07FF'FFFF": "0x07FFFFFF",
}

FILL_DYNAMIC_DIMS_CHECK = """  TF_LITE_ENSURE_MSG(context, IsConstantTensor(dims),
                     "Non-constant >dims< tensor is not supported");
  // The dims tensor must match the output tensor shape.
  // As a byproduct, ensures the dims tensor is of an integer type.
  TF_LITE_ENSURE_OK(context, EnsureEq(context, output->dims, dims));
"""

FILL_DYNAMIC_DIMS_PATCH = """  // ESP32 project patch: Keras RNN conversion can generate a Fill whose dims
  // tensor is computed from the fixed input shape. TFLM FillEval only uses the
  // already allocated output tensor shape, so allow this static-output case.
  (void)dims;
"""


if HUFFMAN_HEADER.exists():
    text = HUFFMAN_HEADER.read_text(encoding="utf-8")
    patched = text
    for old, new in REPLACEMENTS.items():
        patched = patched.replace(old, new)

    if patched != text:
        HUFFMAN_HEADER.write_text(patched, encoding="utf-8", newline="\n")
        print("Patched esp-tflite-micro decode_state_huffman.h for C++11 builds")

if FILL_KERNEL.exists():
    text = FILL_KERNEL.read_text(encoding="utf-8")
    patched = text.replace(FILL_DYNAMIC_DIMS_CHECK, FILL_DYNAMIC_DIMS_PATCH)

    if patched != text:
        FILL_KERNEL.write_text(patched, encoding="utf-8", newline="\n")
        print("Patched esp-tflite-micro Fill kernel for dynamic dims with static output")
