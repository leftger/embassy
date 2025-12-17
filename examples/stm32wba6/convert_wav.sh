#!/bin/bash

# Convert WAV files to raw unsigned 16-bit PCM for direct DMA streaming
# Output: 48kHz mono unsigned 16-bit little-endian (matches SAI DIV4 config)
# Usage: ./convert_wav.sh input.wav output.pcm

if [ $# -ne 2 ]; then
    echo "Usage: $0 input.wav output.pcm"
    echo "Converts WAV to raw unsigned 16-bit PCM (mono, 44.1kHz or 48kHz)"
    exit 1
fi

INPUT="$1"
OUTPUT="$2"

echo "Converting $INPUT to raw PCM format..."
echo "Output: $OUTPUT (unsigned 16-bit, mono)"

# Convert WAV to raw unsigned 16-bit PCM
# -ac 1: mono
# -ar 48000: resample to 48kHz (matches SAI DIV4 config)
# -f u16le: unsigned 16-bit little-endian
# -acodec pcm_u16le: unsigned 16-bit PCM codec
ffmpeg -i "$INPUT" -ac 1 -ar 48000 -f u16le -acodec pcm_u16le "$OUTPUT"

if [ $? -eq 0 ]; then
    echo "✅ Conversion successful!"
    echo "File size: $(stat -f%z "$OUTPUT") bytes"
    echo "Ready for direct DMA streaming (no processing overhead!)"
else
    echo "❌ Conversion failed!"
    exit 1
fi
