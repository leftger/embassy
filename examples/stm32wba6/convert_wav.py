#!/usr/bin/env python3

import sys
import wave
import struct
import os

def convert_wav_to_raw_pcm(input_file, output_file):
    """Convert WAV file to raw unsigned 16-bit PCM format for direct DMA streaming"""

    print(f"Converting {input_file} to raw PCM format...")

    try:
        # Open WAV file
        with wave.open(input_file, 'rb') as wav_file:
            # Get WAV parameters
            n_channels = wav_file.getnchannels()
            sample_width = wav_file.getsampwidth()
            frame_rate = wav_file.getframerate()
            n_frames = wav_file.getnframes()

            print(f"WAV info: {n_channels}ch, {sample_width*8}bit, {frame_rate}Hz, {n_frames} frames")

            # Validate format (16-bit only for now)
            if sample_width != 2:
                print(f"❌ Error: Only 16-bit WAV files supported (got {sample_width*8}bit)")
                return False

            if frame_rate != 48000:
                print(f"⚠️  Warning: Input is {frame_rate}Hz, but SAI is configured for 48000Hz")
                print(f"   Consider resampling with: ffmpeg -i input.wav -ar 48000 temp.wav")

            # Read all frames
            frames = wav_file.readframes(n_frames)

        # Convert to raw unsigned 16-bit PCM
        with open(output_file, 'wb') as pcm_file:
            # Process samples
            sample_count = 0
            for i in range(0, len(frames), sample_width):
                # Get signed 16-bit sample (little-endian)
                if n_channels == 1:
                    # Mono: single sample
                    signed_sample = struct.unpack('<h', frames[i:i+sample_width])[0]
                else:
                    # Stereo: take left channel only (convert to mono)
                    signed_sample = struct.unpack('<h', frames[i:i+sample_width])[0]
                    # Skip right channel
                    i += sample_width

                # Convert signed to unsigned: add 0x8000 (32768)
                unsigned_sample = (signed_sample + 0x8000) & 0xFFFF

                # Write as unsigned 16-bit little-endian
                pcm_file.write(struct.pack('<H', unsigned_sample))
                sample_count += 1

        print(f"✅ Conversion successful!")
        print(f"Output: {output_file}")
        print(f"Samples: {sample_count}")
        print(f"File size: {os.path.getsize(output_file)} bytes")
        print("Ready for direct DMA streaming (no processing overhead!)"

        return True

    except Exception as e:
        print(f"❌ Error: {e}")
        return False

def main():
    if len(sys.argv) != 3:
        print("Usage: python3 convert_wav.py input.wav output.pcm")
        print("Converts WAV to raw unsigned 16-bit PCM (mono)")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2]

    if not os.path.exists(input_file):
        print(f"❌ Error: Input file '{input_file}' not found")
        sys.exit(1)

    success = convert_wav_to_raw_pcm(input_file, output_file)
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()
