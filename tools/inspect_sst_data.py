#!/usr/bin/env python3
import argparse
import struct
import os

# Chunk type names from firmware/src/fw/sst.h
CHUNK_TYPES = {
    0x00: "RATES",
    0x01: "TRAVEL",
    0x02: "MARKER",
    0x03: "IMU",
    0x04: "IMU_META",
    0x05: "GPS",
    0x06: "TEMPERATURE",
}

# Record sizes in bytes (from sst.h)
RECORD_SIZES = {
    0x00: 3,   # samplerate_record: uint8_t type + uint16_t rate (packed)
    0x01: 4,   # travel_record: 2 * uint16_t
    0x02: 0,   # marker: zero-length payload
    0x03: 12,  # imu_record: 6 * int16_t
    0x04: 9,   # imu_meta_record: uint8_t + float + float (packed)
    0x05: 46,  # gps_record: see sst.h for full layout
    0x06: 13,  # temperature_record: int64_t timestamp + uint8_t location + float celsius
}


def type_label(chunk_type):
    return CHUNK_TYPES.get(chunk_type, f"UNKNOWN(0x{chunk_type:02X})")


def hexdump(data, prefix="    ", width=16):
    lines = []
    for i in range(0, len(data), width):
        row = data[i:i + width]
        hex_part = " ".join(f"{b:02x}" for b in row)
        ascii_part = "".join(chr(b) if 32 <= b < 127 else "." for b in row)
        lines.append(f"{prefix}{i:04x}  {hex_part:<{width * 3}}  {ascii_part}")
    return "\n".join(lines)


def parse_sst(path):
    file_size = os.path.getsize(path)
    print(f"File: {path}")
    print(f"Size: {file_size} bytes")
    print()

    with open(path, "rb") as f:
        # Header: Magic(3), Version(1), Padding(4), Timestamp(8)
        header_fmt = "<3sB4xq"
        header_size = struct.calcsize(header_fmt)
        data = f.read(header_size)
        if len(data) != header_size:
            print(f"Error: read {len(data)} of {header_size} header bytes")
            return None

        magic, version, timestamp = struct.unpack(header_fmt, data)
        if magic != b"SST":
            print(f"Invalid magic: {magic!r} (expected b'SST')")
            print("  Header bytes:")
            print(hexdump(data))
            return None

        print(f"SST Version: {version}")
        print(f"Timestamp: {timestamp}")
        print()

        # Track chunk counts, record counts, and sample rates
        chunk_counts = {}
        record_counts = {}
        sample_rates = {}
        warnings = []

        chunk_header_fmt = "<BH"  # Type(1), Length(2)
        chunk_index = 0
        last_good_offset = f.tell()
        last_good_type = None
        stopped_reason = None

        while True:
            chunk_offset = f.tell()
            chunk_header_data = f.read(3)
            if not chunk_header_data:
                stopped_reason = "clean EOF"
                break

            if len(chunk_header_data) < 3:
                print("Malformed: incomplete chunk header")
                print(f"  Chunk index:       #{chunk_index}")
                print(f"  Offset:            {chunk_offset} (0x{chunk_offset:X})")
                print(f"  Bytes read:        {len(chunk_header_data)} of 3")
                print(f"  Raw bytes:         {chunk_header_data.hex(' ')}")
                print(f"  Bytes remaining:   {file_size - chunk_offset}")
                print(f"  Last good chunk:   #{chunk_index - 1} ({type_label(last_good_type)}) "
                      f"ending at offset {last_good_offset} (0x{last_good_offset:X})")
                stopped_reason = "incomplete chunk header"
                break

            chunk_type, chunk_len = struct.unpack(chunk_header_fmt, chunk_header_data)
            body_offset = f.tell()
            chunk_body = f.read(chunk_len)

            if len(chunk_body) != chunk_len:
                print("Malformed: incomplete chunk body")
                print(f"  Chunk index:       #{chunk_index}")
                print(f"  Chunk offset:      {chunk_offset} (0x{chunk_offset:X})")
                print(f"  Chunk type:        {type_label(chunk_type)}")
                print(f"  Declared length:   {chunk_len} bytes")
                print(f"  Body offset:       {body_offset} (0x{body_offset:X})")
                print(f"  Body bytes read:   {len(chunk_body)}")
                print(f"  Body bytes short:  {chunk_len - len(chunk_body)}")
                print(f"  File bytes from chunk start: {file_size - chunk_offset}")
                print(f"  Last good chunk:   #{chunk_index - 1} ({type_label(last_good_type)}) "
                      f"ending at offset {last_good_offset} (0x{last_good_offset:X})")
                print("  Chunk header bytes:")
                print(hexdump(chunk_header_data))
                if chunk_body:
                    shown = chunk_body[:128]
                    print(f"  Partial body (showing {len(shown)} of {len(chunk_body)} bytes):")
                    print(hexdump(shown))
                stopped_reason = "incomplete chunk body"
                break

            # Validate chunk type / length
            record_size = RECORD_SIZES.get(chunk_type)
            if record_size is None:
                warnings.append(
                    f"#{chunk_index} @ 0x{chunk_offset:X}: unknown chunk type 0x{chunk_type:02X}, "
                    f"length {chunk_len}"
                )
            elif record_size == 0:
                if chunk_len != 0:
                    warnings.append(
                        f"#{chunk_index} @ 0x{chunk_offset:X}: {type_label(chunk_type)} expected "
                        f"zero-length body, got {chunk_len}"
                    )
            else:
                if chunk_len == 0:
                    warnings.append(
                        f"#{chunk_index} @ 0x{chunk_offset:X}: {type_label(chunk_type)} empty "
                        f"chunk (no records)"
                    )
                elif chunk_len % record_size != 0:
                    warnings.append(
                        f"#{chunk_index} @ 0x{chunk_offset:X}: {type_label(chunk_type)} length "
                        f"{chunk_len} not a multiple of record size {record_size} "
                        f"(remainder {chunk_len % record_size})"
                    )

            # Count this chunk type
            chunk_counts[chunk_type] = chunk_counts.get(chunk_type, 0) + 1

            # Count records within chunk
            if record_size is not None:
                if record_size == 0:
                    # Marker chunks count as 1 record each
                    num_records = 1
                elif chunk_len > 0:
                    num_records = chunk_len // record_size
                else:
                    num_records = 0
                record_counts[chunk_type] = record_counts.get(chunk_type, 0) + num_records

            # Parse sample rates from RATES chunk
            if chunk_type == 0x00:  # RATES
                num_entries = chunk_len // 3
                for i in range(num_entries):
                    rtype, rrate = struct.unpack_from("<BH", chunk_body, i * 3)
                    type_name = CHUNK_TYPES.get(rtype, f"0x{rtype:02X}")
                    sample_rates[type_name] = rrate

            last_good_offset = f.tell()
            last_good_type = chunk_type
            chunk_index += 1

        # Print chunk summary with record counts
        print()
        print(f"Parsed {chunk_index} chunks ({stopped_reason})")
        print("Chunks found:")
        for chunk_type, count in sorted(chunk_counts.items()):
            label = type_label(chunk_type)
            records = record_counts.get(chunk_type, 0)
            print(f"  {label} (0x{chunk_type:02X}): {count} chunks, {records} records")

        # Print sample rates
        if sample_rates:
            print()
            print("Sample rates:")
            for type_name, rate in sample_rates.items():
                print(f"  {type_name}: {rate} Hz")

        if warnings:
            print()
            print(f"Warnings ({len(warnings)}):")
            for w in warnings:
                print(f"  {w}")

        # Report stopping position and any trailing bytes
        final_offset = f.tell()
        trailing = file_size - final_offset
        print()
        print(f"Stopped at offset {final_offset} (0x{final_offset:X}), {trailing} byte(s) unread")
        if trailing > 0:
            peek = f.read(min(trailing, 128))
            print(f"  Next bytes (showing {len(peek)} of {trailing}):")
            print(hexdump(peek))

        return True


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Inspect SST file structure")
    parser.add_argument("file", help="Path to binary SST file")
    args = parser.parse_args()
    if os.path.exists(args.file):
        parse_sst(args.file)
    else:
        print(f"File not found: {args.file}")
