#!/usr/bin/env python3
"""
Fix corrupted WAV filenames by reading the correct timestamp from metadata.

The firmware bug caused filename timestamps to be corrupted (hours > 24, bad milliseconds)
because they used the buggy internal RTC. However, the metadata "Log Time" field uses
the external DS3231 RTC which was correct.

This script:
1. Reads the correct "Log Time" from the WAV file's META chunk
2. Renames the file to match the correct metadata timestamp

Usage:
    python fix_timestamps.py "G:\48kHz_16Bit_mono_Multi-day" --dry-run
    python fix_timestamps.py "G:\48kHz_16Bit_mono_Multi-day"
"""

import os
import re
import sys
import struct
from datetime import datetime
from pathlib import Path


def parse_filename(filename):
    """
    Parse the WAV filename and extract the prefix (everything before the timestamp).
    
    Format: S0000NY00_048K_S00_MAG000001-16b-CH01_YYYYMMDD_HHMMSS.mmmZ.wav
    """
    # Match everything up to and including the last underscore before the date
    # Then capture the date/time portion
    pattern = r'^(.+_)(\d{8})_(\d+)\.(\d+)Z\.wav$'
    match = re.match(pattern, filename)
    
    if match:
        return {
            'prefix': match.group(1),
            'date_str': match.group(2),
            'time_str': match.group(3),
            'ms_str': match.group(4),
            'full_match': True
        }
    
    return None


def read_metadata_log_time(file_path):
    """
    Read the "Log Time" field from the WAV file's META chunk.
    
    Returns: (success, log_time_str) or (False, None) if not found
    """
    try:
        with open(file_path, 'rb') as f:
            # Read RIFF header
            riff = f.read(4)
            if riff != b'RIFF':
                return (False, None)
            
            file_size = struct.unpack('<I', f.read(4))[0]
            wave = f.read(4)
            if wave != b'WAVE':
                return (False, None)
            
            # Find the META chunk
            while True:
                chunk_id = f.read(4)
                if not chunk_id or len(chunk_id) < 4:
                    break
                
                chunk_size = struct.unpack('<I', f.read(4))[0]
                
                if chunk_id == b'META':
                    meta_data = f.read(chunk_size)
                    
                    if len(meta_data) < 8:
                        return (False, None)
                    
                    # Parse header
                    version = struct.unpack('<I', meta_data[0:4])[0]
                    field_count = struct.unpack('<I', meta_data[4:8])[0]
                    
                    # Search for "Log Time" field
                    # Each record: 32 bytes name + 64 bytes value + 1 byte flag = 97 bytes
                    record_size = 97
                    offset = 8
                    
                    for i in range(field_count):
                        if offset + 96 > len(meta_data):
                            break
                        
                        field_name = meta_data[offset:offset+32].decode('utf-8', errors='ignore').rstrip('\x00')
                        field_value = meta_data[offset+32:offset+96].decode('utf-8', errors='ignore').rstrip('\x00')
                        
                        if field_name == 'Log Time':
                            return (True, field_value)
                        
                        offset += record_size
                    
                    return (False, None)
                else:
                    f.seek(chunk_size, 1)
                    if chunk_size % 2:
                        f.seek(1, 1)
            
            return (False, None)
            
    except Exception as e:
        print(f"    ERROR reading metadata: {e}")
        return (False, None)


def parse_log_time(log_time_str):
    """
    Parse the Log Time string from metadata.
    Format: YYYYMMDD_HHMMSSZ (e.g., 20000130_011026Z)
    
    Returns dict with date and time components or None if parsing fails.
    """
    pattern = r'^(\d{4})(\d{2})(\d{2})_(\d{2})(\d{2})(\d{2})Z?$'
    match = re.match(pattern, log_time_str.strip())
    
    if not match:
        return None
    
    return {
        'year': match.group(1),
        'month': match.group(2),
        'day': match.group(3),
        'hour': match.group(4),
        'minute': match.group(5),
        'second': match.group(6)
    }


def build_filename_from_metadata(prefix, log_time_parsed):
    """
    Build a new filename using the prefix and the correct timestamp from metadata.
    Uses 000 for milliseconds since metadata doesn't have sub-second precision.
    """
    date_str = f"{log_time_parsed['year']}{log_time_parsed['month']}{log_time_parsed['day']}"
    time_str = f"{log_time_parsed['hour']}{log_time_parsed['minute']}{log_time_parsed['second']}"
    
    return f"{prefix}{date_str}_{time_str}.000Z.wav"


def filename_matches_metadata(parsed_filename, log_time_parsed):
    """
    Check if the filename timestamp matches the metadata timestamp.
    We only compare date and HHMMSS, ignoring milliseconds.
    """
    if parsed_filename is None or log_time_parsed is None:
        return False
    
    # Extract HHMMSS from filename (may be corrupted with extra digits)
    time_str = parsed_filename['time_str']
    
    # For valid timestamps, time_str should be 6 digits (HHMMSS)
    if len(time_str) != 6:
        return False  # Corrupted
    
    expected_date = f"{log_time_parsed['year']}{log_time_parsed['month']}{log_time_parsed['day']}"
    expected_time = f"{log_time_parsed['hour']}{log_time_parsed['minute']}{log_time_parsed['second']}"
    
    return (parsed_filename['date_str'] == expected_date and 
            time_str == expected_time)


def process_folder(folder_path, dry_run=False):
    """Process all WAV files in the folder."""
    folder = Path(folder_path)
    
    if not folder.exists():
        print(f"Error: Folder '{folder_path}' does not exist")
        return
    
    wav_files = list(folder.glob('*.wav'))
    print(f"Found {len(wav_files)} WAV files in {folder_path}\n")
    
    fixed_count = 0
    skipped_ok = 0
    skipped_no_meta = 0
    error_count = 0
    
    for wav_file in sorted(wav_files):
        filename = wav_file.name
        
        # Parse the current filename
        parsed = parse_filename(filename)
        if parsed is None:
            print(f"  SKIP (can't parse filename): {filename}")
            skipped_no_meta += 1
            continue
        
        # Read the correct timestamp from metadata
        success, log_time_str = read_metadata_log_time(wav_file)
        if not success or log_time_str is None:
            print(f"  SKIP (no Log Time in metadata): {filename}")
            skipped_no_meta += 1
            continue
        
        # Parse the metadata timestamp
        log_time_parsed = parse_log_time(log_time_str)
        if log_time_parsed is None:
            print(f"  SKIP (can't parse Log Time '{log_time_str}'): {filename}")
            skipped_no_meta += 1
            continue
        
        # Check if filename already matches metadata
        if filename_matches_metadata(parsed, log_time_parsed):
            print(f"  OK: {filename}")
            skipped_ok += 1
            continue
        
        # Build the correct filename from metadata
        new_filename = build_filename_from_metadata(parsed['prefix'], log_time_parsed)
        
        print(f"  FIX: {filename}")
        print(f"    Metadata Log Time: {log_time_str}")
        print(f"    -> {new_filename}")
        
        if not dry_run:
            new_path = wav_file.parent / new_filename
            
            if new_path.exists() and new_path != wav_file:
                print(f"    ERROR: Target file already exists!")
                error_count += 1
                continue
            
            try:
                wav_file.rename(new_path)
                fixed_count += 1
            except Exception as e:
                print(f"    ERROR: {e}")
                error_count += 1
        else:
            fixed_count += 1
    
    print(f"\n{'='*50}")
    print(f"Summary:")
    print(f"  Fixed: {fixed_count}")
    print(f"  Already OK: {skipped_ok}")
    print(f"  Skipped (no metadata): {skipped_no_meta}")
    print(f"  Errors: {error_count}")
    
    if dry_run:
        print("\n(Dry run - no files were actually renamed)")


def main():
    if len(sys.argv) < 2:
        print("Fix WAV filenames using correct timestamps from metadata")
        print("\nThe metadata 'Log Time' field contains the correct timestamp")
        print("from the DS3231 RTC, while filenames may have corrupted timestamps")
        print("from the buggy internal RTC.")
        print("\nUsage: python fix_timestamps.py <folder_path> [--dry-run]")
        print("\nExample:")
        print('  python fix_timestamps.py "G:\\48kHz_16Bit_mono_Multi-day" --dry-run')
        print('  python fix_timestamps.py "G:\\48kHz_16Bit_mono_Multi-day"')
        sys.exit(1)
    
    folder_path = sys.argv[1]
    dry_run = '--dry-run' in sys.argv
    
    if dry_run:
        print("="*50)
        print("DRY RUN MODE (no files will be renamed)")
        print("="*50 + "\n")
    
    process_folder(folder_path, dry_run)


if __name__ == '__main__':
    main()
