#-------------------------------------------------------------------------------
# Name:        SD Card Data Plotter
# Purpose:     Analyze Magpie WAV files with custom metadata and interactive plotting
#
# Author:      ptc48
#
# Created:     23/10/2025
# Copyright:   (c) ptc48 2025
# Licence:     <your licence>
#
# Dependencies: pip install matplotlib pandas numpy tkinter mplcursors
#-------------------------------------------------------------------------------

import os
import re
import matplotlib.pyplot as plt
import pandas as pd
from datetime import datetime
import numpy as np
import tkinter as tk
from tkinter import filedialog, messagebox
from tkinter import ttk
import struct
import subprocess
import matplotlib.dates as mdates

# Configure matplotlib to prevent excessive tick warnings
plt.rcParams['axes.formatter.limits'] = [-7, 7]
from matplotlib.ticker import MaxNLocator

# Try to import mplcursors for hover functionality
try:
    import mplcursors
    HOVER_AVAILABLE = True
except ImportError:
    HOVER_AVAILABLE = False
    print("Warning: mplcursors not installed. Hover tooltips will not be available.")
    print("Install with: pip install mplcursors")

# Utility functions for centering windows
def center_window_on_screen(window, width, height):
    """Center a tkinter window on the screen"""
    window.update_idletasks()
    screen_width = window.winfo_screenwidth()
    screen_height = window.winfo_screenheight()
    x = (screen_width - width) // 2
    y = (screen_height - height) // 2
    window.geometry(f"{width}x{height}+{x}+{y}")

def center_matplotlib_figure(fig):
    """Center a matplotlib figure window on the screen"""
    try:
        mngr = fig.canvas.manager
        if hasattr(mngr, 'window'):
            if hasattr(mngr.window, 'wm_geometry'):
                # Tkinter backend
                import tkinter as tk
                root = tk.Tk()
                root.withdraw()
                screen_width = root.winfo_screenwidth()
                screen_height = root.winfo_screenheight()
                root.destroy()

                window_width = int(fig.get_figwidth() * 100)  # Approximate
                window_height = int(fig.get_figheight() * 100)  # Approximate
                x = (screen_width - window_width) // 2
                y = (screen_height - window_height) // 2

                mngr.window.wm_geometry(f"+{x}+{y}")
            elif hasattr(mngr.window, 'setGeometry'):
                # Qt backend
                import tkinter as tk
                root = tk.Tk()
                root.withdraw()
                screen_width = root.winfo_screenwidth()
                screen_height = root.winfo_screenheight()
                root.destroy()

                window_width = int(fig.get_figwidth() * fig.dpi)
                window_height = int(fig.get_figheight() * fig.dpi)
                x = (screen_width - window_width) // 2
                y = (screen_height - window_height) // 2

                mngr.window.setGeometry(x, y, window_width, window_height)
    except:
        pass  # If centering fails, just continue

def get_drive_name(path):
    """
    Get the volume label (drive name) for a given path on Windows

    Args:
        path (str): Path to check for drive name

    Returns:
        str: Drive name/label, or None if not found
    """
    try:
        # Get the drive letter from the path
        drive = os.path.splitdrive(path)[0]
        if not drive:
            return None

        # Use Windows vol command to get volume label
        result = subprocess.run(['vol', drive],
                              capture_output=True,
                              text=True,
                              shell=True)

        if result.returncode == 0:
            # Parse the output to extract volume label
            lines = result.stdout.strip().split('\n')
            for line in lines:
                if 'Volume in drive' in line and 'is' in line:
                    # Extract volume name from "Volume in drive C is Windows"
                    parts = line.split(' is ')
                    if len(parts) > 1:
                        volume_name = parts[1].strip()
                        return volume_name if volume_name else None

    except Exception as e:
        print(f"Warning: Could not get drive name for {path}: {e}")

    return None

def extract_wav_header_info(file_path, is_old_format=True):
    """
    Extract comprehensive WAV file header information including Magpie custom metadata

    Args:
        file_path (str): Path to the WAV file
        is_old_format (bool): True for old Magpie format (metadata in data chunk),
                              False for new format (metadata in META chunk)

    Returns:
        dict: Dictionary containing all available WAV metadata, or None if failed
    """
    try:
        with open(file_path, 'rb') as f:
            # Read WAV header
            # RIFF header
            riff = f.read(4)
            if riff != b'RIFF':
                return None

            file_size = struct.unpack('<I', f.read(4))[0]
            wave = f.read(4)
            if wave != b'WAVE':
                return None

            metadata = {
                'file_size': file_size,
                'filename': os.path.basename(file_path)
            }

            # Parse all chunks
            while True:
                try:
                    chunk_id = f.read(4)
                    if not chunk_id or len(chunk_id) < 4:
                        break

                    chunk_size = struct.unpack('<I', f.read(4))[0]

                    if chunk_id == b'fmt ':
                        # Parse format chunk
                        format_data = f.read(chunk_size)
                        if len(format_data) >= 16:
                            audio_format = struct.unpack('<H', format_data[0:2])[0]
                            channels = struct.unpack('<H', format_data[2:4])[0]
                            sample_rate = struct.unpack('<I', format_data[4:8])[0]
                            byte_rate = struct.unpack('<I', format_data[8:12])[0]
                            block_align = struct.unpack('<H', format_data[12:14])[0]
                            bit_depth = struct.unpack('<H', format_data[14:16])[0]

                            metadata.update({
                                'sample_rate': sample_rate,
                                'bit_depth': bit_depth,
                                'channels': channels,
                                'audio_format': audio_format,
                                'byte_rate': byte_rate,
                                'block_align': block_align
                            })

                    elif chunk_id == b'META':
                        # New format: META chunk with structured metadata
                        if not is_old_format:
                            # Each record: 1 byte flag + 32 bytes field name + 64 bytes value
                            meta_data = f.read(chunk_size)
                            meta_metadata = extract_meta_chunk(meta_data)
                            if meta_metadata:
                                metadata.update(meta_metadata)
                                print(f"    Found META chunk with {len(meta_metadata)} fields")
                        else:
                            # Skip META chunk for old format files
                            f.seek(chunk_size, 1)

                    elif chunk_id == b'data':
                        # Data chunk
                        metadata['data_size'] = chunk_size
                        current_pos = f.tell()

                        # Old format: look for embedded metadata in data chunk
                        if is_old_format:
                            magpie_metadata = extract_magpie_metadata(f, current_pos)
                            if magpie_metadata:
                                metadata.update(magpie_metadata)
                                print(f"    Found embedded metadata (old format) with {len(magpie_metadata)} fields")

                        # Skip the rest of the data chunk
                        f.seek(current_pos + chunk_size)

                    elif chunk_id == b'LIST':
                        # INFO chunk with metadata
                        list_data = f.read(chunk_size)
                        if len(list_data) >= 4 and list_data[:4] == b'INFO':
                            # Parse INFO subchunks
                            pos = 4
                            while pos < len(list_data) - 8:
                                try:
                                    subchunk_id = list_data[pos:pos+4]
                                    subchunk_size = struct.unpack('<I', list_data[pos+4:pos+8])[0]
                                    pos += 8

                                    if pos + subchunk_size <= len(list_data):
                                        subchunk_data = list_data[pos:pos+subchunk_size]
                                        # Convert to string, removing null terminators
                                        text_data = subchunk_data.decode('utf-8', errors='ignore').rstrip('\x00')

                                        # Map common INFO chunk IDs to readable names
                                        info_map = {
                                            b'IART': 'Artist',
                                            b'ICMT': 'Comment',
                                            b'ICOP': 'Copyright',
                                            b'ICRD': 'Creation Date',
                                            b'IENG': 'Engineer',
                                            b'IGNR': 'Genre',
                                            b'IKEY': 'Keywords',
                                            b'IMED': 'Medium',
                                            b'INAM': 'Title',
                                            b'IPRD': 'Product',
                                            b'ISBJ': 'Subject',
                                            b'ISFT': 'Software',
                                            b'ISRC': 'Source',
                                            b'ISRF': 'Source Form',
                                            b'ITCH': 'Technician'
                                        }

                                        field_name = info_map.get(subchunk_id, subchunk_id.decode('utf-8', errors='ignore'))
                                        if text_data.strip():
                                            metadata[field_name] = text_data

                                        pos += subchunk_size
                                        # Align to word boundary
                                        if subchunk_size % 2:
                                            pos += 1
                                    else:
                                        break
                                except:
                                    break
                        else:
                            f.seek(chunk_size, 1)

                    elif chunk_id == b'bext':
                        # Broadcast Wave Format extension
                        bext_data = f.read(min(chunk_size, 602))  # bext is fixed size
                        if len(bext_data) >= 602:
                            try:
                                description = bext_data[0:256].decode('utf-8', errors='ignore').rstrip('\x00')
                                originator = bext_data[256:288].decode('utf-8', errors='ignore').rstrip('\x00')
                                originator_ref = bext_data[288:320].decode('utf-8', errors='ignore').rstrip('\x00')
                                origination_date = bext_data[320:330].decode('utf-8', errors='ignore').rstrip('\x00')
                                origination_time = bext_data[330:338].decode('utf-8', errors='ignore').rstrip('\x00')

                                if description.strip():
                                    metadata['Description'] = description
                                if originator.strip():
                                    metadata['Originator'] = originator
                                if originator_ref.strip():
                                    metadata['Originator Reference'] = originator_ref
                                if origination_date.strip():
                                    metadata['Origination Date'] = origination_date
                                if origination_time.strip():
                                    metadata['Origination Time'] = origination_time
                            except:
                                pass
                        else:
                            f.seek(chunk_size, 1)

                    else:
                        # Skip unknown chunks
                        f.seek(chunk_size, 1)
                        # Align to word boundary
                        if chunk_size % 2:
                            f.seek(1, 1)

                except struct.error:
                    break
                except:
                    break

            # Calculate duration if we have the necessary info
            if 'sample_rate' in metadata and 'data_size' in metadata and 'bit_depth' in metadata and 'channels' in metadata:
                bytes_per_sample = (metadata['bit_depth'] // 8) * metadata['channels']
                if bytes_per_sample > 0:
                    duration_seconds = metadata['data_size'] / (metadata['sample_rate'] * bytes_per_sample)
                    metadata['duration_seconds'] = round(duration_seconds, 2)

            return metadata

    except Exception as e:
        print(f"Warning: Could not read WAV header for {file_path}: {e}")
        return None

def extract_meta_chunk(meta_data):
    """
    Extract metadata from the new META chunk format.
    
    Structure based on hex dump analysis:
    - 4 bytes: version/type (0x01 0x00 0x00 0x00)
    - 4 bytes: field count
    - Then for each field:
      - 32 bytes: field name (null-padded)
      - 64 bytes: field value (null-padded)
      - 1 byte: separator/flag
    
    Args:
        meta_data: Raw bytes from the META chunk
        
    Returns:
        dict: Dictionary containing extracted metadata fields
    """
    try:
        metadata = {}
        
        # Debug: print first 100 bytes
        print(f"    DEBUG META chunk size: {len(meta_data)} bytes")
        print(f"    DEBUG First 50 bytes: {meta_data[:50].hex()}")
        
        # Parse header
        if len(meta_data) < 8:
            print("    DEBUG: META chunk too small")
            return None
            
        version = struct.unpack('<I', meta_data[0:4])[0]
        field_count = struct.unpack('<I', meta_data[4:8])[0]
        print(f"    DEBUG: version={version}, field_count={field_count}")
        
        # Each record: 32 bytes name + 64 bytes value + 1 byte flag = 97 bytes
        record_size = 97
        offset = 8  # Start after header
        
        for i in range(field_count):
            if offset + 96 > len(meta_data):  # Need at least name + value
                break
                
            # Extract field name (32 bytes, null-terminated)
            field_name_bytes = meta_data[offset:offset + 32]
            field_name = field_name_bytes.decode('utf-8', errors='ignore').rstrip('\x00').strip()
            
            # Extract field value (64 bytes, null-terminated)
            field_value_bytes = meta_data[offset + 32:offset + 96]
            field_value = field_value_bytes.decode('utf-8', errors='ignore').rstrip('\x00').strip()
            
            if field_name:
                metadata[field_name] = field_value
                print(f"    META field: '{field_name}' = '{field_value}'")
            
            # Move to next record (32 + 64 + 1 = 97)
            offset += record_size
        
        return metadata if metadata else None
        
    except Exception as e:
        print(f"Warning: Could not parse META chunk: {e}")
        import traceback
        traceback.print_exc()
        return None


def extract_magpie_metadata(file_handle, data_start_pos):
    """
    Extract Magpie-specific custom metadata from the WAV data section (old format)
    Dynamically discovers and extracts any key-value pairs present in the metadata

    Args:
        file_handle: Open file handle positioned at data chunk
        data_start_pos: Position where data chunk starts

    Returns:
        dict: Dictionary containing all discovered Magpie metadata fields, or None if not found
    """
    try:
        # Read the first 8KB of the data section where Magpie stores metadata
        current_pos = file_handle.tell()
        metadata_section = file_handle.read(8192)  # Increased size to capture more metadata

        if len(metadata_section) < 100:
            print(f"    Debug: Metadata section too small ({len(metadata_section)} bytes)")
            return None

        magpie_metadata = {}
        
        # Debug: Show first 200 bytes as hex and ASCII for troubleshooting
        print(f"    Debug: First 200 bytes of data section:")
        preview = metadata_section[:200]
        ascii_preview = ''.join(chr(b) if 32 <= b <= 126 else '.' for b in preview)
        print(f"    ASCII: {ascii_preview}")

        # Extract all null-terminated strings from the metadata section
        strings = []
        current_string = ""

        for byte in metadata_section:
            if byte == 0:
                if len(current_string) > 0:
                    strings.append(current_string)
                    current_string = ""
            elif 32 <= byte <= 126:  # Printable ASCII
                current_string += chr(byte)
            else:
                # Non-printable character, end current string
                if len(current_string) > 0:
                    strings.append(current_string)
                    current_string = ""

        # Filter for meaningful strings
        meaningful_strings = [s.strip() for s in strings if len(s.strip()) > 1]
        
        # Debug: Show all meaningful strings found
        if meaningful_strings:
            print(f"    Debug: Found {len(meaningful_strings)} meaningful strings:")
            for s in meaningful_strings[:20]:  # Show first 20
                print(f"      - '{s}'")
        else:
            print(f"    Debug: No meaningful strings found in metadata section")

        # Dynamic field-value pairing algorithm
        i = 0
        while i < len(meaningful_strings):
            current_string = meaningful_strings[i]

            # Check if this string looks like a field name
            is_field_name = (
                # Contains parentheses (like "Temperature(C)" or "Voltage(V)")
                ('(' in current_string and ')' in current_string) or
                # Contains common field keywords
                any(keyword in current_string.lower() for keyword in [
                    'time', 'temperature', 'voltage', 'current', 'pressure', 'humidity',
                    'battery', 'power', 'signal', 'gain', 'level', 'status', 'mode',
                    'frequency', 'rate', 'count', 'duration', 'interval', 'threshold',
                    'calibration', 'offset', 'scale', 'factor', 'coefficient'
                ]) or
                # Ends with common field suffixes
                any(current_string.lower().endswith(suffix) for suffix in [
                    'time', 'date', 'id', 'name', 'type', 'mode', 'state', 'status'
                ]) or
                # Contains "Avg" or "Average"
                'avg' in current_string.lower() or 'average' in current_string.lower() or
                # Contains measurement-related terms
                any(term in current_string.lower() for term in [
                    'recording', 'stopping', 'starting', 'measurement', 'reading',
                    'sensor', 'device', 'system', 'config', 'setting'
                ])
            )

            if is_field_name and i + 1 < len(meaningful_strings):
                # Look for the next string as a potential value
                next_string = meaningful_strings[i + 1]

                # Check if the next string looks like a value
                is_value = (
                    # Numeric value (integer or float, possibly negative)
                    (next_string.replace('.', '').replace('-', '').replace('+', '').isdigit()) or
                    # Timestamp format (like "20251102_141614Z")
                    (len(next_string) >= 10 and ('_' in next_string or ':' in next_string or '-' in next_string)) or
                    # Boolean-like values
                    next_string.lower() in ['true', 'false', 'yes', 'no', 'on', 'off', 'enabled', 'disabled'] or
                    # Short alphanumeric values (like device IDs, modes, etc.)
                    (len(next_string) <= 20 and next_string.replace('_', '').replace('-', '').isalnum()) or
                    # Values with units (like "8.25V", "20.0C", etc.)
                    any(next_string.lower().endswith(unit) for unit in ['v', 'a', 'ma', 'c', 'f', 'hz', 'khz', 'mhz', 'db', '%', 'ppm'])
                )

                if is_value:
                    # Store the raw key-value pair
                    field_name = current_string
                    field_value = next_string

                    # Add debug info to see what's being extracted
                    print(f"    Found field: '{field_name}' = '{field_value}'")

                    magpie_metadata[field_name] = field_value

                    # Create formatted/enhanced versions for common field types
                    field_lower = field_name.lower()

                    # Handle timestamps
                    if ('time' in field_lower or 'date' in field_lower) and len(field_value) >= 15 and '_' in field_value:
                        try:
                            if 'Z' in field_value and len(field_value) >= 15:
                                date_part = field_value[:8]  # 20251102
                                time_part = field_value[9:15]  # 141614
                                formatted_time = f"{date_part[:4]}-{date_part[4:6]}-{date_part[6:8]} {time_part[:2]}:{time_part[2:4]}:{time_part[4:6]} UTC"
                                magpie_metadata[f"{field_name} (Formatted)"] = formatted_time
                        except:
                            pass

                    # Handle temperature values
                    elif 'temperature' in field_lower:
                        try:
                            if field_value.replace('.', '').replace('-', '').isdigit():
                                temp_val = float(field_value)
                                if '(c)' in field_lower or 'celsius' in field_lower:
                                    magpie_metadata[f"{field_name} (Formatted)"] = f"{temp_val}°C"
                                elif '(f)' in field_lower or 'fahrenheit' in field_lower:
                                    magpie_metadata[f"{field_name} (Formatted)"] = f"{temp_val}°F"
                                else:
                                    magpie_metadata[f"{field_name} (Formatted)"] = f"{temp_val}°C"  # Default to Celsius
                        except:
                            pass

                    # Handle voltage values
                    elif 'voltage' in field_lower:
                        try:
                            if field_value.replace('.', '').replace('-', '').isdigit():
                                volt_val = float(field_value)
                                magpie_metadata[f"{field_name} (Formatted)"] = f"{volt_val}V"
                        except:
                            pass

                    # Handle current values
                    elif 'current' in field_lower:
                        try:
                            if field_value.replace('.', '').replace('-', '').isdigit():
                                curr_val = float(field_value)
                                if '(ma)' in field_lower or 'milliamp' in field_lower:
                                    magpie_metadata[f"{field_name} (Formatted)"] = f"{curr_val}mA"
                                elif '(a)' in field_lower or 'amp' in field_lower:
                                    magpie_metadata[f"{field_name} (Formatted)"] = f"{curr_val}A"
                                else:
                                    magpie_metadata[f"{field_name} (Formatted)"] = f"{curr_val}mA"  # Default to mA
                        except:
                            pass

                    # Handle frequency values
                    elif 'frequency' in field_lower or 'freq' in field_lower:
                        try:
                            if field_value.replace('.', '').isdigit():
                                freq_val = float(field_value)
                                if '(khz)' in field_lower:
                                    magpie_metadata[f"{field_name} (Formatted)"] = f"{freq_val}kHz"
                                elif '(mhz)' in field_lower:
                                    magpie_metadata[f"{field_name} (Formatted)"] = f"{freq_val}MHz"
                                elif '(hz)' in field_lower:
                                    magpie_metadata[f"{field_name} (Formatted)"] = f"{freq_val}Hz"
                        except:
                            pass

                    # Skip the value string in the next iteration
                    i += 2
                    continue

            i += 1

        # Debug output
        if magpie_metadata:
            print(f"    Extracted {len(magpie_metadata)} Magpie metadata fields:")
            for key in sorted(magpie_metadata.keys()):
                if not key.endswith(" (Formatted)"):  # Only show raw fields in debug
                    print(f"      {key}: {magpie_metadata[key]}")
            return magpie_metadata

        return None

    except Exception as e:
        print(f"Warning: Could not extract Magpie metadata: {e}")
        return None

def check_wav_header_for_dma_overrun(file_path):
    """
    Check WAV file header for DMA overrun metadata

    Args:
        file_path (str): Path to the WAV file

    Returns:
        bool: True if DMA overrun metadata is found, False otherwise
    """
    try:
        with open(file_path, 'rb') as f:
            # Read the first 1024 bytes to check for metadata
            header_data = f.read(1024)

            # Convert to string for text search (ignore encoding errors)
            header_text = header_data.decode('utf-8', errors='ignore').lower()

            # Check for various forms of DMA overrun indicators
            dma_indicators = [
                'dma overrun',
                'dma_overrun',
                'dmaoverrun',
                'overrun',
                'buffer overrun',
                'data loss'
            ]

            for indicator in dma_indicators:
                if indicator in header_text:
                    return True

            # Also check in the raw bytes for binary metadata
            header_lower = header_data.lower()
            for indicator in dma_indicators:
                if indicator.encode('utf-8') in header_lower:
                    return True

        return False

    except Exception as e:
        print(f"Warning: Could not read header for {file_path}: {e}")
        return False

def select_folder():
    """
    Open a folder selection dialog and return the selected path

    Returns:
        str: Selected folder path, or None if cancelled
    """
    root = tk.Tk()
    root.withdraw()  # Hide the main window
    root.lift()      # Bring to front
    root.attributes('-topmost', True)  # Keep on top

    folder_path = filedialog.askdirectory(
        title="Select SD Card or Folder Containing WAV Files",
        initialdir=os.getcwd()
    )

    root.destroy()
    return folder_path if folder_path else None

def scan_wav_files(sd_card_path, progress_callback=None):
    """
    Scan SD card for WAV files matching the Magpie format and extract data

    Args:
        sd_card_path (str): Path to the SD card or directory to scan
        progress_callback (function): Optional callback for progress updates

    Returns:
        tuple: (datetime_list, filesize_list, filenames_list, dma_overrun_list, wav_metadata_list, wav_info) containing extracted data
    """

    # Patterns to match both old and new filename formats
    # Old format: Magpie00_YYYYMMDD_hhmmss[.fff]Z_48kHz_16_bit_1_channel.wav
    # New format: S0000NY00_048K_S00_MAG000001-16b-CH01_YYYYMMDD_hhmmss[.fff]Z.wav
    # Sample rate, bit depth, and channels are extracted from WAV header
    pattern_old = r'Magpie\d+_(\d{8})_(\d{6})(?:\.(\d{3}))?Z?_\d+kHz_\d+_bit_\d+_channel\.wav'
    pattern_new = r'.+_(\d{8})_(\d{6})(?:\.(\d{3}))?Z?\.wav'

    datetime_list = []
    filesize_list = []
    filenames_list = []
    dma_overrun_list = []
    wav_metadata_list = []  # Store metadata for each file
    wav_info = None  # Will store info from first valid WAV file

    # First pass: count total files for progress
    total_files = 0
    for root, dirs, files in os.walk(sd_card_path):
        total_files += len([f for f in files if f.lower().endswith('.wav')])

    processed_files = 0

    # Walk through all directories and subdirectories
    for root, dirs, files in os.walk(sd_card_path):
        for file in files:
            if file.lower().endswith('.wav'):
                processed_files += 1

                if progress_callback:
                    progress_callback(processed_files, total_files, file)

                # Try old pattern first (more specific), then new pattern
                is_old_format = False
                match = re.match(pattern_old, file, re.IGNORECASE)
                if match:
                    is_old_format = True
                else:
                    match = re.match(pattern_new, file, re.IGNORECASE)
                    is_old_format = False
                    
                if match:
                    try:
                        # Extract date and time from filename
                        date_str = match.group(1)  # YYYYMMDD
                        time_str = match.group(2)  # hhmmss
                        milliseconds_str = match.group(3)  # fff (optional)

                        # Handle invalid hours (24+) by rolling over to next day
                        hour = int(time_str[:2])
                        if hour >= 24:
                            # Calculate extra days and adjusted hour
                            extra_days = hour // 24
                            adjusted_hour = hour % 24
                            time_str = f"{adjusted_hour:02d}{time_str[2:]}"
                            # Parse base date and add extra days
                            base_date = datetime.strptime(date_str, "%Y%m%d")
                            from datetime import timedelta
                            adjusted_date = base_date + timedelta(days=extra_days)
                            date_str = adjusted_date.strftime("%Y%m%d")
                        
                        # Parse datetime (with or without milliseconds)
                        if milliseconds_str:
                            # Convert milliseconds (3 digits) to microseconds (6 digits) for %f
                            microseconds_str = milliseconds_str.ljust(6, '0')  # Pad with zeros
                            datetime_str = f"{date_str}_{time_str}.{microseconds_str}"
                            file_datetime = datetime.strptime(datetime_str, "%Y%m%d_%H%M%S.%f")
                        else:
                            datetime_str = f"{date_str}_{time_str}"
                            file_datetime = datetime.strptime(datetime_str, "%Y%m%d_%H%M%S")
                        
                        # Debug: Print first few parsed datetimes
                        if len(datetime_list) < 5:
                            format_type = "old" if is_old_format else "new"
                            print(f"Debug: Parsed {file} ({format_type} format) -> {file_datetime}")

                        # Get file size in bytes
                        file_path = os.path.join(root, file)
                        file_size = os.path.getsize(file_path)

                        # Extract WAV header info for each file
                        # Pass is_old_format to determine metadata extraction method
                        file_metadata = extract_wav_header_info(file_path, is_old_format)
                        if file_metadata is None:
                            file_metadata = {'filename': file}  # Fallback metadata

                        # Store info from first valid file for general display
                        if wav_info is None and file_metadata and 'sample_rate' in file_metadata:
                            wav_info = file_metadata
                            print(f"WAV Format Info: {wav_info['sample_rate']}Hz, {wav_info['bit_depth']}-bit, {wav_info['channels']} channel(s)")

                        # Debug: Show if Magpie metadata was found
                        magpie_fields = [k for k in file_metadata.keys() if any(term in k.lower() for term in
                                       ['log', 'time', 'temperature', 'voltage', 'current', 'recording', 'stopping'])]
                        if magpie_fields:
                            print(f"  Magpie metadata found: {len(magpie_fields)} fields")

                        # Check for DMA overrun in WAV header
                        has_dma_overrun = check_wav_header_for_dma_overrun(file_path)

                        # Store data
                        datetime_list.append(file_datetime)
                        filesize_list.append(file_size)
                        filenames_list.append(file)
                        dma_overrun_list.append(has_dma_overrun)
                        wav_metadata_list.append(file_metadata)

                        dma_status = " [DMA OVERRUN]" if has_dma_overrun else ""
                        # Show WAV header info (sample rate, bit depth, channels from header)
                        format_info = ""
                        if file_metadata and 'sample_rate' in file_metadata:
                            sr = file_metadata.get('sample_rate', '?')
                            bd = file_metadata.get('bit_depth', '?')
                            ch = file_metadata.get('channels', '?')
                            format_info = f" [{sr}Hz/{bd}bit/{ch}ch]"
                        print(f"Found: {file} - {file_datetime} - {file_size:,} bytes{format_info}{dma_status}")

                    except (ValueError, OSError) as e:
                        print(f"Error processing {file}: {e}")
                        continue

    return datetime_list, filesize_list, filenames_list, dma_overrun_list, wav_metadata_list, wav_info

def convert_file_sizes(sizes_bytes, unit='MB'):
    """
    Convert file sizes to more readable units

    Args:
        sizes_bytes (list): List of file sizes in bytes
        unit (str): Target unit ('B', 'KB', 'MB', 'GB')

    Returns:
        list: Converted file sizes
    """
    conversions = {
        'B': 1,
        'KB': 1024,
        'MB': 1024**2,
        'GB': 1024**3
    }

    factor = conversions.get(unit.upper(), 1024**2)  # Default to MB
    return [size / factor for size in sizes_bytes]

def plot_wav_data(datetime_list, filesize_list, dma_overrun_list, wav_metadata_list, wav_info=None, size_unit='MB', folder_path=""):
    """
    Create interactive plots of datetime vs file size and individual metadata fields

    Args:
        datetime_list (list): List of datetime objects
        filesize_list (list): List of file sizes in bytes
        dma_overrun_list (list): List of boolean values indicating DMA overrun
        wav_metadata_list (list): List of dictionaries containing WAV metadata for each file
        wav_info (dict): WAV format information (sample rate, bit depth, channels)
        size_unit (str): Unit for file size display
        folder_path (str): Path being analyzed (for title)
    """

    if not datetime_list or not filesize_list:
        messagebox.showwarning("No Data", "No matching WAV files found to plot!")
        return

    # Convert file sizes to specified unit
    converted_sizes = convert_file_sizes(filesize_list, size_unit)

    # Create the plot with split layout - make info panel wider
    fig = plt.figure(figsize=(14, 8))  # Reduced size to fit on screen

    # Create main plot on the left (3/4 of width)
    ax = fig.add_subplot(1, 4, (1, 3))  # Takes columns 1-3 out of 4

    # Create info panel on the right (1/4 of width)
    info_ax = fig.add_subplot(1, 4, 4)  # Takes column 4 out of 4
    info_ax.axis('off')  # Hide axes for info panel

    # Separate data points based on DMA overrun status
    normal_dates = []
    normal_sizes = []
    normal_indices = []
    overrun_dates = []
    overrun_sizes = []
    overrun_indices = []

    for i, has_overrun in enumerate(dma_overrun_list):
        if has_overrun:
            overrun_dates.append(datetime_list[i])
            overrun_sizes.append(converted_sizes[i])
            overrun_indices.append(i)
        else:
            normal_dates.append(datetime_list[i])
            normal_sizes.append(converted_sizes[i])
            normal_indices.append(i)

    # Debug: Print data separation results
    print(f"\n=== DATA SEPARATION DEBUG ===")
    print(f"Total files: {len(datetime_list)}")
    print(f"Normal files: {len(normal_dates)} (indices: {normal_indices})")
    print(f"Overrun files: {len(overrun_dates)} (indices: {overrun_indices})")

    # Debug: Check metadata for each file
    print(f"\n=== METADATA DEBUG ===")
    for i, metadata in enumerate(wav_metadata_list):
        filename = metadata.get('filename', f'Unknown_{i}')
        metadata_count = len([k for k in metadata.keys() if not k.startswith('_')])
        print(f"File {i}: {filename} ({metadata_count} metadata fields)")

        # Show a few key fields to verify uniqueness
        sample_fields = {}
        for key in ['Log Time', 'Temperature(C)', 'Recording Voltage(V)']:
            if key in metadata:
                sample_fields[key] = metadata[key]
        if sample_fields:
            print(f"  Sample fields: {sample_fields}")
    print("=" * 50)

    # Plot normal data points in blue
    scatter_normal = None
    if normal_dates:
        scatter_normal = ax.scatter(normal_dates, normal_sizes, alpha=0.7, s=40,
                                  c='blue', edgecolors='navy', linewidth=0.5,
                                  label='Normal files')

    # Plot DMA overrun data points in red with X markers
    scatter_overrun = None
    if overrun_dates:
        scatter_overrun = ax.scatter(overrun_dates, overrun_sizes, alpha=0.8, s=50,
                                   c='red', edgecolors='darkred', linewidth=0.8,
                                   label='DMA Overrun detected', marker='X')

    # Connect all points with a line (sorted by datetime)
    ax.plot(datetime_list, converted_sizes, alpha=0.4, linewidth=1, color='gray', zorder=0)

    ax.set_xlabel('Date and Time', fontsize=12)
    ax.set_ylabel(f'File Size ({size_unit})', fontsize=12)

    # Create a more informative title - use full path
    folder_display = folder_path if folder_path else "Selected Folder"

    # Add WAV format info to title if available
    title = f'WAV File Sizes Over Time\nSource: {folder_display}'
    if wav_info:
        format_info = f"{wav_info['sample_rate']/1000:.0f}kHz, {wav_info['bit_depth']}-bit, {wav_info['channels']}ch"
        title += f'\nFormat: {format_info}'

    ax.set_title(title, fontsize=14, pad=20)

    ax.grid(True, alpha=0.3)

    # Debug: Print datetime range info
    time_span = max(datetime_list) - min(datetime_list)
    total_hours = time_span.total_seconds() / 3600
    print(f"Debug: Time span = {time_span}, Total hours = {total_hours:.2f}")
    print(f"Debug: First datetime = {min(datetime_list)}")
    print(f"Debug: Last datetime = {max(datetime_list)}")
    print(f"Debug: Number of data points = {len(datetime_list)}")
    
    # Use simple 1-hour intervals as requested
    ax.xaxis.set_major_locator(mdates.HourLocator(interval=1))
    ax.xaxis.set_major_formatter(mdates.DateFormatter('%Y/%m/%d %H:%M'))
    
    # Set minor ticks to 15-minute intervals
    ax.xaxis.set_minor_locator(mdates.MinuteLocator(byminute=[0, 15, 30, 45]))
    
    # If we have too many hours, increase the interval
    if total_hours > 48:  # More than 2 days
        interval = max(1, int(total_hours / 48))  # Limit to ~48 ticks
        ax.xaxis.set_major_locator(mdates.HourLocator(interval=interval))
        print(f"Debug: Using {interval}-hour intervals due to large time span")

    # Rotate labels for better readability
    plt.setp(ax.xaxis.get_majorticklabels(), rotation=45, ha='right')

    # Add some statistics to the plot
    avg_size = np.mean(converted_sizes)
    ax.axhline(y=avg_size, color='red', linestyle='--', alpha=0.7,
                label=f'Average: {avg_size:.2f} {size_unit}')

    # Add min/max lines
    min_size = np.min(converted_sizes)
    max_size = np.max(converted_sizes)
    ax.axhline(y=min_size, color='green', linestyle=':', alpha=0.5,
                label=f'Min: {min_size:.2f} {size_unit}')
    ax.axhline(y=max_size, color='orange', linestyle=':', alpha=0.5,
                label=f'Max: {max_size:.2f} {size_unit}')

    # Add click-based info display in right panel
    # Initialize info panel state
    current_info_text = [None]  # Use list to make it mutable in nested functions

    # Initialize keyboard navigation state
    current_point_index = [0]  # Current selected point index (0-based)
    keyboard_mode = [False]  # Track if we're in keyboard navigation mode

    def format_metadata_info(metadata, is_dma_overrun, file_size_mb):
        """Format metadata into a readable info panel string"""
        info_lines = []

        # Add DMA overrun status prominently
        if is_dma_overrun:
            info_lines.append("⚠️ DMA OVERRUN")
            info_lines.append("")

        # Add filename with proper text wrapping
        if 'filename' in metadata:
            filename = metadata['filename']
            info_lines.append("File:")

            # Break filename into chunks that fit the panel width
            max_chars_per_line = 30  # Adjust based on panel width
            if len(filename) <= max_chars_per_line:
                info_lines.append(f"  {filename}")
            else:
                # Split filename intelligently at underscores or other separators
                parts = filename.replace('_', '_|').replace('.', '.|').split('|')
                current_line = "  "

                for part in parts:
                    if len(current_line + part) <= max_chars_per_line + 2:  # +2 for the "  " prefix
                        current_line += part
                    else:
                        if current_line.strip():  # Don't add empty lines
                            info_lines.append(current_line)
                        current_line = "  " + part

                # Add the last line if it has content
                if current_line.strip():
                    info_lines.append(current_line)

            info_lines.append("")

        # Add file size and basic audio info
        info_lines.append(f"Size: {file_size_mb:.2f} {size_unit}")

        if 'sample_rate' in metadata:
            info_lines.append(f"Rate: {metadata['sample_rate']:,} Hz")
        if 'bit_depth' in metadata:
            info_lines.append(f"Bits: {metadata['bit_depth']}")
        if 'channels' in metadata:
            info_lines.append(f"Ch: {metadata['channels']}")
        if 'duration_seconds' in metadata:
            duration = metadata['duration_seconds']
            if duration >= 60:
                mins = int(duration // 60)
                secs = duration % 60
                info_lines.append(f"Time: {mins}m {secs:.1f}s")
            else:
                info_lines.append(f"Time: {duration:.1f}s")

        info_lines.append("")
        info_lines.append("Device Data:")

        # Skip standard WAV metadata fields
        skip_keys = {
            'filename', 'sample_rate', 'bit_depth', 'channels', 'duration_seconds',
            'file_size', 'data_size', 'audio_format', 'byte_rate', 'block_align'
        }

        # Show important metadata fields (abbreviated for space)
        metadata_count = 0
        for key, value in metadata.items():
            if key in skip_keys or not value or not str(value).strip():
                continue

            # Skip formatted versions if we have the original
            if key.endswith(" (Formatted)"):
                original_key = key.replace(" (Formatted)", "")
                if original_key in metadata:
                    continue

            # Use formatted version if available, otherwise use raw value
            formatted_key = f"{key} (Formatted)"
            display_value = metadata.get(formatted_key, value)

            # Less aggressive abbreviation for wider panel
            short_key = key.replace("Temperature", "Temp").replace("Voltage", "V").replace("Current", "I")
            if len(short_key) > 25:  # Increased from 15
                short_key = short_key[:25] + "..."

            short_value = str(display_value)
            if len(short_value) > 20:  # Increased from 12
                short_value = short_value[:20] + "..."

            info_lines.append(f"{short_key}:")
            info_lines.append(f"  {short_value}")
            metadata_count += 1

            # Allow more fields in taller panel
            if metadata_count >= 12:  # Increased from 8
                info_lines.append("...")
                break

        if metadata_count == 0:
            info_lines.append("No device data")

        return "\n".join(info_lines)

    def update_info_panel(text):
        """Update the info panel with new text"""
        if current_info_text[0]:
            current_info_text[0].remove()

        current_info_text[0] = info_ax.text(0.02, 0.98, text, transform=info_ax.transAxes,
                                           fontsize=10, verticalalignment='top',
                                           bbox=dict(boxstyle="round,pad=0.8", facecolor="lightblue", alpha=0.8),
                                           wrap=True)
        fig.canvas.draw_idle()

    def highlight_point_by_index(index):
        """Highlight a specific point by its index in the datetime_list"""
        if index < 0 or index >= len(datetime_list):
            return

        # Remove previous highlight
        if highlight_point[0]:
            highlight_point[0].remove()
            highlight_point[0] = None

        # Get point data
        x_pos = datetime_list[index]
        y_pos = converted_sizes[index]
        is_overrun = dma_overrun_list[index]

        # Add highlight with appropriate marker
        if is_overrun:
            highlight_point[0] = ax.scatter([x_pos], [y_pos], s=120, c='yellow',
                                           edgecolors='orange', linewidth=3,
                                           alpha=0.8, zorder=1001, marker='X')
        else:
            highlight_point[0] = ax.scatter([x_pos], [y_pos], s=100, c='yellow',
                                           edgecolors='orange', linewidth=2,
                                           alpha=0.8, zorder=1001)

        # Update info panel
        metadata = wav_metadata_list[index]
        info_text = format_metadata_info(metadata, is_overrun, converted_sizes[index])

        # Add navigation info to the text
        nav_info = f"Point {index + 1} of {len(datetime_list)}\n← → keys to navigate\n\n"
        info_text = nav_info + info_text

        update_info_panel(info_text)
        fig.canvas.draw_idle()

    def on_key_press(event):
        """Handle keyboard navigation"""
        if event.key == 'left':
            keyboard_mode[0] = True
            if current_point_index[0] > 0:
                current_point_index[0] -= 1
            else:
                current_point_index[0] = len(datetime_list) - 1  # Wrap to last point
            highlight_point_by_index(current_point_index[0])

        elif event.key == 'right':
            keyboard_mode[0] = True
            if current_point_index[0] < len(datetime_list) - 1:
                current_point_index[0] += 1
            else:
                current_point_index[0] = 0  # Wrap to first point
            highlight_point_by_index(current_point_index[0])

        elif event.key == 'escape':
            # Exit keyboard mode and return to hover mode
            keyboard_mode[0] = False
            if highlight_point[0]:
                highlight_point[0].remove()
                highlight_point[0] = None
            instruction_text = "Hover over any data point\nto view detailed\nmetadata information\n\nPress ← → keys to navigate\nthrough all points\n\nClick plot to return to hover mode"
            update_info_panel(instruction_text)
            fig.canvas.draw_idle()

    def on_mouse_click(event):
        """Handle mouse clicks to exit keyboard mode and return to hover mode"""
        if event.inaxes == ax and keyboard_mode[0]:
            # Exit keyboard mode when clicking on the plot
            keyboard_mode[0] = False
            if highlight_point[0]:
                highlight_point[0].remove()
                highlight_point[0] = None
            instruction_text = "Hover over any data point\nto view detailed\nmetadata information\n\nPress ← → keys to navigate\nthrough all points\n\nClick plot to return to hover mode"
            update_info_panel(instruction_text)
            fig.canvas.draw_idle()



    # Add hover highlighting
    highlight_point = [None]  # Store the highlighted point

    def on_hover(event):
        """Handle mouse hover to highlight points and show info"""
        # Don't process hover events if we're in keyboard navigation mode
        if keyboard_mode[0]:
            return

        if event.inaxes != ax:
            # Remove highlight and show default info if mouse leaves the plot area
            if highlight_point[0]:
                highlight_point[0].remove()
                highlight_point[0] = None
                # Show default instruction text
                instruction_text = "Hover over any data point\nto view detailed\nmetadata information\n\nPress ← → keys to navigate\nthrough all points\n\nClick plot to return to hover mode"
                update_info_panel(instruction_text)
                fig.canvas.draw_idle()
            return

        found_point = False

        # Check if hovering over normal points
        if scatter_normal is not None and len(normal_dates) > 0:
            contains, info = scatter_normal.contains(event)
            if contains and 'ind' in info and len(info['ind']) > 0:
                point_idx = info['ind'][0]

                if point_idx < len(normal_indices):
                    # Remove previous highlight
                    if highlight_point[0]:
                        highlight_point[0].remove()

                    # Add new highlight
                    x_pos = normal_dates[point_idx]
                    y_pos = normal_sizes[point_idx]
                    highlight_point[0] = ax.scatter([x_pos], [y_pos], s=100, c='yellow',
                                                   edgecolors='orange', linewidth=2,
                                                   alpha=0.8, zorder=1001)

                    # Update info panel with metadata
                    original_idx = normal_indices[point_idx]
                    metadata = wav_metadata_list[original_idx]
                    info_text = format_metadata_info(metadata, False, normal_sizes[point_idx])
                    update_info_panel(info_text)

                    fig.canvas.draw_idle()
                    found_point = True

        # Check if hovering over overrun points
        if not found_point and scatter_overrun is not None and len(overrun_dates) > 0:
            contains, info = scatter_overrun.contains(event)
            if contains and 'ind' in info and len(info['ind']) > 0:
                point_idx = info['ind'][0]

                if point_idx < len(overrun_indices):
                    # Remove previous highlight
                    if highlight_point[0]:
                        highlight_point[0].remove()

                    # Add new highlight
                    x_pos = overrun_dates[point_idx]
                    y_pos = overrun_sizes[point_idx]
                    highlight_point[0] = ax.scatter([x_pos], [y_pos], s=120, c='yellow',
                                                   edgecolors='orange', linewidth=3,
                                                   alpha=0.8, zorder=1001, marker='X')

                    # Update info panel with metadata
                    original_idx = overrun_indices[point_idx]
                    metadata = wav_metadata_list[original_idx]
                    info_text = format_metadata_info(metadata, True, overrun_sizes[point_idx])
                    update_info_panel(info_text)

                    fig.canvas.draw_idle()
                    found_point = True

        # Remove highlight and show default info if not hovering over any point
        if not found_point and highlight_point[0]:
            highlight_point[0].remove()
            highlight_point[0] = None
            # Show default instruction text
            instruction_text = "Hover over any data point\nto view detailed\nmetadata information\n\nPress ← → keys to navigate\nthrough all points\n\nClick plot to return to hover mode"
            update_info_panel(instruction_text)
            fig.canvas.draw_idle()

    # Connect hover, keyboard, and mouse click events
    fig.canvas.mpl_connect('motion_notify_event', on_hover)
    fig.canvas.mpl_connect('key_press_event', on_key_press)
    fig.canvas.mpl_connect('button_press_event', on_mouse_click)

    # Set window title with navigation instructions
    try:
        # Try different methods depending on the backend
        if hasattr(fig.canvas, 'manager') and hasattr(fig.canvas.manager, 'set_window_title'):
            fig.canvas.manager.set_window_title('WAV Data Plotter - Click plot area then use ← → keys to navigate')
        elif hasattr(fig.canvas, 'manager') and hasattr(fig.canvas.manager, 'window'):
            if hasattr(fig.canvas.manager.window, 'wm_title'):
                fig.canvas.manager.window.wm_title('WAV Data Plotter - Click plot area then use ← → keys to navigate')
    except:
        # If all methods fail, just continue without setting the title
        pass

    # Add initial instruction text to info panel
    instruction_text = "Hover over any data point\nto view detailed\nmetadata information\n\nPress ← → keys to navigate\nthrough all points\n\nClick plot to return to hover mode"
    update_info_panel(instruction_text)

    print("Interactive display enabled:")
    print("- Hover over data points to see metadata")
    print("- Use LEFT/RIGHT arrow keys to navigate through all points")
    print("- Click on plot area to return to hover mode")
    print("- Press ESC to exit keyboard navigation mode")

    ax.legend()
    plt.tight_layout()

    # Center the plot window on screen
    center_matplotlib_figure(fig)
    plt.show()

    # Print summary statistics
    dma_overrun_count = sum(dma_overrun_list)
    print(f"\n--- Summary Statistics ---")
    print(f"Total files found: {len(datetime_list)}")
    print(f"Files with DMA overrun: {dma_overrun_count}")
    print(f"DMA overrun rate: {(dma_overrun_count/len(datetime_list)*100):.1f}%")
    print(f"Date range: {min(datetime_list)} to {max(datetime_list)}")
    print(f"File size range: {min_size:.2f} - {max_size:.2f} {size_unit}")
    print(f"Average file size: {avg_size:.2f} {size_unit}")
    print(f"Total storage used: {sum(converted_sizes):.2f} {size_unit}")

class ProgressWindow:
    """Simple progress window for file scanning"""

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Scanning WAV Files...")
        self.root.geometry("400x150")
        self.root.resizable(False, False)

        # Center the window
        center_window_on_screen(self.root, 400, 150)

        # Progress bar
        self.progress_var = tk.DoubleVar()
        self.progress_bar = ttk.Progressbar(
            self.root,
            variable=self.progress_var,
            maximum=100,
            length=350
        )
        self.progress_bar.pack(pady=20)

        # Status label
        self.status_label = tk.Label(self.root, text="Initializing...", wraplength=380)
        self.status_label.pack(pady=10)

        # File count label
        self.count_label = tk.Label(self.root, text="")
        self.count_label.pack()

        self.root.update()

    def update_progress(self, current, total, current_file):
        """Update progress bar and labels"""
        if total > 0:
            progress = (current / total) * 100
            self.progress_var.set(progress)

            self.status_label.config(text=f"Processing: {current_file}")
            self.count_label.config(text=f"File {current} of {total} ({progress:.1f}%)")

            self.root.update()

    def close(self):
        """Close the progress window"""
        self.root.destroy()

def show_options_dialog(wav_metadata_list=None):
    """Show dialog for selecting file size unit and metadata field options"""

    dialog = tk.Toplevel()
    dialog.title("Plot Options")
    dialog.geometry("800x800")
    dialog.resizable(True, True)
    dialog.wm_attributes("-topmost", True)

    # Center the dialog
    center_window_on_screen(dialog, 800, 800)

    # Make dialog modal
    dialog.transient()
    dialog.grab_set()

    # Disable the window close button behavior
    dialog.protocol("WM_DELETE_WINDOW", lambda: None)

    result = {'unit': 'MB', 'save_csv': False, 'show_metadata_plots': True, 'selected_fields': [], 'cancelled': False}

    # Discover available metadata fields
    available_fields = []
    if wav_metadata_list:
        # Find all unique metadata keys across all files
        all_keys = set()
        for metadata in wav_metadata_list:
            all_keys.update(metadata.keys())

        # Skip standard WAV fields and non-varying fields
        skip_keys = {
            'filename', 'sample_rate', 'bit_depth', 'channels', 'audio_format',
            'byte_rate', 'block_align', 'file_size', 'data_size', 'duration_seconds'
        }

        # Find fields that actually vary between files and are numeric or can be plotted
        for key in all_keys:
            if key in skip_keys or key.endswith(" (Formatted)"):
                continue

            # Collect all values for this key
            values = []
            for metadata in wav_metadata_list:
                if key in metadata and metadata[key] is not None:
                    value = metadata[key]
                    # Try to convert to numeric if possible
                    try:
                        if isinstance(value, str):
                            # Handle common numeric formats
                            clean_value = value.strip()
                            if clean_value.replace('.', '').replace('-', '').replace('+', '').isdigit():
                                values.append(float(clean_value))
                            else:
                                values.append(value)
                        else:
                            values.append(float(value))
                    except (ValueError, TypeError):
                        values.append(value)

            # Check if field has varying values and at least some numeric data
            if len(set(str(v) for v in values)) > 1:  # Values vary
                numeric_values = [v for v in values if isinstance(v, (int, float))]
                if len(numeric_values) > len(values) * 0.5:  # At least 50% numeric
                    available_fields.append(key)

    # Main frame with padding
    main_frame = tk.Frame(dialog)
    main_frame.pack(fill='both', expand=True, padx=20, pady=20)

    # Title
    title_label = tk.Label(main_frame, text="Choose Display Options",
                          font=("Arial", 12, "bold"))
    title_label.pack(pady=(0, 15))

    # File size unit selection
    unit_frame = tk.LabelFrame(main_frame, text="File Size Unit",
                              font=("Arial", 10, "bold"), padx=10, pady=10)
    unit_frame.pack(fill='x', pady=(0, 15))

    unit_var = tk.StringVar(value="MB")
    units = [("Bytes (B)", "B"), ("Kilobytes (KB)", "KB"),
             ("Megabytes (MB)", "MB"), ("Gigabytes (GB)", "GB")]

    for text, value in units:
        rb = tk.Radiobutton(unit_frame, text=text, variable=unit_var,
                           value=value, font=("Arial", 9))
        rb.pack(anchor='w', pady=2)

    # Plot options
    plot_frame = tk.LabelFrame(main_frame, text="Plot Options",
                              font=("Arial", 10, "bold"), padx=10, pady=10)
    plot_frame.pack(fill='x', pady=(0, 15))

    metadata_plots_var = tk.BooleanVar(value=True)
    metadata_check = tk.Checkbutton(plot_frame, text="Show individual metadata field plots",
                                   variable=metadata_plots_var, font=("Arial", 9))
    metadata_check.pack(anchor='w')

    # Metadata field selection
    if available_fields:
        field_frame = tk.LabelFrame(main_frame, text="Select Metadata Fields to Plot",
                                   font=("Arial", 10, "bold"), padx=10, pady=10)
        field_frame.pack(fill='both', expand=True, pady=(0, 15))

        # Create scrollable frame for field checkboxes
        canvas = tk.Canvas(field_frame, height=150)
        scrollbar = tk.Scrollbar(field_frame, orient="vertical", command=canvas.yview)
        scrollable_frame = tk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        # Field selection variables
        field_vars = {}

        # Add "Select All" and "Select None" buttons
        button_frame = tk.Frame(scrollable_frame)
        button_frame.pack(fill='x', pady=(0, 10))

        def select_all():
            for var in field_vars.values():
                var.set(True)

        def select_none():
            for var in field_vars.values():
                var.set(False)

        select_all_btn = tk.Button(button_frame, text="Select All", command=select_all,
                                  font=("Arial", 8))
        select_all_btn.pack(side='left', padx=(0, 5))

        select_none_btn = tk.Button(button_frame, text="Select None", command=select_none,
                                   font=("Arial", 8))
        select_none_btn.pack(side='left')

        # Add checkboxes for each field
        for field in sorted(available_fields):
            field_vars[field] = tk.BooleanVar(value=True)  # Default to selected
            cb = tk.Checkbutton(scrollable_frame, text=field, variable=field_vars[field],
                               font=("Arial", 9))
            cb.pack(anchor='w', pady=1)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # Enable/disable field selection based on metadata plots checkbox
        def toggle_field_selection():
            state = 'normal' if metadata_plots_var.get() else 'disabled'
            for widget in scrollable_frame.winfo_children():
                if isinstance(widget, tk.Checkbutton):
                    widget.configure(state=state)
                elif isinstance(widget, tk.Frame):
                    for child in widget.winfo_children():
                        if isinstance(child, tk.Button):
                            child.configure(state=state)

        metadata_check.configure(command=toggle_field_selection)
    else:
        # No fields available
        no_fields_label = tk.Label(main_frame,
                                  text="No varying metadata fields found for plotting",
                                  font=("Arial", 9), fg="gray")
        no_fields_label.pack(pady=10)

    # Export options
    export_frame = tk.LabelFrame(main_frame, text="Export Options",
                                font=("Arial", 10, "bold"), padx=10, pady=10)
    export_frame.pack(fill='x', pady=(0, 20))

    csv_var = tk.BooleanVar()
    csv_check = tk.Checkbutton(export_frame, text="Save data to CSV file",
                              variable=csv_var, font=("Arial", 9))
    csv_check.pack(anchor='w')

    # Button frame
    button_frame = tk.Frame(main_frame)
    button_frame.pack(pady=(10, 0))

    def on_ok():
        result['unit'] = unit_var.get()
        result['save_csv'] = csv_var.get()
        result['show_metadata_plots'] = metadata_plots_var.get()

        # Collect selected metadata fields
        if available_fields and metadata_plots_var.get():
            result['selected_fields'] = [field for field, var in field_vars.items() if var.get()]
        else:
            result['selected_fields'] = []

        dialog.destroy()

    def on_cancel():
        result['cancelled'] = True
        dialog.destroy()

    # Styled buttons
    ok_button = tk.Button(button_frame, text="Create Plot", command=on_ok,
                         width=12, height=2, font=("Arial", 9, "bold"),
                         bg="#4CAF50", fg="white", cursor="hand2")
    ok_button.pack(side='left', padx=10)

    cancel_button = tk.Button(button_frame, text="Cancel", command=on_cancel,
                             width=12, height=2, font=("Arial", 9),
                             bg="#f44336", fg="white", cursor="hand2")
    cancel_button.pack(side='left', padx=10)

    # Set focus to OK button and make it default
    ok_button.focus_set()
    dialog.bind('<Return>', lambda event: on_ok())
    dialog.bind('<Escape>', lambda event: on_cancel())

    # Add instruction text
    instruction_label = tk.Label(main_frame,
                                text="Select your preferences and click 'Create Plot' to continue.",
                                font=("Arial", 8), fg="gray")
    instruction_label.pack(pady=(10, 0))

    dialog.wait_window()
    return result

def main():
    """
    Main function to execute the WAV file analysis
    """

    print("WAV File Analyzer for Magpie Audio Files")
    print("=" * 45)

    # Select folder using GUI
    folder_path = select_folder()

    if not folder_path:
        print("No folder selected. Exiting...")
        return

    # Check if path exists
    if not os.path.exists(folder_path):
        messagebox.showerror("Error", f"Selected path does not exist!")
        return

    print(f"Scanning for WAV files in: {folder_path}")
    print("-" * 50)

    # Create progress window
    progress_window = ProgressWindow()

    try:
        # Scan for WAV files with progress updates
        datetime_data, filesize_data, filenames_data, dma_overrun_data, wav_metadata_data, wav_info = scan_wav_files(
            folder_path,
            progress_callback=progress_window.update_progress
        )
    finally:
        progress_window.close()

    if not datetime_data:
        messagebox.showinfo("No Files Found",
                           "No matching WAV files found!\n\n" +
                           "Supported filename formats:\n\n" +
                           "Old: Magpie##_YYYYMMDD_hhmmss[.fff]Z_##kHz_##_bit_#_channel.wav\n" +
                           "New: *_YYYYMMDD_hhmmss[.fff]Z.wav\n\n" +
                           "Examples:\n" +
                           "Magpie00_20251102_144202Z_48kHz_16_bit_1_channel.wav\n" +
                           "S0000NY00_048K_S00_MAG000001-16b-CH01_20251126_192823.045Z.wav")
        return

    # Create DataFrame for easier manipulation
    df = pd.DataFrame({
        'filename': filenames_data,
        'datetime': datetime_data,
        'file_size_bytes': filesize_data,
        'dma_overrun': dma_overrun_data,
        'wav_metadata': wav_metadata_data
    })

    # Sort by datetime
    df = df.sort_values('datetime')

    print(f"\nFound {len(df)} matching WAV files")
    dma_count = df['dma_overrun'].sum()
    print(f"Files with DMA overrun detected: {dma_count}")
    print(f"First few entries:")
    print(df[['filename', 'datetime', 'file_size_bytes', 'dma_overrun']].head())

    # Show options dialog with metadata for field selection
    options = show_options_dialog(df['wav_metadata'].tolist())

    if options['cancelled']:
        print("Analysis cancelled by user.")
        return

    # Plot the file size data
    plot_wav_data(
        df['datetime'].tolist(),
        df['file_size_bytes'].tolist(),
        df['dma_overrun'].tolist(),
        df['wav_metadata'].tolist(),
        wav_info=wav_info,
        size_unit=options['unit'],
        folder_path=folder_path
    )

    # Plot individual metadata fields if requested
    if options['show_metadata_plots'] and options['selected_fields']:
        plot_metadata_fields(
            df['datetime'].tolist(),
            df['wav_metadata'].tolist(),
            df['dma_overrun'].tolist(),
            folder_path=folder_path,
            selected_fields=options['selected_fields'],
            wav_info=wav_info
        )

    # Save to CSV if requested
    if options['save_csv']:
        csv_filename = "wav_file_analysis.csv"
        df_export = df.copy()
        df_export['file_size_MB'] = convert_file_sizes(df_export['file_size_bytes'], 'MB')

        try:
            df_export.to_csv(csv_filename, index=False)
            messagebox.showinfo("CSV Saved", f"Data saved to {csv_filename}")
            print(f"Data saved to {csv_filename}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save CSV: {str(e)}")

    print("\nAnalysis complete!")

def plot_metadata_fields(datetime_list, wav_metadata_list, dma_overrun_list, folder_path="", selected_fields=None, wav_info=None):
    """
    Create individual plots for each metadata field that varies between files

    Args:
        datetime_list (list): List of datetime objects
        wav_metadata_list (list): List of dictionaries containing WAV metadata for each file
        dma_overrun_list (list): List of boolean values indicating DMA overrun
        folder_path (str): Path being analyzed (for title)
        selected_fields (list): List of specific fields to plot, or None for all fields
        wav_info (dict): WAV format information (sample rate, bit depth, channels)
    """
    if not datetime_list or not wav_metadata_list:
        messagebox.showwarning("No Data", "No metadata found to plot!")
        return

    # Find all unique metadata keys across all files
    all_keys = set()
    for metadata in wav_metadata_list:
        all_keys.update(metadata.keys())

    # Skip standard WAV fields and non-varying fields
    skip_keys = {
        'filename', 'sample_rate', 'bit_depth', 'channels', 'audio_format',
        'byte_rate', 'block_align', 'file_size', 'data_size', 'duration_seconds'
    }

    # Find fields that actually vary between files and are numeric or can be plotted
    varying_fields = {}

    for key in all_keys:
        if key in skip_keys or key.endswith(" (Formatted)"):
            continue

        # Collect all values for this key
        values = []
        for metadata in wav_metadata_list:
            if key in metadata and metadata[key] is not None:
                value = metadata[key]
                # Try to convert to numeric if possible
                try:
                    if isinstance(value, str):
                        # Handle common numeric formats
                        clean_value = value.strip()
                        if clean_value.replace('.', '').replace('-', '').replace('+', '').isdigit():
                            values.append(float(clean_value))
                        else:
                            values.append(value)
                    else:
                        values.append(float(value))
                except (ValueError, TypeError):
                    values.append(value)

        # Check if field has varying values and at least some numeric data
        if len(set(str(v) for v in values)) > 1:  # Values vary
            numeric_values = [v for v in values if isinstance(v, (int, float))]
            if len(numeric_values) > len(values) * 0.5:  # At least 50% numeric
                # Only include if no specific selection or if field is selected
                if selected_fields is None or key in selected_fields:
                    varying_fields[key] = values

    if not varying_fields:
        print("No varying numeric metadata fields found to plot.")
        return

    print(f"\nFound {len(varying_fields)} varying metadata fields to plot:")
    for key in sorted(varying_fields.keys()):
        print(f"  - {key}")

    # Create subplot grid
    n_fields = len(varying_fields)
    if n_fields == 1:
        rows, cols = 1, 1
    elif n_fields <= 4:
        rows, cols = 2, 2
    elif n_fields <= 6:
        rows, cols = 2, 3
    elif n_fields <= 9:
        rows, cols = 3, 3
    else:
        rows, cols = 4, 3  # Max 12 plots
        if n_fields > 12:
            print(f"Warning: Only plotting first 12 of {n_fields} fields")
            varying_fields = dict(list(varying_fields.items())[:12])

    fig, axes = plt.subplots(rows, cols, figsize=(4*cols, 3*rows))
    if n_fields == 1:
        axes = [axes]
    elif rows == 1 or cols == 1:
        axes = axes.flatten()
    else:
        axes = axes.flatten()

    # Create title for the figure - use full path
    folder_display = folder_path if folder_path else "Selected Folder"

    # Add WAV format info to title if available
    title = f'WAV Metadata Overtime\nSource: {folder_display}'
    if wav_info:
        format_info = f"{wav_info['sample_rate']/1000:.0f}kHz, {wav_info['bit_depth']}-bit, {wav_info['channels']}ch"
        title += f'\nFormat: {format_info}'

    fig.suptitle(title, fontsize=12, y=0.96)

    # Plot each field
    for idx, (field_name, values) in enumerate(sorted(varying_fields.items())):
        if idx >= len(axes):
            break

        ax = axes[idx]

        # Prepare data for plotting
        plot_datetimes = []
        plot_values = []
        plot_dma_status = []

        for i, metadata in enumerate(wav_metadata_list):
            if field_name in metadata and metadata[field_name] is not None:
                value = metadata[field_name]
                try:
                    if isinstance(value, str):
                        clean_value = value.strip()
                        if clean_value.replace('.', '').replace('-', '').replace('+', '').isdigit():
                            numeric_value = float(clean_value)
                        else:
                            continue  # Skip non-numeric values for this plot
                    else:
                        numeric_value = float(value)

                    plot_datetimes.append(datetime_list[i])
                    plot_values.append(numeric_value)
                    plot_dma_status.append(dma_overrun_list[i])
                except (ValueError, TypeError):
                    continue

        if not plot_values:
            ax.text(0.5, 0.5, f'No numeric data\nfor {field_name}',
                   ha='center', va='center', transform=ax.transAxes)
            ax.set_title(field_name, fontsize=10)
            continue

        # Separate normal and DMA overrun points
        normal_dates = []
        normal_values = []
        overrun_dates = []
        overrun_values = []

        for dt, val, is_overrun in zip(plot_datetimes, plot_values, plot_dma_status):
            if is_overrun:
                overrun_dates.append(dt)
                overrun_values.append(val)
            else:
                normal_dates.append(dt)
                normal_values.append(val)

        # Plot the data
        if normal_dates:
            ax.scatter(normal_dates, normal_values, alpha=0.7, s=30,
                      c='blue', edgecolors='navy', linewidth=0.5, label='Normal')

        if overrun_dates:
            ax.scatter(overrun_dates, overrun_values, alpha=0.8, s=40,
                      c='red', edgecolors='darkred', linewidth=0.8,
                      label='DMA Overrun', marker='X')

        # Connect points with line
        if len(plot_datetimes) > 1:
            # Sort by datetime for proper line connection
            sorted_data = sorted(zip(plot_datetimes, plot_values))
            sorted_dates, sorted_values = zip(*sorted_data)
            ax.plot(sorted_dates, sorted_values, alpha=0.4, linewidth=1, color='gray', zorder=0)

        # Format the plot
        ax.set_title(field_name, fontsize=10, pad=10)
        ax.grid(True, alpha=0.3)

        # Format x-axis with simple hourly intervals
        if plot_datetimes:
            time_span = max(plot_datetimes) - min(plot_datetimes)
            total_hours = time_span.total_seconds() / 3600
            
            # Use 1-hour intervals, but increase if too many hours
            if total_hours <= 24:
                ax.xaxis.set_major_locator(mdates.HourLocator(interval=1))
                ax.xaxis.set_major_formatter(mdates.DateFormatter('%Y/%m/%d\n%H:%M'))
            elif total_hours <= 168:  # 1 week
                interval = max(1, int(total_hours / 24))  # Limit to ~24 ticks
                ax.xaxis.set_major_locator(mdates.HourLocator(interval=interval))
                ax.xaxis.set_major_formatter(mdates.DateFormatter('%Y/%m/%d\n%H:%M'))
            else:
                interval = max(1, int(total_hours / (24 * 7)))  # Weekly intervals
                ax.xaxis.set_major_locator(mdates.DayLocator(interval=interval))
                ax.xaxis.set_major_formatter(mdates.DateFormatter('%Y/%m/%d'))
        plt.setp(ax.xaxis.get_majorticklabels(), rotation=45, ha='right', fontsize=6)
        plt.setp(ax.yaxis.get_majorticklabels(), fontsize=6)

        # Add statistics
        if plot_values:
            avg_val = np.mean(plot_values)
            ax.axhline(y=avg_val, color='red', linestyle='--', alpha=0.5, linewidth=1)

            # Add value range info in bottom left
            min_val, max_val = min(plot_values), max(plot_values)
            if max_val != min_val:
                ax.text(0.02, 0.02, f'Range: {min_val:.2f} - {max_val:.2f}\nAvg: {avg_val:.2f}',
                       transform=ax.transAxes, fontsize=8, verticalalignment='bottom',
                       bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8))

    # Hide unused subplots
    for idx in range(len(varying_fields), len(axes)):
        axes[idx].set_visible(False)

    # Add figure-level legend in top-right corner
    # Create dummy plots for legend
    normal_handle = plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='blue',
                              markeredgecolor='navy', markersize=8, label='Normal')
    overrun_handle = plt.Line2D([0], [0], marker='X', color='w', markerfacecolor='red',
                               markeredgecolor='darkred', markersize=10, label='DMA Overrun')

    fig.legend(handles=[normal_handle, overrun_handle], loc='upper right',
               bbox_to_anchor=(0.98, 0.98), fontsize=10, framealpha=0.9)

    plt.tight_layout()
    plt.subplots_adjust(top=0.82)  # Make room for suptitle with format info

    # Center the metadata plots window on screen
    center_matplotlib_figure(fig)
    plt.show()

    # Print summary of plotted fields
    print(f"\nMetadata plots created for {len(varying_fields)} fields:")
    for field_name in sorted(varying_fields.keys()):
        values = varying_fields[field_name]
        numeric_values = [v for v in values if isinstance(v, (int, float))]
        if numeric_values:
            print(f"  {field_name}: {len(numeric_values)} numeric values, range {min(numeric_values):.2f} - {max(numeric_values):.2f}")

if __name__ == "__main__":
    main()