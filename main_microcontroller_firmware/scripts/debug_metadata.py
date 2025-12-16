#!/usr/bin/env python3
"""Debug script to examine WAV file metadata structure."""

import sys
import struct

def examine_wav(file_path):
    """Examine the metadata section of a WAV file."""
    print(f"Examining: {file_path}\n")
    
    with open(file_path, 'rb') as f:
        # Read RIFF header
        riff = f.read(4)
        print(f"RIFF header: {riff}")
        
        file_size = struct.unpack('<I', f.read(4))[0]
        print(f"File size: {file_size}")
        
        wave = f.read(4)
        print(f"WAVE header: {wave}")
        
        # Find chunks
        print("\nChunks found:")
        while True:
            chunk_id = f.read(4)
            if not chunk_id or len(chunk_id) < 4:
                break
            
            chunk_size = struct.unpack('<I', f.read(4))[0]
            chunk_pos = f.tell()
            
            print(f"  {chunk_id.decode('utf-8', errors='ignore')}: {chunk_size} bytes at position {chunk_pos}")
            
            if chunk_id == b'META':
                # Read META chunk
                print(f"\n  Reading META chunk ({chunk_size} bytes)...")
                meta_data = f.read(chunk_size)
                
                # Show first 200 bytes as hex
                print(f"  First 100 bytes (hex): {meta_data[:100].hex()}")
                
                # Parse META chunk structure
                # Header: 4 bytes version + 4 bytes field count
                if len(meta_data) >= 8:
                    version = struct.unpack('<I', meta_data[0:4])[0]
                    field_count = struct.unpack('<I', meta_data[4:8])[0]
                    print(f"\n  Version: {version}, Field count: {field_count}")
                    
                    # Each record: 32 bytes name + 64 bytes value + 1 byte flag = 97 bytes
                    offset = 8
                    record_size = 97
                    
                    print(f"\n  Fields found:")
                    for i in range(min(field_count, 30)):  # Limit to 30 fields
                        if offset + 96 > len(meta_data):
                            break
                        
                        field_name = meta_data[offset:offset+32].decode('utf-8', errors='ignore').rstrip('\x00')
                        field_value = meta_data[offset+32:offset+96].decode('utf-8', errors='ignore').rstrip('\x00')
                        
                        if field_name:
                            print(f"    [{i}] '{field_name}' = '{field_value}'")
                            
                            if 'Log Time' in field_name or 'time' in field_name.lower():
                                print(f"        ^ TIMESTAMP FIELD at offset {offset}")
                        
                        offset += record_size
                
                # Don't break, continue to see other chunks
                
            elif chunk_id == b'data':
                # Just note the data chunk
                print(f"\n  Data chunk found, skipping content...")
                break
            else:
                f.seek(chunk_size, 1)
                if chunk_size % 2:
                    f.seek(1, 1)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python debug_metadata.py <wav_file>")
        sys.exit(1)
    
    examine_wav(sys.argv[1])
