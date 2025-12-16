/* Private includes --------------------------------------------------------------------------------------------------*/

#include "wav_header.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

/* Private defines ---------------------------------------------------------------------------------------------------*/

// This value depends on whether or not there is the extra 2 bytes after the bits_per_sample field and before "data"
// If the extra two bytes are there, this must be 18, if not it must be 16
#define WAVE_HEADER_FMT_CHUNK_SIZE (16)

// Always use PCM format for now. If we implement compression in the future this may change
#define WAVE_HEADER_FMT_TAG_PCM (1)

// Header size expanded to 4 sectors (2048 bytes) for metadata storage
#define WAVE_HEADER_SECTOR_SIZE (512)
#define WAVE_HEADER_NUM_SECTORS (4)
#define WAVE_HEADER_TOTAL_SIZE (WAVE_HEADER_SECTOR_SIZE * WAVE_HEADER_NUM_SECTORS)
#define WAVE_HEADER_CORE_SIZE (44)
#define WAVE_HEADER_METADATA_SIZE (WAVE_HEADER_TOTAL_SIZE - WAVE_HEADER_CORE_SIZE)

// Key-value pair constants
#define MAX_KEY_LENGTH (32)
#define MAX_VALUE_LENGTH (64)
#define MAX_METADATA_PAIRS (20)

/* Private types -----------------------------------------------------------------------------------------------------*/

/**
 * @brief A structure for holding key-value metadata pairs
 */
typedef struct __attribute__((packed))
{
    char key[MAX_KEY_LENGTH];
    char value[MAX_VALUE_LENGTH];
    uint8_t valid; // 1 if this entry is valid, 0 if empty
} Metadata_Pair_t;

/**
 * @brief A structure for holding wav file header information is represented here.
 *
 * This can be cast to a char* and written directly to disk.
 * 
 * IMPORTANT: The "data" chunk MUST be the last thing before actual audio samples.
 * Metadata is placed in a custom "META" chunk BEFORE the data chunk.
 */
typedef struct __attribute__((packed))
{
    // RIFF header
    char riff[4];              /* always the string "RIFF" */
    uint32_t file_len_minus_8; /* file length in bytes - 8 bytes */
    char wave[4];              /* always the string "WAVE" */
    
    // fmt chunk
    char fmt_[4];              /* always the string "fmt " (note the trailing space) */
    uint32_t fmt_chunk_size;   /* size of FMT chunk in bytes, usually 16 or 18 */
    uint16_t fmt_tag;          /* 1=PCM, 257=Mu-Law, 258=A-Law, 259=ADPCM  */
    uint16_t num_channels;     /* 1=mono, 2=stereo */
    uint32_t sample_rate;      /* samples per second */
    uint32_t bytes_per_sec;    /* bytes per second = sample_rate * bytes_per_sample */
    uint16_t bytes_per_block;  /* num channels * bytes per sample */
    uint16_t bits_per_sample;  /* number of bits per sample */
    
    // Custom META chunk for metadata (placed BEFORE data chunk)
    char meta[4];              /* custom chunk ID "META" */
    uint32_t meta_chunk_size;  /* size of metadata chunk */
    uint32_t metadata_version; /* Version of metadata format */
    uint32_t metadata_count;   /* Number of valid metadata pairs */
    Metadata_Pair_t metadata_pairs[MAX_METADATA_PAIRS];
    
    // Padding to align to sector boundary
    char pad_to_sectors[WAVE_HEADER_METADATA_SIZE - 8 - sizeof(uint32_t) - sizeof(uint32_t) - (sizeof(Metadata_Pair_t) * MAX_METADATA_PAIRS)];
    
    // data chunk - MUST be last before audio samples
    char data[4];              /* always the string "data" */
    uint32_t data_length;      /* data length in bytes (file_length - the length of this struct) */
} Wave_Header_t;

/* Private variables -------------------------------------------------------------------------------------------------*/

// we use one static instance of Wave_Header_t and update its fields using the wav_header_set_attributes(a) function
static Wave_Header_t wave_header = {
    // Some attributes of the wave header never change. We set them once here, and then never update them again.
    // Other attributes (file length, sample rate, etc) do change dynamically via calls to wav_header_set_attributes(a)
    .riff = {'R', 'I', 'F', 'F'},
    .wave = {'W', 'A', 'V', 'E'},
    .fmt_ = {'f', 'm', 't', ' '},
    .fmt_chunk_size = WAVE_HEADER_FMT_CHUNK_SIZE,
    .fmt_tag = WAVE_HEADER_FMT_TAG_PCM,
    .meta = {'M', 'E', 'T', 'A'},
    .meta_chunk_size = WAVE_HEADER_METADATA_SIZE - 8,  // chunk size excludes the chunk ID and size fields
    .metadata_version = 1,
    .metadata_count = 0,
    .metadata_pairs = {{{0}, {0}, 0}},
    .pad_to_sectors = {0},
    .data = {'d', 'a', 't', 'a'},
};

const uint32_t HEADER_LENGTH = sizeof(wave_header);

/* Public function definitions ---------------------------------------------------------------------------------------*/

void wav_header_set_attributes(Wave_Header_Attributes_t *attributes)
{
    wave_header.file_len_minus_8 = attributes->file_length - 8;
    wave_header.num_channels = attributes->num_channels;
    wave_header.sample_rate = attributes->sample_rate;
    wave_header.bytes_per_block = (attributes->bits_per_sample / 8) * wave_header.num_channels;
    wave_header.bytes_per_sec = wave_header.bytes_per_block * wave_header.sample_rate;
    wave_header.bits_per_sample = attributes->bits_per_sample;
    wave_header.data_length = attributes->file_length - HEADER_LENGTH;
}

char *wav_header_get_header()
{
    // cast the struct as an array of bytes so we can write it directly to the SD card
    return (char *)&wave_header;
}

uint32_t wav_header_get_header_length()
{
    return HEADER_LENGTH;
}

Wav_Metadata_Result_t wav_header_add_metadata(const char *key, const char *value)
{
    // Validate input parameters
    if (!key || !value) {
        return WAV_METADATA_ERROR_INVALID_PARAM;
    }
    
    if (strlen(key) >= MAX_KEY_LENGTH || strlen(value) >= MAX_VALUE_LENGTH) {
        return WAV_METADATA_ERROR_TOO_LONG;
    }
    
    // Check if we have space for more metadata
    if (wave_header.metadata_count >= MAX_METADATA_PAIRS) {
        return WAV_METADATA_ERROR_NO_SPACE;
    }
    
    // Generate a unique key by appending incremental numbers if key already exists
    char unique_key[MAX_KEY_LENGTH];
    strncpy(unique_key, key, MAX_KEY_LENGTH - 1);
    unique_key[MAX_KEY_LENGTH - 1] = '\0';
    
    uint32_t counter = 0;
    bool key_exists = true;
    
    while (key_exists) {
        key_exists = false;
        
        // Check if current unique_key already exists
        for (uint32_t i = 0; i < wave_header.metadata_count; i++) {
            if (wave_header.metadata_pairs[i].valid && 
                strncmp(wave_header.metadata_pairs[i].key, unique_key, MAX_KEY_LENGTH) == 0) {
                key_exists = true;
                break;
            }
        }
        
        if (key_exists) {
            counter++;
            // Create new key with incremental number
            int result = snprintf(unique_key, MAX_KEY_LENGTH, "%s_%u", key, counter);
            
            // Check if the generated key would be too long
            if (result >= MAX_KEY_LENGTH) {
                return WAV_METADATA_ERROR_TOO_LONG;
            }
        }
    }
    
    // Add new key-value pair with unique key
    uint32_t index = wave_header.metadata_count;
    strncpy(wave_header.metadata_pairs[index].key, unique_key, MAX_KEY_LENGTH - 1);
    wave_header.metadata_pairs[index].key[MAX_KEY_LENGTH - 1] = '\0';
    
    strncpy(wave_header.metadata_pairs[index].value, value, MAX_VALUE_LENGTH - 1);
    wave_header.metadata_pairs[index].value[MAX_VALUE_LENGTH - 1] = '\0';
    
    wave_header.metadata_pairs[index].valid = 1;
    wave_header.metadata_count++;
    
    return WAV_METADATA_SUCCESS;
}

const char* wav_header_get_metadata(const char *key)
{
    if (!key) {
        return NULL;
    }
    
    for (uint32_t i = 0; i < wave_header.metadata_count; i++) {
        if (wave_header.metadata_pairs[i].valid && 
            strncmp(wave_header.metadata_pairs[i].key, key, MAX_KEY_LENGTH) == 0) {
            return wave_header.metadata_pairs[i].value;
        }
    }
    
    return NULL; // Key not found
}

Wav_Metadata_Result_t wav_header_remove_metadata(const char *key)
{
    if (!key) {
        return WAV_METADATA_ERROR_INVALID_PARAM;
    }
    
    for (uint32_t i = 0; i < wave_header.metadata_count; i++) {
        if (wave_header.metadata_pairs[i].valid && 
            strncmp(wave_header.metadata_pairs[i].key, key, MAX_KEY_LENGTH) == 0) {
            
            // Mark as invalid
            wave_header.metadata_pairs[i].valid = 0;
            memset(wave_header.metadata_pairs[i].key, 0, MAX_KEY_LENGTH);
            memset(wave_header.metadata_pairs[i].value, 0, MAX_VALUE_LENGTH);
            
            // Compact the array by moving later entries forward
            for (uint32_t j = i; j < wave_header.metadata_count - 1; j++) {
                wave_header.metadata_pairs[j] = wave_header.metadata_pairs[j + 1];
            }
            
            // Clear the last entry
            memset(&wave_header.metadata_pairs[wave_header.metadata_count - 1], 0, sizeof(Metadata_Pair_t));
            wave_header.metadata_count--;
            
            return WAV_METADATA_SUCCESS;
        }
    }
    
    return WAV_METADATA_ERROR_NOT_FOUND;
}

void wav_header_clear_metadata()
{
    wave_header.metadata_count = 0;
    memset(wave_header.metadata_pairs, 0, sizeof(wave_header.metadata_pairs));
}

uint32_t wav_header_get_metadata_count()
{
    return wave_header.metadata_count;
}

void wav_header_debug_sizes()
{
    printf("WAV Header Size Debug:\n");
    printf("  Core header size: %d bytes\n", WAVE_HEADER_CORE_SIZE);
    printf("  Total header size: %d bytes\n", WAVE_HEADER_TOTAL_SIZE);
    printf("  Metadata section size: %d bytes\n", WAVE_HEADER_METADATA_SIZE);
    printf("  Actual struct size: %d bytes\n", (int)sizeof(Wave_Header_t));
    printf("  Metadata version size: %d bytes\n", (int)sizeof(uint32_t));
    printf("  Metadata count size: %d bytes\n", (int)sizeof(uint32_t));
    printf("  Metadata pairs size: %d bytes\n", (int)(sizeof(Metadata_Pair_t) * MAX_METADATA_PAIRS));
    printf("  Padding size: %d bytes\n", (int)(WAVE_HEADER_METADATA_SIZE - sizeof(uint32_t) - sizeof(uint32_t) - (sizeof(Metadata_Pair_t) * MAX_METADATA_PAIRS)));
}

const char* wav_metadata_result_to_string(Wav_Metadata_Result_t result)
{
    switch (result) {
        case WAV_METADATA_SUCCESS:
            return "Success";
        case WAV_METADATA_UPDATED:
            return "Updated existing key";
        case WAV_METADATA_ERROR_NO_SPACE:
            return "No space available";
        case WAV_METADATA_ERROR_INVALID_PARAM:
            return "Invalid parameter";
        case WAV_METADATA_ERROR_TOO_LONG:
            return "Key or value too long";
        case WAV_METADATA_ERROR_NOT_FOUND:
            return "Key not found";
        default:
            return "Unknown error";
    }
}