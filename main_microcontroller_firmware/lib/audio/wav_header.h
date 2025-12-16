/**
 * @file    wav_header.h
 * @brief   A software module for generating WAVE file headers is represented here.
 * @details WAVE is a specific format for writing audio files. WAVE headers contain information about the audio sample
 * rate, bit depth, file size, and other metadata.
 *
 * We can configure WAVE headers with 4 pieces of information:
 * - Mono/stereo
 * - Bits per sample
 * - Sample rate
 * - Total file length
 *
 * Other internal fields can be derived from the above 4 numbers.
 *
 * The file length field is the total size of the entire file, and includes the size of the header itself.
 * Because of this, it is common to write a file like this:
 * 1) open a new file for writing
 * 2) advance the write pointer by the size of the wave header so the audio data starts right after the header
 * 3) write all the audio data
 * 4) save the size of the file into the file_length field of a Wave_Header_Attributes_t variable
 *      a) the size we just saved includes both the audio data and the (currently blank) header
 * 5) reset the write pointer back to the top of the file
 * 6) use the Wave_Header_Attributes_t variable to set the wave attributes, including the total file size just set
 * 7) write the wave header
 * 8) close the file
 *
 * In code it would look something like this (psuedocode, some args omitted for clarity):
 * 1) f_open(my_file_name);
 * 2) f_lseek(wav_header_get_header_length());
 * 3) f_write(audio_buffer, audio_buffer_size);
 *    ... potentially more rounds of writing audio data to the SD card
 * 4) wav_attributes.file_length = f_size();
 * 5) f_lseek(0);
 * 6) wav_header_set_attributes(&wav_attributes);
 * 7) f_write(wav_header_get_header(), wav_header_get_header_length());
 * 8) f_close();
 *
 * The other wave header attributes may be set at the same time as the total file length, or set at an earlier time.
 */

#ifndef WAV_HEADER_H_
#define WAV_HEADER_H_

/* Includes ----------------------------------------------------------------------------------------------------------*/

#include "audio_enums.h"

#include <stdint.h>

/* Public types ------------------------------------------------------------------------------------------------------*/

/**
 * @brief Enumerated wave header options for number of channels are represented here.
 */
typedef enum
{
    WAVE_HEADER_MONO = 1,
    WAVE_HEADER_STEREO = 2,
} Wave_Header_Num_Channels_t;

/**
 * @brief Enumerated return codes for metadata operations
 */
typedef enum
{
    WAV_METADATA_SUCCESS = 0,           /** Operation completed successfully */
    WAV_METADATA_UPDATED = 1,           /** Existing key was updated successfully (used by other functions) */
    WAV_METADATA_ERROR_NO_SPACE = -1,   /** No space available for more metadata pairs */
    WAV_METADATA_ERROR_INVALID_PARAM = -2, /** Invalid parameters (NULL key or value) */
    WAV_METADATA_ERROR_TOO_LONG = -3,   /** Key or value exceeds maximum length */
    WAV_METADATA_ERROR_NOT_FOUND = -4,  /** Key not found during removal operation */
} Wav_Metadata_Result_t;

/**
 * @brief A structure for holding the configurable wave header attributes is represented here.
 */
typedef struct
{
    Wave_Header_Num_Channels_t num_channels; /** The enumerated number of channels */
    Audio_Bits_Per_Sample_t bits_per_sample; /** Enumerated bits per sample */
    Audio_Sample_Rate_t sample_rate;         /** The sample rate in Hz */
    uint32_t file_length;                    /** The total file length, including the length of the header */
} Wave_Header_Attributes_t;

/* Public function declarations --------------------------------------------------------------------------------------*/

/**
 * @brief `wav_header_set_attributes(a)` sets the wave header attributes to the values contained in `a`.
 *
 * @param attributes the wave header attributes to use
 *
 * @post the values of the wave header are set to the given attributes. The next time `wave_header_get_header()` is
 * called, the fields will be updated with the values from the input parameter `attributes`
 */
void wav_header_set_attributes(Wave_Header_Attributes_t *attributes);

/**
 * @brief `wav_header_get_header()` is a pointer to the wave header with the most recent set attributes applied.
 *
 * @pre `wav_header_set_attributes(a)` has been called with a valid set of attributes in `a`.
 *
 * @retval pointer to the array of bytes that comprise the wave header with all attributes set. The length of the
 * header is given by `wav_header_get_header_length()`
 */
char *wav_header_get_header(void);

/**
 * @brief `wav_header_get_header_length()` is the length in bytes of the wave header. This does not change dynamically.
 *
 * @retval the length of the WAV header in bytes.
 */
uint32_t wav_header_get_header_length(void);

/**
 * @brief `wav_header_add_metadata(key, value)` adds a key-value pair in the WAV header metadata section.
 *        If the key already exists, automatically appends an incremental number to make it unique.
 *
 * @param key The metadata key (max 31 characters)
 * @param value The metadata value (max 63 characters)
 *
 * @retval WAV_METADATA_SUCCESS: New key-value pair added successfully (may have incremental suffix if key existed)
 *         WAV_METADATA_ERROR_NO_SPACE: No space available for more metadata pairs
 *         WAV_METADATA_ERROR_INVALID_PARAM: Invalid parameters (NULL key or value)
 *         WAV_METADATA_ERROR_TOO_LONG: Key or value exceeds maximum length (including generated suffix)
 */
Wav_Metadata_Result_t wav_header_add_metadata(const char *key, const char *value);

/**
 * @brief `wav_header_get_metadata(key)` retrieves the value for a given key from the metadata section.
 *
 * @param key The metadata key to look up
 *
 * @retval Pointer to the value string if found, NULL if key not found or invalid parameter
 */
const char* wav_header_get_metadata(const char *key);

/**
 * @brief `wav_header_remove_metadata(key)` removes a key-value pair from the metadata section.
 *
 * @param key The metadata key to remove
 *
 * @retval WAV_METADATA_SUCCESS: Key removed successfully
 *         WAV_METADATA_ERROR_INVALID_PARAM: Invalid parameter (NULL key)
 *         WAV_METADATA_ERROR_NOT_FOUND: Key not found
 */
Wav_Metadata_Result_t wav_header_remove_metadata(const char *key);

/**
 * @brief `wav_header_clear_metadata()` removes all metadata key-value pairs.
 *
 * @post All metadata pairs are cleared from the header
 */
void wav_header_clear_metadata(void);

/**
 * @brief `wav_header_get_metadata_count()` returns the number of metadata pairs currently stored.
 *
 * @retval The number of valid metadata key-value pairs
 */
uint32_t wav_header_get_metadata_count(void);

/**
 * @brief `wav_header_debug_sizes()` prints debug information about header sizes (for development/debugging).
 */
void wav_header_debug_sizes(void);

/**
 * @brief `wav_metadata_result_to_string(result)` converts a metadata result enum to a human-readable string.
 *
 * @param result The metadata operation result to convert
 *
 * @retval Pointer to a string describing the result
 */
const char* wav_metadata_result_to_string(Wav_Metadata_Result_t result);

#endif /* WAV_HEADER_H_ */