/**
 * @file    sys_config.h
 * @brief   This module simply provides some definitions for configuring the demo application.
 * @details Configuration values can be overridden at runtime by placing a schedule.sch file
 *          on SD_CARD_BANK_CARD_SLOT_5. If no schedule file is found, these defaults are used.
 *
 * @warning This header must ONLY be included by main.c
 */

#ifndef SYS_CONFIG_H_
#define SYS_CONFIG_H_

/* Enforce single-file inclusion - main.c must define this before including */
#ifndef SYS_CONFIG_ALLOW_INCLUDE
#error "system_config.h can only be included by main.c. Define SYS_CONFIG_ALLOW_INCLUDE before including."
#endif

/* Includes ----------------------------------------------------------------------------------------------------------*/

#include "audio_enums.h"
#include "sd_card_bank_ctl.h"
#include "wav_header.h"

/* Public defines ----------------------------------------------------------------------------------------------------*/

// Project identification (not overridable via schedule.sch)
#define SYS_CONFIG_PROJECT_ID   "S0000NY00"   // Project identifier (e.g., S6741NY01)
#define SYS_CONFIG_SITE_ID      "S00"         // Site identifier (e.g., S01)
#define SYS_CONFIG_RECORDER_ID  "MAG000001"   // Recorder identifier (used for session folders and filenames)

// SD card slot for schedule file
#define SYS_CONFIG_SCHEDULE_SD_SLOT     SD_CARD_BANK_CARD_SLOT_5
#define SYS_CONFIG_SCHEDULE_FILENAME    "schedule.sch"

/* Runtime-configurable variables (can be overridden by schedule.sch) -------------------------------------------------
 * Edit these default values as needed. They will be overridden if a valid schedule.sch file is found on SD slot 5.
 */

// the length of the WAVE file to write to the SD card, a positive integer
// max value 4k seconds, about 70 minutes
static uint32_t SYS_CONFIG_AUDIO_FILE_LEN_IN_SECONDS = 3600;

// Sample rate setting (24000, 48000, 96000, 192000, 384000)
static uint32_t SYS_CONFIG_SAMPLE_RATE = AUDIO_SAMPLE_RATE_48kHz;

// Bit depth setting (16 or 24)
static uint32_t SYS_CONFIG_NUM_BIT_DEPTH = AUDIO_BIT_DEPTH_16_BITS_PER_SAMPLE;

// Number of channels: WAVE_HEADER_MONO (1) or WAVE_HEADER_STEREO (2)
static uint32_t SYS_CONFIG_NUM_CHANNEL = WAVE_HEADER_MONO;

// In mono mode, which channel to record: AUDIO_CHANNEL_0 or AUDIO_CHANNEL_1
static Audio_Channel_t SYS_CONFIG_CHANNEL_TO_USE_FOR_MONO_MODE = AUDIO_CHANNEL_0;

/* Fixed configuration (not overridable via schedule.sch) ------------------------------------------------------------*/

// gain is shared by both mics
static const Audio_Gain_t SYS_CONFIG_AUDIO0_GAIN = AUDIO_GAIN_40dB;
static const Audio_Gain_t SYS_CONFIG_AUDIO1_GAIN = AUDIO_GAIN_40dB;

// SD card slot to use for recording
static const SD_Card_Bank_Card_Slot_t SYS_CONFIG_SD_CARD_SLOT_TO_USE = SD_CARD_BANK_CARD_SLOT_0;

#endif /* SYS_CONFIG_H_ */
