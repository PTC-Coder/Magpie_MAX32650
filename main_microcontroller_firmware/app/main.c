
/* Private includes --------------------------------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "mxc_delay.h"

#include "ad4630.h"
#include "afe_control.h"
#include "audio_dma.h"
#include "audio_enums.h"
#include "board.h"
#include "bsp_i2c.h"
#include "bsp_pins.h"
#include "bsp_status_led.h"
#include "data_converters.h"
#include "decimation_filter.h"
#include "sd_card.h"
#include "sd_card_bank_ctl.h"
#include "wav_header.h"

#define SYS_CONFIG_ALLOW_INCLUDE
#include "system_config.h"

#include "bsp_pushbutton.h"
#include "gpio.h"
#include "DS3231_driver.h"
#include <time.h>

#include "icc.h"
#include "fuel_gauge.h"
#include "environmental_sensor.h"

#include "mxc_device.h"
#include "mxc_sys.h"
#include "nvic_table.h"
#include "rtc.h"
#include "time_helpers.h"

#ifdef TERMINAL_IO_USE_SEGGER_RTT
#include "SEGGER_RTT.h"
#endif

// // Parameters for One-shot timer
// #define INTERVAL_TIME_OST 5 // (ms)
// #define OST_TIMER MXC_TMR5 // Can be MXC_TMR0 through MXC_TMR5
// #define OST_TIMER_IRQn TMR5_IRQn

/* Private function declarations -------------------------------------------------------------------------------------*/

/**
 * @brief `write_wav_file(a, l)` writes a wav file with attributes `a`, and length in seconds `l`, with a name
 * derived from the attributes. Calling this function starts the ADC/DMA and continuously records audio in blocking
 * fashion until the time is up.
 *
 * @pre initialization is complete for the ADC, DMA, decimation filters, and SD card, the SD card must be mounted
 *
 * @param wav_attr pointer to the wav header attributes structure holding information about sample rate, bit depth, etc
 *
 * @param file_len_secs the length of the audio file to write, in seconds
 *
 * @post this function consumes buffers from the ADC/DMA until the duration of the file length has elapsed and writes
 * the audio data out to a .wav file on the SD card. The wav header for the file is also written in this function.
 */
static void write_wav_file(Wave_Header_Attributes_t *wav_attr, uint32_t file_len_secs);

// the error handler simply rapidly blinks the given LED color forever
static void error_handler(Status_LED_Color_t c);

//system control functions
static void initialize_system(void);
static void setup_realtimeclock(void);
static void power_on_audio_chain(void);
static void power_off_audio_chain(void);

static void LED_cascade_right(void);
static void LED_cascade_left(void);

static void start_recording(uint8_t number_of_channel, Audio_Sample_Rate_t rate, Audio_Bits_Per_Sample_t bit, SD_Card_Bank_Card_Slot_t sd_slot, int32_t duration_s);
static void user_pushbutton_interrupt_callback(void *cbdata);
static void setup_user_pushbutton_interrupt(void);

// RTC sync functions
static void sync_RTC_to_DS3231(void);
static void ds3231_ISR(void *cbdata);
static void reset_MAX_RTC(int hour, int minute, int sec);
static void enable_DS3231_Interrupt(void);
void RTC_IRQHandler(void);
static void print_time_with_milliseconds(void);
static uint32_t get_internal_RTC_time(void *buff, void *strbuff);

// Folder management functions
static int setup_session_folder(void);
static int setup_day_folder(int year, int month, int day, uint32_t sample_rate, uint32_t bit_depth, uint32_t num_channels);

// Schedule file loading function
static int load_schedule_from_slot5(void);

// SD card slot selection with space checking
static SD_Card_Bank_Card_Slot_t find_available_sd_slot(uint32_t required_bytes);
static uint64_t calculate_wav_file_size(uint32_t sample_rate, uint32_t bits_per_sample, uint32_t num_channels, uint32_t duration_s);


//#define FIRST_SET_RTC 1    //uncomment this to set the clock time in the setup_realtimeclock()

#define DEFAULT_FILENAME "Default_19000101_000000"
char savedFileName[64] = DEFAULT_FILENAME;  // Increased size for longer paths

// Session folder management
static bool isFirstRecordingAfterBoot = true;
static char currentSessionFolder[32] = "";   // e.g., "MAG000001_000"

// SD card slot tracking - starts at slot 0, never goes back up after moving to next slot
static int currentMinSlotNumber = 0;  // Minimum slot number to start searching from (0-5)
static int lastUsedSlotNumber = -1;   // Track which slot was last used (-1 = none yet)
static char currentDayFolder[96] = "";       // e.g., "MAG000001_000/20251125_48kHz_16bit_CH01"
static int currentDayOfYear = -1;            // Track current day to detect day changes
static uint32_t currentSampleRateKHz = 0;    // Track current sample rate to detect changes
static uint32_t currentBitDepth = 0;         // Track current bit depth to detect changes
static uint32_t currentNumChannels = 0;      // Track current number of channels to detect changes

//==============DS3231 Related=========================
#define OUTPUT_MSG_BUFFER_SIZE       128U
#define ALARM_SYNC_DELAY_S           5
#define SYNC_TIMEOUT_MS              10000  // 10 second timeout

#define SECS_PER_MIN                 60
#define SECS_PER_HR                  (60 * SECS_PER_MIN)
#define SECS_PER_DAY                 (24 * SECS_PER_HR)

ds3231_driver_t DS3231_RTC;
static struct tm ds3231_datetime;
static struct tm_ms internal_RTC_datetime;
static char ds3231_datetime_str[17];
static char internal_RTC_datetime_str[20];
static float ds3231_temperature;
static uint8_t output_msgBuffer[OUTPUT_MSG_BUFFER_SIZE];
static volatile bool isAlarmTriggered = false;

// DS3231 interrupt configuration - must be static/global to persist
// Uses P1.22 which is connected to DS3231 INT/SQW pin
static mxc_gpio_cfg_t rtc_int_cfg = {
    .port = MXC_GPIO1,
    .mask = MXC_GPIO_PIN_22,
    .pad = MXC_GPIO_PAD_WEAK_PULL_UP,
    .func = MXC_GPIO_FUNC_IN,
    .vssel = MXC_GPIO_VSSEL_VDDIOH,
};

const struct tm ds3231_dateTimeDefault = {
	.tm_year = 118U,
	.tm_mon = 00U,
	.tm_mday = 1U,
	.tm_hour = 0U,
	.tm_min = 0U,
	.tm_sec = 0U
};

//=================================================

static bool isContinuousRecording = false; //Flag to indicate if the recording is continuous or not

// Non-blocking LED blink variables (reserved for future use)
// static uint32_t led_blink_start_time = 0;
// static bool led_blink_active = false;

/* Public function definitions ---------------------------------------------------------------------------------------*/
int main(void)
{
#ifdef TERMINAL_IO_USE_CONSOLE_UART
    bsp_console_uart_init();
#else
    bsp_console_uart_deinit();
#endif

    MXC_ICC_Enable();

    initialize_system();

    setup_user_pushbutton_interrupt();

    setup_realtimeclock();
    
    // Sync internal RTC to DS3231 at startup
    sync_RTC_to_DS3231();

    // Load schedule configuration from SD slot 5 (if available)
    load_schedule_from_slot5();

    printf("\n\n==================================\n");
    printf("   Cornell Lab of Ornithology     \n");
    printf("       K. Lisa Yang Center       \n");
    printf("   For Conservation Bioacoustics    \n");
    printf("==================================\n");
    fflush(stdout);
    printf(" __  __                   _      \n");
    printf("|  \\/  | __ _  __ _ _ __ (_) ___ \n");
    printf("| |\\/| |/ _` |/ _` | \'_ \\| |/ _ \\\n");
    printf("| |  | | (_| | (_| | |_) | |  __/\n");
    printf("|_|  |_|\\__,_|\\__, | .__/|_|\\___|\n");
    printf("              |___/|_|           \n\n");
    printf("==================================\n");
    fflush(stdout);

    MXC_Delay(MXC_DELAY_MSEC(500)); //Allow RTT to output and clear buffer data

    //Get Temperature from RTC
    if (E_NO_ERROR != DS3231_RTC.read_temperature(&ds3231_temperature)) {
        printf("\nDS3231 read temperature error\n");
    } else {
        sprintf((char*)output_msgBuffer, "\n-->Temperature (C): %.2f\r\n", (double)ds3231_temperature);
        printf((char*)output_msgBuffer);
    }
    
    // Display time with milliseconds for the first time
    print_time_with_milliseconds();

    printf("Build configuration check:\n");
#ifdef NATIVE_SDHC
    printf("NATIVE_SDHC is defined as: %d\n", NATIVE_SDHC);
#else
    printf("NATIVE_SDHC is NOT defined\n");
#endif
   
    printf("Standing by and waiting for a push from user button ...\n");

    // MXC_NVIC_SetVector(OST_TIMER_IRQn, OneshotTimerHandler);
    // NVIC_EnableIRQ(OST_TIMER_IRQn);
    // OneshotTimer();

    const u_int32_t idle_blink_interval = 20;  //Blink blue every 2 seconds, 100ms per loop
    uint32_t loopCount = 0;

    while (1)
    {
        //Idle blinking loop
         if(loopCount == idle_blink_interval)
        {
            loopCount = 0;
            status_led_set(STATUS_LED_COLOR_BLUE, TRUE);
            MXC_Delay(MXC_DELAY_MSEC(50));
            status_led_set(STATUS_LED_COLOR_BLUE, FALSE);
        }
        MXC_Delay(MXC_DELAY_MSEC(100));
        loopCount++;

        

        uint8_t button_state = get_user_pushbutton_state();

        if((BUTTON_STATE_JUST_PRESSED == button_state)||(BUTTON_STATE_PRESSED == button_state)||isContinuousRecording)
        //if (BUTTON_STATE_PRESSED == button_state)
        {
            // The first time user pushed the button, we set it to contiounous recording mode
            isContinuousRecording = true;   //this will get set to false if user interrupts the recording

            LED_cascade_left();
            printf("Start recording ...\n");

            // Remove callback for push button
            // MXC_GPIO_RegisterCallback(&bsp_pins_user_pushbutton_cfg, NULL, NULL);
            // MXC_GPIO_DisableInt(bsp_pins_user_pushbutton_cfg.port, bsp_pins_user_pushbutton_cfg.mask);
            // NVIC_DisableIRQ(GPIO0_IRQn);

            status_led_set(STATUS_LED_COLOR_GREEN, TRUE);
            MXC_Delay(MXC_DELAY_MSEC(20));
            status_led_set(STATUS_LED_COLOR_GREEN, FALSE);
                               

            //re-enable button (this is for checking a stop in the middle of recording)
            printf("[INFO]--> Re-enabling pushbutton (before recording)...\n");
            pushbuttons_init();
            setup_user_pushbutton_interrupt();

            //Get Date Time from RTC with milliseconds
            if(E_NO_ERROR != DS3231_RTC.read_datetime(&ds3231_datetime, ds3231_datetime_str))
            {
                sprintf(savedFileName,"%s",DEFAULT_FILENAME);
            } else {
                // Get milliseconds from internal RTC
   
                get_internal_RTC_time(&internal_RTC_datetime, internal_RTC_datetime_str);
                
                // Format: PROJECTID_SITEID_RECORDERID_YYYYMMDD_HHMMSS.fffZ (file will be saved in day folder)
                sprintf(savedFileName,"%s_%s_%s_%04d%02d%02d_%02d%02d%02d.%03dZ",
                    SYS_CONFIG_PROJECT_ID,
                    SYS_CONFIG_SITE_ID,
                    SYS_CONFIG_RECORDER_ID,
                    ds3231_datetime.tm_year + 1900,
                    ds3231_datetime.tm_mon + 1,
                    ds3231_datetime.tm_mday,
                    internal_RTC_datetime.tm_hour,
                    internal_RTC_datetime.tm_min,
                    internal_RTC_datetime.tm_sec,
                    internal_RTC_datetime.tm_subsec);
                printf("File to be saved: %s \n", savedFileName);
            }

            start_recording(SYS_CONFIG_NUM_CHANNEL, SYS_CONFIG_SAMPLE_RATE,
                            SYS_CONFIG_NUM_BIT_DEPTH,
                            SYS_CONFIG_SD_CARD_SLOT_TO_USE,
                            SYS_CONFIG_AUDIO_FILE_LEN_IN_SECONDS);
            
            
            printf("Recording Done ...\n");    
            
            //re-enable button (this is for the next recording)
            printf("[INFO]--> Re-enabling pushbutton (after recording)...\n");
            pushbuttons_init();
            setup_user_pushbutton_interrupt();

            printf("Standing by and waiting for a push from user button or a Continue Signal ...\n\n");
        }
    }
}

/* Private function definitions --------------------------------------------------------------------------------------*/

void write_wav_file(Wave_Header_Attributes_t *wav_attr, uint32_t file_len_secs)
{
    wav_header_clear_metadata();

    printf(
        "\n[STARTING]--> %dk %d-bit %d-second %d-channel recording...\n",
        wav_attr->sample_rate / 1000,
        wav_attr->bits_per_sample,
        file_len_secs,
        wav_attr->num_channels);

    // a variable to store the number of bytes written to the SD card, can be checked against the intended amount
    uint32_t bytes_written;

    // a string buffer to write file names into
    char file_name_buff[128];

    //Get Time from RTC with milliseconds again to get the most accurate time stamp

 
    if (get_internal_RTC_time(&internal_RTC_datetime, internal_RTC_datetime_str) != E_NO_ERROR)
    {
        // Error getting time - use default filename
        printf("[WARNING]--> Could not get internal RTC time\n");
    }
    
    // Format: ProjectID_3digitK_SiteID_RecorderID-bitb-CHnn_YYYYMMDD_HHMMSS.fffZ.wav
    // Example: S6741NY01_048K_S01_MAG000001-16b-CH01_20251126_150541.041Z.wav
    snprintf(file_name_buff, sizeof(file_name_buff), 
        "%s_%03luK_%s_%s-%lub-CH%02u_%04d%02d%02d_%02d%02d%02d.%03dZ.wav",
        SYS_CONFIG_PROJECT_ID,
        (unsigned long)(wav_attr->sample_rate / 1000),
        SYS_CONFIG_SITE_ID,
        SYS_CONFIG_RECORDER_ID,
        (unsigned long)wav_attr->bits_per_sample,
        (unsigned int)wav_attr->num_channels,
        ds3231_datetime.tm_year + 1900,
        ds3231_datetime.tm_mon + 1,
        ds3231_datetime.tm_mday,
        internal_RTC_datetime.tm_hour,
        internal_RTC_datetime.tm_min,
        internal_RTC_datetime.tm_sec,
        internal_RTC_datetime.tm_subsec);
    
    if (sd_card_fopen(file_name_buff, POSIX_FILE_MODE_WRITE) != E_NO_ERROR)
    {
        printf("[ERROR]--> SD card fopen\n");
        error_handler(STATUS_LED_COLOR_RED);
    }
    else
    {
        printf("[SUCCESS]--> SD card fopen\n");
    }

    // Write zeros to the header area to ensure no garbage data
    // This prevents undefined file content when using lseek
    {
        static uint8_t zero_buffer[512];
        memset(zero_buffer, 0, sizeof(zero_buffer));
        uint32_t header_len = wav_header_get_header_length();
        uint32_t bytes_written;
        for (uint32_t i = 0; i < header_len; i += sizeof(zero_buffer)) {
            uint32_t chunk_size = (header_len - i < sizeof(zero_buffer)) ? (header_len - i) : sizeof(zero_buffer);
            sd_card_fwrite(zero_buffer, chunk_size, &bytes_written);
        }
        printf("[SUCCESS]--> Initialized header area with zeros (%lu bytes)\n", (unsigned long)header_len);
    }
    // File pointer is now at the correct position after the header

    // there will be some integer truncation here, good enough for this early demo, but improve file-len code eventually
    const uint32_t file_len_in_microsecs = file_len_secs * 1000000;
    const uint32_t num_dma_blocks_in_the_file = file_len_in_microsecs / AUDIO_DMA_CHUNK_READY_PERIOD_IN_MICROSECS;

    // Reset decimation filter state to prevent garbage data at the beginning of recording
    decimation_filter_reset();
    decimation_filter_set_sample_rate(wav_attr->sample_rate);

// expanding to q31 means 4 bytes per sample, 2 channels
#define MAX_WORKSPACE_NEEDED_FOR_ENDIAN_SWAP_AND_Q31_EXPANSION (AUDIO_DMA_BUFF_LEN_IN_SAMPS * DATA_CONVERTERS_Q31_SIZE_IN_BYTES * 2)

// tradeoff between memory and current consumption, bigger buffers reduces current by doing fewer, larger, SD writes
#define FULL_BYTE_POOL_SIZE (MAX_WORKSPACE_NEEDED_FOR_ENDIAN_SWAP_AND_Q31_EXPANSION * 4)   //Orginally *3

    /*
     This is a single pool of memory which we use to do all the processing, we reserve a small potion at the start of
     the array for decimation for the non-384kHz rates. The rest of the pool is used to cram as many processed DMA
     blocks in as we can, and then write them all to the SD card once this pool is as full as it can get.

     For the 384k case, we just use the whole pool for chunk-packing with no reserved portion at the start, since we
     don't need to do any decimation filtering for 384k.
     
     Placed in SRAM6-7 to avoid overflowing SRAM0-3 which is used for stack/heap/general vars.
     */
    __attribute__((section(".sram6"))) static uint8_t workspace_byte_pool[FULL_BYTE_POOL_SIZE];
    
    // Clear workspace buffer to ensure no garbage data at the beginning
    memset(workspace_byte_pool, 0, sizeof(workspace_byte_pool));

    // 384kHz is treated differently to all the other sample rates, since it does not go through the decimation filter
    const bool its_the_special_case_of_384kHz = (wav_attr->sample_rate == AUDIO_SAMPLE_RATE_384kHz);

    // decimate by 2 for 192k, 4 for 96k, etc
    const uint32_t decimation_factor = AUDIO_SAMPLE_RATE_384kHz / wav_attr->sample_rate;

    // this represents the final sample width that is written to the SD card, not during intermediate steps
    const uint32_t bytes_per_processed_sample = (wav_attr->bits_per_sample / 8);

    // a single SD card write event will consist of one or more processed blocks, most of the time it will be several blocks
    const uint32_t num_bytes_in_one_processed_DMA_block = (AUDIO_DMA_BUFF_LEN_IN_SAMPS / decimation_factor) * bytes_per_processed_sample * wav_attr->num_channels;

    // we don't reserve any bytes in the 384k case, but we reserve a potion for the decimated rates to do the filtering
    const uint32_t num_bytes_reserved_for_decimation = its_the_special_case_of_384kHz ? 0 : (AUDIO_DMA_BUFF_LEN_IN_SAMPS * 4 * wav_attr->num_channels);

    // this portion represents everything except the bytes at the array start reserved for decimation, will be the whole pool for the 384k case
    const uint32_t num_bytes_reserved_for_chunk_packing = FULL_BYTE_POOL_SIZE - num_bytes_reserved_for_decimation;

    // this will have some integer truncation, so there may be some portion of the memory pool wasted for some combos
    const uint32_t num_processed_chunks_we_can_fit_in_the_workspace_buff = num_bytes_reserved_for_chunk_packing / num_bytes_in_one_processed_DMA_block;

    // we start the chunk-packing portion of the memory pool immediately after the portion reserved for decimation (if any, none for 384k)
    uint8_t *start_of_chunk_packing_sector = workspace_byte_pool + num_bytes_reserved_for_decimation;

    uint32_t chunk_packing_idx = 0;
    uint32_t total_num_bytes_in_the_buff = 0;

    printf("[Info]--> Start ADC clock and DMA. Recording for %d sec(s) ... \n", file_len_secs);

    status_led_set(STATUS_LED_COLOR_GREEN, true); // green led on


    ad4630_384kHz_fs_clk_and_cs_start();
    audio_dma_start();

    //status_led_set(STATUS_LED_COLOR_GREEN, true); // green led on while recording

    uint32_t num_dma_blocks_consumed = 0;

    while (num_dma_blocks_consumed < num_dma_blocks_in_the_file)
    {        
        // check if the user has pressed the button to stop recording
        uint8_t button_state = get_user_pushbutton_state();       
    

        if (audio_dma_overrun_occured(AUDIO_CHANNEL_0) || audio_dma_overrun_occured(AUDIO_CHANNEL_1))
        {
            printf("[ERROR]--> Audio DMA overrrun\n");
            wav_header_add_metadata("Error", "DMA Overrun");
            //error_handler(STATUS_LED_COLOR_BLUE);
              //error_handler(STATUS_LED_COLOR_BLUE);
            //num_dma_blocks_consumed = num_dma_blocks_in_the_file;  // stop recording this run and exit the loop
            //audio_dma_clear_overrun(AUDIO_CHANNEL_0);
            //audio_dma_clear_overrun(AUDIO_CHANNEL_1);
            //we'll start over with the net
            //break;
            audio_dma_clear_overrun(AUDIO_CHANNEL_0);
            audio_dma_clear_overrun(AUDIO_CHANNEL_1);
            continue;  //experiment to just continue
        }

        if (audio_dma_num_buffers_available(AUDIO_CHANNEL_0) > 0 && audio_dma_num_buffers_available(AUDIO_CHANNEL_1) > 0)
        {
            uint8_t *next_chunk_to_write_to = start_of_chunk_packing_sector + (num_bytes_in_one_processed_DMA_block * chunk_packing_idx);
            chunk_packing_idx++;

            if (wav_attr->num_channels == WAVE_HEADER_STEREO)
            {
                if (its_the_special_case_of_384kHz)
                {
                    if (wav_attr->bits_per_sample == AUDIO_BIT_DEPTH_24_BITS_PER_SAMPLE)
                    {
                        data_converters_interleave_2_i24_and_swap_endianness(
                            audio_dma_consume_buffer(AUDIO_CHANNEL_0),
                            audio_dma_consume_buffer(AUDIO_CHANNEL_1),
                            next_chunk_to_write_to,
                            AUDIO_DMA_BUFF_LEN_IN_SAMPS);
                    }
                    else // it's 384k 16 bits
                    {
                        data_converters_interleave_2_i24_to_q15_and_swap_endianness(
                            audio_dma_consume_buffer(AUDIO_CHANNEL_0),
                            audio_dma_consume_buffer(AUDIO_CHANNEL_1),
                            (q15_t *)next_chunk_to_write_to,
                            AUDIO_DMA_BUFF_LEN_IN_SAMPS);
                    }
                }
                else // it's not the special case of 384kHz, all other sample rates are filtered
                {
                    // we use some space at the start of the big workspace array for filtering
                    // since decimation works with q31's, we need to move the ch1 start up by nsamps * 4 bytes
                    const uint8_t *workspace_for_ch0 = workspace_byte_pool;
                    const uint8_t *workspace_for_ch1 = workspace_byte_pool + (AUDIO_DMA_BUFF_LEN_IN_SAMPS * 4);

                    data_converters_i24_to_q31_with_endian_swap(
                        audio_dma_consume_buffer(AUDIO_CHANNEL_0),
                        (q31_t *)workspace_for_ch0,
                        AUDIO_DMA_BUFF_LEN_IN_SAMPS);

                    data_converters_i24_to_q31_with_endian_swap(
                        audio_dma_consume_buffer(AUDIO_CHANNEL_1),
                        (q31_t *)workspace_for_ch1,
                        AUDIO_DMA_BUFF_LEN_IN_SAMPS);

                    const uint32_t decimated_len_in_samps = decimation_filter_downsample(
                        (q31_t *)workspace_for_ch0,
                        (q31_t *)workspace_for_ch0, // we're decimating "in place", this is ok because the decimated output will always be less than the original input
                        AUDIO_DMA_BUFF_LEN_IN_SAMPS,
                        AUDIO_CHANNEL_0);

                    decimation_filter_downsample( // the len will be the same, we already grabbed it from ch0 above
                        (q31_t *)workspace_for_ch1,
                        (q31_t *)workspace_for_ch1, //  decimating in-place
                        AUDIO_DMA_BUFF_LEN_IN_SAMPS,
                        AUDIO_CHANNEL_1);

                    if (wav_attr->bits_per_sample == AUDIO_BIT_DEPTH_24_BITS_PER_SAMPLE)
                    {
                        data_converters_interleave_2_q31_to_i24(
                            (q31_t *)workspace_for_ch0,
                            (q31_t *)workspace_for_ch1,
                            next_chunk_to_write_to,
                            decimated_len_in_samps);
                    }
                    else // it's one of the decimated sample rates at 16 bits
                    {
                        data_converters_interleave_2_q31_to_q15(
                            (q31_t *)workspace_for_ch0,
                            (q31_t *)workspace_for_ch1,
                            (q15_t *)next_chunk_to_write_to,
                            decimated_len_in_samps);
                    }
                }
            }
            else // num-channels must be MONO
            {
                // hack- consume the buffer we're not using so we don't get overruns, TODO change this
                audio_dma_consume_buffer(SYS_CONFIG_CHANNEL_TO_USE_FOR_MONO_MODE ? AUDIO_CHANNEL_0 : AUDIO_CHANNEL_1);

                if (its_the_special_case_of_384kHz)
                {
                    if (wav_attr->bits_per_sample == AUDIO_BIT_DEPTH_24_BITS_PER_SAMPLE)
                    {
                        data_converters_i24_swap_endianness(
                            audio_dma_consume_buffer(SYS_CONFIG_CHANNEL_TO_USE_FOR_MONO_MODE),
                            next_chunk_to_write_to,
                            AUDIO_DMA_BUFF_LEN_IN_SAMPS);
                    }
                    else // it's 384k 16 bits
                    {
                        data_converters_i24_to_q15_with_endian_swap(
                            audio_dma_consume_buffer(SYS_CONFIG_CHANNEL_TO_USE_FOR_MONO_MODE),
                            (q15_t *)next_chunk_to_write_to,
                            AUDIO_DMA_BUFF_LEN_IN_SAMPS);
                    }
                }
                else // it's not the special case of 384kHz, all other sample rates are filtered
                {
                    // all sample rates other than 384k are filtered, so we need to swap endianness and also expand to 32 bit words as expected by the filters
                    data_converters_i24_to_q31_with_endian_swap(
                        audio_dma_consume_buffer(SYS_CONFIG_CHANNEL_TO_USE_FOR_MONO_MODE),
                        (q31_t *)workspace_byte_pool,
                        AUDIO_DMA_BUFF_LEN_IN_SAMPS);

                    // workspace_buff starting at index zero now has the endian-swapped and q31-expanded data for this DMA chunk

                    const uint32_t decimated_len_in_samps = decimation_filter_downsample(
                        (q31_t *)workspace_byte_pool,
                        (q31_t *)workspace_byte_pool, // decimating in-place
                        AUDIO_DMA_BUFF_LEN_IN_SAMPS,
                        SYS_CONFIG_CHANNEL_TO_USE_FOR_MONO_MODE);

                    // workspace_buff starting at index zero now has the decimated version of this DMA chunk

                    if (wav_attr->bits_per_sample == AUDIO_BIT_DEPTH_24_BITS_PER_SAMPLE)
                    {
                        data_converters_q31_to_i24((q31_t *)workspace_byte_pool, next_chunk_to_write_to, decimated_len_in_samps);
                    }
                    else // it's one of the decimated sample rates at 16 bits
                    {
                        data_converters_q31_to_q15((q31_t *)workspace_byte_pool, (q15_t *)next_chunk_to_write_to, decimated_len_in_samps);
                    }
                }
            }

            total_num_bytes_in_the_buff += num_bytes_in_one_processed_DMA_block;

            if (chunk_packing_idx >= num_processed_chunks_we_can_fit_in_the_workspace_buff) // we filled up the chunk-packing sector as much as we can
            {
                if (sd_card_fwrite(start_of_chunk_packing_sector, total_num_bytes_in_the_buff, &bytes_written) != E_NO_ERROR)
                {
                    error_handler(STATUS_LED_COLOR_RED);
                }

                chunk_packing_idx = 0;
                total_num_bytes_in_the_buff = 0;
            }

            num_dma_blocks_consumed += 1;

            //Blink Green LED while recording (non-blocking)
            static uint32_t led_blink_counter = 0;
            static bool led_is_on = false;
            
            if(num_dma_blocks_consumed % 120 == 0 && !led_is_on)
            {                
                status_led_set(STATUS_LED_COLOR_GREEN, TRUE);
                led_is_on = true;
                led_blink_counter = 0;
            }
            
            if(led_is_on)
            {
                led_blink_counter++;
                // Turn off after processing a few more blocks (~25ms worth)
                if(led_blink_counter >= 3) // Adjust this value as needed
                {
                    status_led_set(STATUS_LED_COLOR_GREEN, FALSE);
                    led_is_on = false;
                }
            }
        }
        
        if((BUTTON_STATE_JUST_PRESSED == button_state)||(BUTTON_STATE_PRESSED == button_state))
        {
            num_dma_blocks_consumed = num_dma_blocks_in_the_file; // stop recording if the button is pressed
            isContinuousRecording = false; // set the flag to false so that we don't continue recording next time
            
            printf("[INFO]--> User interrupted recording.  End recording ....\n");
            status_led_set(STATUS_LED_COLOR_BLUE, TRUE); // turn the green LED back on
            // MXC_Delay(MXC_DELAY_MSEC(250));
            // status_led_set(STATUS_LED_COLOR_BLUE, FALSE);

        }
    }

    // there may be some data left in the chunk-packing buffer, if we don't write this out the file will be a little shorter than expected
    if (total_num_bytes_in_the_buff > 0)
    {
        if (sd_card_fwrite(start_of_chunk_packing_sector, total_num_bytes_in_the_buff, &bytes_written) != E_NO_ERROR)
        {
            error_handler(STATUS_LED_COLOR_RED);
        }
    }

   
    char tempWAVBuffer[50];
    //================ Get time stamp and temperature ===========    

    //Get Time Stamp from RTC
    if (E_NO_ERROR != DS3231_RTC.read_datetime(&ds3231_datetime, ds3231_datetime_str)) 
    {
        snprintf(tempWAVBuffer, sizeof(tempWAVBuffer), "DS3231 Error");
    } else {
        strftime(tempWAVBuffer, OUTPUT_MSG_BUFFER_SIZE, "%Y%m%d_%H%M%SZ", &ds3231_datetime);
    }
    wav_header_add_metadata("Log Time", tempWAVBuffer);

    //Get Temperature from RTC
    if (E_NO_ERROR != DS3231_RTC.read_temperature(&ds3231_temperature)) 
    {
        snprintf(tempWAVBuffer, sizeof(tempWAVBuffer), "DS3231 Error");
    } else {
        sprintf(tempWAVBuffer, "%.2f", (double)ds3231_temperature);
    }
    wav_header_add_metadata("Temperature(C)", tempWAVBuffer);

     //================ Get Fuel Gauge data while ADC and DMA still powered =======

    fuel_gauge_data_t fg_metadata = Fuel_gauge_data_collect("Recording");
    
    sprintf(tempWAVBuffer, "%.2f", (double)fg_metadata.pack_voltage);  //There's a 0.4V drop due to the protection circuit
    wav_header_add_metadata("Recording Voltage(V)", tempWAVBuffer);
    
    sprintf(tempWAVBuffer, "%.2f", (double)fg_metadata.current_ma);  
    wav_header_add_metadata("Recording Current(mA)", tempWAVBuffer);

    sprintf(tempWAVBuffer, "%.2f", (double)fg_metadata.power_mw);  //2S
    wav_header_add_metadata("Recording Power(mW)", tempWAVBuffer);

    sprintf(tempWAVBuffer, "%.2f", (double)fg_metadata.avg_pack_voltage);  //2S
    wav_header_add_metadata("R Avg. Voltage(V)", tempWAVBuffer);
    
    sprintf(tempWAVBuffer, "%.2f", (double)fg_metadata.avg_current_ma);  
    wav_header_add_metadata("R Avg. Current(mA)", tempWAVBuffer);

    sprintf(tempWAVBuffer, "%.2f", (double)fg_metadata.avg_power_mw);  //2S
    wav_header_add_metadata("R Avg. Power(mW)", tempWAVBuffer);

    sprintf(tempWAVBuffer, "%.2f", (double)fg_metadata.temperature_c);  
    wav_header_add_metadata("FG Temperature(C)", tempWAVBuffer);

    sprintf(tempWAVBuffer, "%.2f", (double)fg_metadata.avg_temperature_c);  
    wav_header_add_metadata("FG Avg. Temperature(C)", tempWAVBuffer);

    //=================== Get Environmental Sensor data ========================
    bme688_data_t sensor_data = bme688_read_all_data("Recording");
        
    if (sensor_data.valid_data) 
    {
        double sensor_temperature = (double)sensor_data.temperature_c;
        double sensor_pressure_pa = (double)sensor_data.pressure_pa; // Get pressure in Pascals
        double sensor_pressure_psi = sensor_pressure_pa * 0.000145038; // Convert Pa to PSI (1 Pa = 0.000145038 PSI)
        double sensor_pressure_atm = sensor_pressure_pa / 101325.0; // Convert Pa to ATM (1 ATM = 101325 Pa)
        double sensor_humidity = (double)sensor_data.humidity_percent;
        double sensor_gas_resistance = (double)sensor_data.gas_resistance_ohm;
        
        sprintf(tempWAVBuffer, "%.2f", sensor_temperature);
        wav_header_add_metadata("Sensor T(C)", tempWAVBuffer);

        sprintf(tempWAVBuffer, "%.3f", sensor_pressure_psi);
        wav_header_add_metadata("Sensor P(psi)", tempWAVBuffer);

        sprintf(tempWAVBuffer, "%.4f", sensor_pressure_atm);
        wav_header_add_metadata("Sensor P(atm)", tempWAVBuffer);

        sprintf(tempWAVBuffer, "%.2f", sensor_humidity);
        wav_header_add_metadata("Sensor H(%RH)", tempWAVBuffer);

        sprintf(tempWAVBuffer, "%.0f", sensor_gas_resistance);
        wav_header_add_metadata("Sensor Gas R(ohms)", tempWAVBuffer);
    }

    //====== Stop ADC and DMA =============

    ad4630_384kHz_fs_clk_and_cs_stop();
    audio_dma_stop();

    // back to the top of the file so we can write the wav header now that we can determine the size of the file
    if (sd_card_lseek(0) != E_NO_ERROR)
    {
        printf("[ERROR]--> SD card lseek to top of file\n");
        error_handler(STATUS_LED_COLOR_RED);
    }
    else
    {
        printf("[SUCCESS]--> SD card lseek to top of file\n");
    }

    wav_attr->file_length = sd_card_fsize();
    wav_header_set_attributes(wav_attr);

         
    // //================ Get Fuel Gauge data after ADC and DMA stopped  =======
    // fg_metadata = Fuel_gauge_data_collect("Stopping");
    
    // sprintf(tempWAVBuffer, "%.2f", fg_metadata.vcell_voltage*2);  
    // wav_header_add_metadata("Stopping Voltage(V)", tempWAVBuffer);
    
    // sprintf(tempWAVBuffer, "%.2f", fg_metadata.current_ma);  
    // wav_header_add_metadata("Stopping Current(mA)", tempWAVBuffer);

    // sprintf(tempWAVBuffer, "%.2f", fg_metadata.avg_vcell_voltage*2);  
    // wav_header_add_metadata("S Avg. Voltage(V)", tempWAVBuffer);
    
    // sprintf(tempWAVBuffer, "%.2f", fg_metadata.avg_current_ma);  
    // wav_header_add_metadata("S Avg. Current(mA)", tempWAVBuffer);
    

    if (sd_card_fwrite(wav_header_get_header(), wav_header_get_header_length(), &bytes_written) != E_NO_ERROR)
    {
        printf("[ERROR]--> SD card WAV header fwrite\n");
        error_handler(STATUS_LED_COLOR_RED);
    }
    else
    {
        printf("[SUCCESS]--> SD card WAV header fwrite\n");
    }

    if (sd_card_fclose() != E_NO_ERROR)
    {
        printf("[ERROR]--> SD card fclose\n");
        error_handler(STATUS_LED_COLOR_RED);
    }
    else
    {
        printf("[SUCCESS]--> SD card fclose\n");

        //Generate time stamp
	    set_file_timestamp(file_name_buff, ds3231_datetime.tm_year + 1900, 
        ds3231_datetime.tm_mon + 1, 
        ds3231_datetime.tm_mday, 
        ds3231_datetime.tm_hour,
        ds3231_datetime.tm_min, 
        ds3231_datetime.tm_sec);
    }

    printf("[SUCCESS]--> Wrote file %s\n", file_name_buff);

    status_led_set(STATUS_LED_COLOR_GREEN, false); // green led off after recording is complete
}

void error_handler(Status_LED_Color_t color)
{
    status_led_all_off();

    const uint32_t fast_blink = 100000;
    while (true)
    {
        status_led_toggle(color);
        MXC_Delay(fast_blink);
    }
}


static void setup_user_pushbutton_interrupt(void)
{
    printf("[INFO]--> Setting up pushbutton interrupt handler...\n");
    
    // Configure interrupt
    MXC_GPIO_RegisterCallback(&bsp_pins_user_pushbutton_cfg, user_pushbutton_interrupt_callback, NULL);
    printf("[INFO]-->   Callback registered\n");

    // Configure for falling edge detection (trigger on button press)
    MXC_GPIO_IntConfig(&bsp_pins_user_pushbutton_cfg, MXC_GPIO_INT_FALLING);
    printf("[INFO]-->   Interrupt configured for falling edge\n");

    // Enable interrupt
    MXC_GPIO_EnableInt(bsp_pins_user_pushbutton_cfg.port, bsp_pins_user_pushbutton_cfg.mask);
    printf("[INFO]-->   GPIO interrupt enabled (port=%p, mask=0x%08X)\n", 
           (void*)bsp_pins_user_pushbutton_cfg.port, bsp_pins_user_pushbutton_cfg.mask);
    
    // Enable NVIC interrupt for the GPIO port (shared with DS3231)
    // Use the proper method to get the IRQ number for the GPIO port
    IRQn_Type irq = MXC_GPIO_GET_IRQ(MXC_GPIO_GET_IDX(bsp_pins_user_pushbutton_cfg.port));
    NVIC_EnableIRQ(irq);
    printf("[INFO]-->   NVIC IRQ %d enabled\n", (int)irq);
    
    printf("[INFO]--> Pushbutton interrupt handler ACTIVE\n");
}


static void LED_cascade_right(void) //R->G->B
{
    status_led_all_off();
    for (size_t i = 0; i < 3; i++)
    {
        //Turn each color LED on
        status_led_set(i,TRUE);
        MXC_Delay(MXC_DELAY_MSEC(100));
    }
    MXC_Delay(MXC_DELAY_MSEC(100));
    status_led_all_off();
}

static void LED_cascade_left(void) //R<-G<-B
{
    status_led_all_off();
    for (size_t i = 3; i > 0; i--)
    {
        //Turn each color LED on
        status_led_set(i-1,TRUE);
        MXC_Delay(MXC_DELAY_MSEC(100));
    }
    MXC_Delay(MXC_DELAY_MSEC(100));
    status_led_all_off();
}

static void initialize_system(void)
{
    printf("\n*********** Initializing Magpie Core Systems ***********\n\n");
    status_led_all_off();    
    
    //initialize push button
    printf("Enabling push buttons ...\n");
    pushbuttons_init();
    printf("Enabling LDOs ...\n");
    bsp_power_on_LDOs();
    MXC_Delay(MXC_DELAY_MSEC(100)); // Allow LDOs to stabilize and devices to power up

    LED_cascade_right();
    LED_cascade_left();

    if (ad4630_init() != E_NO_ERROR)
    {
        printf("[ERROR]--> AD4630 init\n");
        error_handler(STATUS_LED_COLOR_RED);
    }
    else
    {
        printf("[SUCCESS]--> AD4630 init\n");
    }

    if (audio_dma_init() != E_NO_ERROR)
    {
        printf("[ERROR]--> DMA init\n");
        error_handler(STATUS_LED_COLOR_RED);
    }
    else
    {
        printf("[SUCCESS]--> DMA init\n");
    }

    if (bsp_3v3_i2c_init() != E_NO_ERROR)   //This allows I2C connecting to DS3231, SD, SD_CTL
    {
        printf("[ERROR]--> 3V3 I2C init\n");
        error_handler(STATUS_LED_COLOR_RED);
    }
    else
    {
        printf("[SUCCESS]--> 3V3 I2C init\n");
    }

    if (bsp_1v8_i2c_init() != E_NO_ERROR)  //This allows I2C connecting to Bosch Sensor, Fuel Gauge, LDO, AFE
    {
        printf("[ERROR]--> 1V8 I2C init\n");
        error_handler(STATUS_LED_COLOR_RED);
    }
    else
    {
        printf("[SUCCESS]--> 1V8 I2C init\n");
    }  

    MXC_Delay(MXC_DELAY_MSEC(100)); // Allow I2C devices to stabilize after bus init

    //////////////////// Fuel Gauge INIT ////////////////

    max17261_soft_reset(); // perform soft reset
    MXC_Delay(250);
    if (max17261_por_detected())
    {   // load max17261 configuration
        if (!max17261_wait_dnr(0)) {
            printf("[ERROR]--> DNR wait timeout POR config.\r\n");
        }
        else
        {
            printf("[SUCCESS]--> Fuel Gauge Started\n");
        }
        if (!max17261_config_ez(0)) {
            printf("[ERROR]--> EZCfg timeout POR config.\r\n");
        }
        else
        {
            printf("[SUCCESS]--> EZ Config Loaded\n");
        }
        max17261_clear_por_bit();
    }
    else
    {
        printf("[INFO]--> POR not detected, Re-config\r\n");

        // Force reconfiguration even without POR
        if (!max17261_wait_dnr(0)) {
            printf("[ERROR]--> DNR wait timeout POR config.\r\n");
        }
        else
        {
            printf("[SUCCESS]--> Fuel Gauge Started\n");
        }
        if (!max17261_config_ez(0)) {
            printf("[ERROR]--> EZCfg timeout POR config.\r\n");
        }
        else
        {
            printf("[SUCCESS]--> EZ Config Loaded\n");
        }
    }

    // Force reconfiguration if DesignCap is wrong.  
    // This is often the case when the battery is changed or the pack is restarted
    uint16_t current_designcap = max17261_read_designcap();
    
    if (current_designcap != 0x1950) {
        printf("[INFO]--> Incorrect DesignCap (0x%04X)\n", current_designcap);
        printf("[INFO]--> Re-configuring DesignCap ...\n");
        max17261_soft_reset();
        MXC_Delay(500000); // 500ms delay
         if (!max17261_wait_dnr(0)) {
            printf("[ERROR]--> DNR wait timeout POR config.\r\n");
        }
        else
        {
            printf("[SUCCESS]--> Fuel Gauge Started\n");
        }
        if (!max17261_config_ez(0)) {
            printf("[ERROR]--> EZCfg timeout POR config.\r\n");
        }
        else
        {
            printf("[SUCCESS]--> EZ Config Loaded\n");
        }
        
        // // Reset QH register for fresh start with new capacity
        // max17261_reset_qh();        
        // printf("Reconfiguration and QH reset complete\n");
    }

    MXC_Delay(MXC_DELAY_MSEC(5)); //Allow RTT to output and clear buffer data
    fflush(stdout);

    ////////////////// Environmental Sensor ////////////
     // Initialize BME688 sensor
    if (E_NO_ERROR != bme688_init()) {
        printf("[ERROR]--> BME688 init failed\n");
    }
    else
    {
    printf("[SUCCESS] --> BME688 initialized\n");
    }

    // // Brief delay after initialization
    // MXC_Delay(100000);


    ////////////////////  AUDIO INIT ////////////////////

    if (afe_control_init() != E_NO_ERROR)
    {
        printf("[ERROR]--> AFE Control init\n");
        error_handler(STATUS_LED_COLOR_RED);
    }
    else
    {
        printf("[SUCCESS]--> AFE Control init\n");
    }
    if (afe_control_enable(AUDIO_CHANNEL_0) != E_NO_ERROR)
    {
        printf("[ERROR]--> AFE Control CH0 EN\n");
        error_handler(STATUS_LED_COLOR_RED);
    }
    else
    {
        printf("[SUCCESS]--> AFE Control CH0 EN\n");
    }
    if (afe_control_enable(AUDIO_CHANNEL_1) != E_NO_ERROR)
    {
        printf("[ERROR]--> AFE Control CH1 EN\n");
        error_handler(STATUS_LED_COLOR_RED);
    }
    else
    {
        printf("[SUCCESS]--> AFE Control CH1 EN\n");
    }
    if (afe_control_set_gain(AUDIO_CHANNEL_0, SYS_CONFIG_AUDIO0_GAIN) != E_NO_ERROR)
    {
        printf("[ERROR]--> AFE Control CH0 gain set to %ddB\n", SYS_CONFIG_AUDIO0_GAIN);
        error_handler(STATUS_LED_COLOR_RED);
    }
    else
    {
        printf("[SUCCESS]--> AFE Control CH0 gain set to %ddB\n", SYS_CONFIG_AUDIO0_GAIN);
    }
    if (afe_control_set_gain(AUDIO_CHANNEL_1, SYS_CONFIG_AUDIO1_GAIN) != E_NO_ERROR)
    {
        printf("[ERROR]--> AFE Control CH1 gain set to %ddB\n", SYS_CONFIG_AUDIO1_GAIN);
        error_handler(STATUS_LED_COLOR_RED);
    }
    else
    {
        printf("[SUCCESS]--> AFE Control CH1 gain set to %ddB\n", SYS_CONFIG_AUDIO1_GAIN);
    }

    Audio_Gain_t readback_gain = afe_control_get_gain(AUDIO_CHANNEL_0);
    if (readback_gain != SYS_CONFIG_AUDIO0_GAIN)
    {
        printf("[ERROR]--> AFE CH0 set (%ddB) and get (%ddB) gain don't match\n", SYS_CONFIG_AUDIO0_GAIN, readback_gain);
        error_handler(STATUS_LED_COLOR_RED);
    }
    else
    {
        printf("[SUCCESS]--> AFE CH0 get-gain matches AFE set-gain\n");
    }
    readback_gain = afe_control_get_gain(AUDIO_CHANNEL_1);
    if (readback_gain != SYS_CONFIG_AUDIO1_GAIN)
    {
        printf("[ERROR]--> AFE CH1 set (%ddB) and get (%ddB) gain don't match\n", SYS_CONFIG_AUDIO1_GAIN, readback_gain);
        error_handler(STATUS_LED_COLOR_RED);
    }
    else
    {
        printf("[SUCCESS]--> AFE CH1 get-gain matches AFE set-gain\n");
    }

    //////////////// SD CARD Bank Control INIT /////////////////////

    if (sd_card_bank_ctl_init() != E_NO_ERROR)
    {
        printf("[ERROR]--> SD card bank ctl init\n");
        error_handler(STATUS_LED_COLOR_GREEN);
    }
    else
    {
        printf("[SUCCESS]--> SD card bank ctl init\n");
    }
   
}

static void setup_realtimeclock()
{
    DS3231_RTC = DS3231_Open();

    if(E_NO_ERROR != DS3231_RTC.init(bsp_i2c_3v3_i2c_handle))
    {
      printf("[ERROR] --> Unable to initialize RTC driver.\n");
      error_handler(STATUS_LED_COLOR_RED);
    }

    //only do this if the clock has never been set before
    #ifdef FIRST_SET_RTC
	// //Set Date Time to something
	//Year is always Year - 1900
	//Month is 0-11 so subtract 1 from the month you want to set
	//Time is in UTC so set appropriately
	// hour is 0-23
	// min is 0-59
	// sec is 0-59
	struct tm newTime = {
		.tm_year = 2025 - 1900U,
		.tm_mon =  11 - 1U,
		.tm_mday = 2,
		.tm_hour = 14,
		.tm_min = 13,
		.tm_sec = 0
	};

	
	//Set Date Time on RTC. 
	
	if (E_NO_ERROR != DS3231_RTC.set_datetime(&newTime)) {
		printf("\nDS3231 set time error\n");
	} else {
		strftime((char*)output_msgBuffer, OUTPUT_MSG_BUFFER_SIZE, "\n-->Set DateTime: %F %TZ\r\n", &newTime);
		printf(output_msgBuffer);
	}
	#endif
}

__attribute__((unused))
static void power_on_audio_chain(void)
{
    bsp_power_on_LDOs();            
}

__attribute__((unused))
static void power_off_audio_chain(void)
{

    // if (afe_control_disable(AUDIO_CHANNEL_0) != E_NO_ERROR)
    // {
    //     printf("[ERROR]--> AFE Control CH0 DIS\n");
    //     error_handler(STATUS_LED_COLOR_RED);
    // }
    // else
    // {
    //     printf("[SUCCESS]--> AFE Control CH0 DIS\n");
    // }

    // if (afe_control_disable(AUDIO_CHANNEL_1) != E_NO_ERROR)
    // {
    //     printf("[ERROR]--> AFE Control CH1 DIS\n");
    //     error_handler(STATUS_LED_COLOR_RED);
    // }
    // else
    // {
    //     printf("[SUCCESS]--> AFE Control CH0 DIS\n");
    // }

    bsp_power_off_LDOs();
}

/**
 * @brief Calculate the expected WAV file size in bytes
 * @param sample_rate Sample rate in Hz
 * @param bits_per_sample Bit depth (16 or 24)
 * @param num_channels Number of channels (1 for mono, 2 for stereo)
 * @param duration_s Duration in seconds
 * @return Expected file size in bytes (including WAV header overhead)
 */
static uint64_t calculate_wav_file_size(uint32_t sample_rate, uint32_t bits_per_sample, uint32_t num_channels, uint32_t duration_s)
{
    // Audio data size = sample_rate * (bits_per_sample / 8) * num_channels * duration
    uint64_t audio_data_size = (uint64_t)sample_rate * (bits_per_sample / 8) * num_channels * duration_s;
    
    // Add WAV header size (typically ~44 bytes, but use larger estimate for metadata)
    uint64_t header_overhead = 1024;  // Conservative estimate for header + metadata
    
    return audio_data_size + header_overhead;
}

/**
 * @brief Convert slot enum to human-readable slot number (0-5)
 * @note The enum values are reversed: SLOT_0=5, SLOT_1=4, ..., SLOT_5=0
 */
static int slot_enum_to_number(SD_Card_Bank_Card_Slot_t slot)
{
    // SLOT_0=5, SLOT_1=4, SLOT_2=3, SLOT_3=2, SLOT_4=1, SLOT_5=0
    return 5 - (int)slot;
}

/**
 * @brief Convert slot number (0-5) to slot enum
 */
static SD_Card_Bank_Card_Slot_t slot_number_to_enum(int slot_num)
{
    // slot_num 0 -> enum 5 (SLOT_0), slot_num 1 -> enum 4 (SLOT_1), etc.
    return (SD_Card_Bank_Card_Slot_t)(5 - slot_num);
}

/**
 * @brief Find an available SD card slot with enough free space
 * @param required_bytes Minimum free space required in bytes
 * @return The slot enum to use, or SD_CARD_BANK_ALL_SLOTS_DISABLED if no suitable slot found
 * 
 * @note Searches from currentMinSlotNumber to slot 5 (all slots available for recording)
 *       Skips slots with no card inserted or insufficient space
 *       Once a slot is skipped due to being full, we never go back to earlier slots
 */
static SD_Card_Bank_Card_Slot_t find_available_sd_slot(uint32_t required_bytes)
{
    printf("\n[INFO]--> Searching for SD card with at least %lu bytes free...\n", (unsigned long)required_bytes);
    printf("[INFO]--> Starting search from slot %d (slots 0-%d already exhausted)\n", 
           currentMinSlotNumber, currentMinSlotNumber > 0 ? currentMinSlotNumber - 1 : -1);
    
    // Search slots from currentMinSlotNumber through 5 (all slots available)
    // Enum values: SLOT_0=5, SLOT_1=4, SLOT_2=3, SLOT_3=2, SLOT_4=1, SLOT_5=0
    
    for (int slot_num = currentMinSlotNumber; slot_num <= 5; slot_num++)
    {
        SD_Card_Bank_Card_Slot_t slot = slot_number_to_enum(slot_num);
        
        printf("[INFO]--> Checking slot %d (enum=%d)...\n", slot_num, (int)slot);
        
        // Enable the slot
        if (sd_card_bank_ctl_enable_slot(slot) != E_NO_ERROR)
        {
            printf("[INFO]--> Could not enable slot %d, skipping\n", slot_num);
            // Don't advance currentMinSlotNumber for enable failures (might be temporary)
            continue;
        }
        
        // Check if card is inserted
        sd_card_bank_ctl_read_and_cache_detect_pins();
        if (!sd_card_bank_ctl_active_card_is_inserted())
        {
            printf("[INFO]--> No card in slot %d, advancing to next slot\n", slot_num);
            sd_card_bank_ctl_disable_all();
            // Advance minimum slot since card is not present
            if (slot_num >= currentMinSlotNumber)
            {
                currentMinSlotNumber = slot_num + 1;
                printf("[INFO]--> Updated minimum slot to %d (slot %d has no card)\n", currentMinSlotNumber, slot_num);
            }
            continue;
        }
        
        // Try to initialize the card
        if (sd_card_init() != E_NO_ERROR)
        {
            printf("[INFO]--> Could not init card in slot %d, advancing to next slot\n", slot_num);
            sd_card_bank_ctl_disable_all();
            // Advance minimum slot since card failed to init
            if (slot_num >= currentMinSlotNumber)
            {
                currentMinSlotNumber = slot_num + 1;
                printf("[INFO]--> Updated minimum slot to %d (slot %d init failed)\n", currentMinSlotNumber, slot_num);
            }
            continue;
        }
        
        MXC_Delay(100000);  // Brief delay before mount
        
        // Try to mount
        if (sd_card_mount() != E_NO_ERROR)
        {
            printf("[INFO]--> Could not mount card in slot %d, advancing to next slot\n", slot_num);
            sd_card_bank_ctl_disable_all();
            // Advance minimum slot since card failed to mount
            if (slot_num >= currentMinSlotNumber)
            {
                currentMinSlotNumber = slot_num + 1;
                printf("[INFO]--> Updated minimum slot to %d (slot %d mount failed)\n", currentMinSlotNumber, slot_num);
            }
            continue;
        }
        
        // Check free space
        QWORD free_space = sd_card_free_space_bytes();
        printf("[INFO]--> Slot %d free space: %llu bytes\n", slot_num, free_space);
        
        if (free_space >= required_bytes)
        {
            printf("[SUCCESS]--> Slot %d has sufficient space\n", slot_num);
            // Leave card mounted and return this slot
            return slot;
        }
        
        printf("[INFO]--> Slot %d has insufficient space (%llu < %lu), skipping\n", 
               slot_num, free_space, (unsigned long)required_bytes);
        
        // Update minimum slot to skip this full slot in future searches
        if (slot_num >= currentMinSlotNumber)
        {
            currentMinSlotNumber = slot_num + 1;
            printf("[INFO]--> Updated minimum slot to %d (slot %d is full)\n", currentMinSlotNumber, slot_num);
        }
        
        // Unmount and disable before trying next slot
        sd_card_unmount();
        sd_card_bank_ctl_disable_all();
    }
    
    printf("[ERROR]--> No suitable SD card slot found! All slots 0-5 exhausted.\n");
    return SD_CARD_BANK_ALL_SLOTS_DISABLED;
}

static void start_recording(uint8_t number_of_channel,
                            Audio_Sample_Rate_t rate, Audio_Bits_Per_Sample_t bit,
                            SD_Card_Bank_Card_Slot_t sd_slot, int32_t duration_s)
{
    // Calculate required file size
    uint64_t required_size = calculate_wav_file_size(rate, bit, number_of_channel, duration_s);
    printf("[INFO]--> Estimated file size: %llu bytes (%.2f MB)\n", 
           (unsigned long long)required_size, (double)required_size / (1024.0 * 1024.0));
    
    // Find a suitable SD card slot with enough space
    // Start from the configured slot and search forward if needed
    SD_Card_Bank_Card_Slot_t selected_slot = find_available_sd_slot((uint32_t)required_size);
    
    if (selected_slot == SD_CARD_BANK_ALL_SLOTS_DISABLED)
    {
        printf("[ERROR]--> No SD card with sufficient space found!\n");
        error_handler(STATUS_LED_COLOR_RED);
        return;
    }
    
    int selected_slot_num = slot_enum_to_number(selected_slot);
    printf("[SUCCESS]--> Using SD card slot %d for recording\n", selected_slot_num);
    
    // Check if we switched to a different SD card slot
    bool slotChanged = (lastUsedSlotNumber != selected_slot_num);
    if (slotChanged && lastUsedSlotNumber >= 0)
    {
        printf("[INFO]--> Slot changed from %d to %d, resetting folder tracking\n", 
               lastUsedSlotNumber, selected_slot_num);
        // Reset folder tracking since we're on a different card
        currentSessionFolder[0] = '\0';
        currentDayFolder[0] = '\0';
        currentDayOfYear = -1;
        currentSampleRateKHz = 0;
        currentBitDepth = 0;
        currentNumChannels = 0;
    }
    lastUsedSlotNumber = selected_slot_num;
    
    // Card is already mounted from find_available_sd_slot()
    QWORD disk_size = sd_card_disk_size_bytes();
    QWORD disk_free = sd_card_free_space_bytes();
    printf("[INFO]--> SD card Total Space: %llu Bytes\n", disk_size);
    printf("[INFO]--> SD card Free Space: %llu Bytes\n", disk_free);

    // Setup session folder on first recording after boot OR when slot changed
    if (isFirstRecordingAfterBoot || slotChanged)
    {
        if (setup_session_folder() != E_NO_ERROR)
        {
            printf("[ERROR]--> Failed to setup session folder\n");
            error_handler(STATUS_LED_COLOR_RED);
        }
        isFirstRecordingAfterBoot = false;
    }

    Wave_Header_Attributes_t wav_attr = {
        .num_channels = number_of_channel,
        .sample_rate = rate,
        .bits_per_sample = bit,
    };

    // Setup day folder based on current date, sample rate, bit depth, and channels
    if (setup_day_folder(ds3231_datetime.tm_year + 1900,
                         ds3231_datetime.tm_mon + 1,
                         ds3231_datetime.tm_mday,
                         wav_attr.sample_rate,
                         wav_attr.bits_per_sample,
                         wav_attr.num_channels) != E_NO_ERROR)
    {
        printf("[ERROR]--> Failed to setup day folder\n");
        error_handler(STATUS_LED_COLOR_RED);
    }

    write_wav_file(&wav_attr, duration_s);

    if (sd_card_unmount() != E_NO_ERROR)
    {
        printf("[ERROR]--> SD card unmount\n");
        error_handler(STATUS_LED_COLOR_RED);
    }
    else
    {
        printf("[SUCCESS]--> SD card unmounted\n");
    }
    
    // Disable the SD card slot after recording
    sd_card_bank_ctl_disable_all();

    printf("[SUCCESS]--> Recording complete on slot %d\n", selected_slot_num);
}

///////////////////////////RTC SYNC FUNCTIONS ////////////////////////////////////

static void sync_RTC_to_DS3231(void)
{    
    printf("Syncing internal RTC with DS3231 using alarm method...\n");
    
    // Reset the alarm triggered flag
    isAlarmTriggered = false;
    
    // Get current time from DS3231 first
    if (E_NO_ERROR != DS3231_RTC.read_datetime(&ds3231_datetime, ds3231_datetime_str)) {
        printf("DS3231 read datetime error during sync\n");
        return;
    } else {
        strftime((char*)output_msgBuffer, OUTPUT_MSG_BUFFER_SIZE, "DS3231 DateTime: %F %TZ\n", &ds3231_datetime);
        printf((char*)output_msgBuffer);
    }

    //Configure Interrupt pin for RTC
    MXC_GPIO_Config(&bsp_pins_rtc_int_cfg);

    // Set DS3231 alarm ALARM_SYNC_DELAY_S seconds from now
    struct tm alarmTime = {
        .tm_year = ds3231_datetime.tm_year,
        .tm_mon = ds3231_datetime.tm_mon,
        .tm_mday = ds3231_datetime.tm_mday,
        .tm_hour = ds3231_datetime.tm_hour,
        .tm_min = ds3231_datetime.tm_min,
        .tm_sec = ds3231_datetime.tm_sec + ALARM_SYNC_DELAY_S
    };
    
    // Handle minute/hour overflow
    if (alarmTime.tm_sec >= 60) {
        alarmTime.tm_sec -= 60;
        alarmTime.tm_min++;
        if (alarmTime.tm_min >= 60) {
            alarmTime.tm_min -= 60;
            alarmTime.tm_hour++;
            if (alarmTime.tm_hour >= 24) {
                alarmTime.tm_hour -= 24;
                alarmTime.tm_mday++;
            }
        }
    }
    
    // Clear any existing DS3231 interrupt flags before setting alarm
    printf("Clearing DS3231 interrupt flags...\n");
    
    if (E_NO_ERROR != DS3231_RTC.set_alarm(&alarmTime)) {
        printf("DS3231 set alarm error\n");
        // Fallback to direct sync
        printf("Falling back to direct sync...\n");
        reset_MAX_RTC(ds3231_datetime.tm_hour, ds3231_datetime.tm_min, ds3231_datetime.tm_sec);
        return;
    } else {
        strftime((char*)output_msgBuffer, OUTPUT_MSG_BUFFER_SIZE, "DS3231 Alarm set for: %F %TZ\n", &alarmTime);
        printf((char*)output_msgBuffer);
    }

    // Now enable DS3231 interrupt AFTER setting the alarm
    enable_DS3231_Interrupt();    

    printf("Waiting for DS3231 alarm to sync internal RTC...\n");
    
    // Wait for alarm with timeout
    uint32_t timeout_count = 0;
    (void)MXC_GPIO_InGet(MXC_GPIO1, MXC_GPIO_PIN_22); // Read initial state (unused but kept for debugging)
    
    while(!isAlarmTriggered && timeout_count < (SYNC_TIMEOUT_MS / 100)) {
        MXC_Delay(MXC_DELAY_MSEC(100));
        timeout_count++;
        
        // Check if alarm was triggered during the delay
        if (isAlarmTriggered) {
            break;
        }
        
        // Check if pin state changed (for debugging)
        uint32_t current_pin_state = MXC_GPIO_InGet(MXC_GPIO1, MXC_GPIO_PIN_22);
        
        // Check DS3231 status register every 2 seconds to see if alarm flag is set
        if (timeout_count % 20 == 0 && timeout_count > 0) {
            // Read DS3231 status register to check alarm flag
            // This is a manual check to see if DS3231 is setting the alarm flag
            printf("Checking DS3231 status register...\n");
        }
        
        // Print progress every second
        if (timeout_count % 10 == 0) {
            printf("Waiting... (%d seconds) [P1.22=%s]\n", 
                   timeout_count / 10, current_pin_state ? "HIGH" : "LOW");
            status_led_toggle(STATUS_LED_COLOR_BLUE);
            status_led_toggle(STATUS_LED_COLOR_GREEN);                
        }
    }
    
    if (isAlarmTriggered) {
        printf("RTC sync completed via alarm!\n");
    } else {
        printf("Alarm sync timeout! Falling back to direct sync...\n");
        reset_MAX_RTC(ds3231_datetime.tm_hour, ds3231_datetime.tm_min, ds3231_datetime.tm_sec);
    }
}



static void reset_MAX_RTC(int hour, int minute, int sec)
{
    int total_sec = (hour * SECS_PER_HR) + (minute * SECS_PER_MIN) + sec;

    printf("Setting internal RTC to %02d:%02d:%02d (total_sec: %d)\n", hour, minute, sec, total_sec);

    if (MXC_RTC_Init(total_sec, 0) != E_NO_ERROR) {
        printf("Failed RTC Initialization\n");
        return;
    }

    if (MXC_RTC_Start() != E_NO_ERROR) {
        printf("Failed RTC_Start\n");
        return;
    }
    
    printf("Internal RTC initialized and started successfully\n");
}

static void enable_DS3231_Interrupt(void)
{
    printf("Configuring DS3231 interrupt on GPIO P1.22...\n");
    
    // Read the pin state before configuration
    uint32_t pin_state_before = MXC_GPIO_InGet(MXC_GPIO1, MXC_GPIO_PIN_22);
    printf("P1.22 state before config: %s\n", pin_state_before ? "HIGH" : "LOW");
    
    // Configure the GPIO pin (rtc_int_cfg is now static/global)
    MXC_GPIO_Config(&rtc_int_cfg);
    
    // Register the callback
    MXC_GPIO_RegisterCallback(&rtc_int_cfg, ds3231_ISR, NULL);
    
    // Configure interrupt for both edges to catch the alarm transition
    if (MXC_GPIO_IntConfig(&rtc_int_cfg, MXC_GPIO_INT_BOTH) != E_NO_ERROR) {
        printf("Failed to configure DS3231 interrupt\n");
        return;
    }
    
    // Enable the interrupt
    MXC_GPIO_EnableInt(rtc_int_cfg.port, rtc_int_cfg.mask);
    
    // Enable the NVIC interrupt for GPIO1
    NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(MXC_GPIO_GET_IDX(rtc_int_cfg.port)));
    
    printf("DS3231 interrupt configured successfully on P1.22 with pull-up\n");
    
    // Read the current pin state after configuration
    uint32_t pin_state_after = MXC_GPIO_InGet(MXC_GPIO1, MXC_GPIO_PIN_22);
    printf("P1.22 state after config: %s\n", pin_state_after ? "HIGH" : "LOW");
    
    // If the pin is already LOW, the alarm might have already triggered
    if (!pin_state_after) {
        printf("WARNING: P1.22 is LOW after config - alarm may have already triggered!\n");
    }
}

static void ds3231_ISR(void *cbdata)
{
    // Prevent multiple triggers - check if already processed
    if (isAlarmTriggered) {
        printf("DS3231 alarm already processed, ignoring duplicate interrupt\n");
        return;
    }
    
    // Check current pin state to determine which edge triggered
    uint32_t pin_state = MXC_GPIO_InGet(MXC_GPIO1, MXC_GPIO_PIN_22);
    printf("DS3231 interrupt triggered! Pin state: %s\n", pin_state ? "HIGH" : "LOW");
    
    // Clear GPIO interrupt flags first
    uint32_t gpio_flags = MXC_GPIO_GetFlags(rtc_int_cfg.port);
    MXC_GPIO_ClearFlags(rtc_int_cfg.port, gpio_flags);
    
    // We want to sync on either edge since the alarm can trigger quickly
    // Disable the DS3231 interrupt to prevent repeated triggers
    MXC_GPIO_DisableInt(rtc_int_cfg.port, rtc_int_cfg.mask);
    
    // Clear DS3231 interrupt flags and disable alarm
    DS3231_clearInterrupts();
    DS3231_clearDisableInterrupts();  // This disables the alarm to prevent re-triggering
    
    // Calculate the alarm time (original time + delay) with proper overflow handling
    int alarm_hour = ds3231_datetime.tm_hour;
    int alarm_min = ds3231_datetime.tm_min;
    int alarm_sec = ds3231_datetime.tm_sec + ALARM_SYNC_DELAY_S;
    
    // Handle second overflow
    if (alarm_sec >= 60) {
        alarm_sec -= 60;
        alarm_min++;
        if (alarm_min >= 60) {
            alarm_min -= 60;
            alarm_hour++;
            if (alarm_hour >= 24) {
                alarm_hour -= 24;
            }
        }
    }

    // When DS3231 interrupt triggers, sync the internal RTC to the alarm time
    reset_MAX_RTC(alarm_hour, alarm_min, alarm_sec);
    
    // Enable RTC interrupt 
    NVIC_EnableIRQ(RTC_IRQn);    
        
    // Set the flag to indicate sync is complete
    isAlarmTriggered = true;
    
    printf("Internal RTC synced to DS3231 alarm time\n");
}

void RTC_IRQHandler(void)
{
    // Handle internal RTC interrupts if needed
    int flags = MXC_RTC_GetFlags();
    MXC_RTC_ClearFlags(flags);
}

void GPIO1_IRQHandler(void)
{
    // Handle GPIO1 interrupts (includes DS3231 RTC interrupt on P1.22)
    MXC_GPIO_Handler(MXC_GPIO_GET_IDX(MXC_GPIO1));
}

static uint32_t get_internal_RTC_time(void *buff, void *strbuff)
{
    struct tm_ms *timeinfo = (struct tm_ms *)buff;
    char *time_str = (char *)strbuff;

    uint32_t sec, rtc_readout, subsec_readout;
    int err;

    // Read subseconds first (RTC subsecond counter counts at 4096 Hz)
    do {
        err = MXC_RTC_GetSubSeconds(&subsec_readout);
    } while (err != E_NO_ERROR);  //TODO: add timeout
    
    // Convert subseconds (0-4095) to milliseconds (0-999)
    timeinfo->tm_subsec = (subsec_readout * 1000) / 4096;

    do {
        err = MXC_RTC_GetSeconds(&rtc_readout);
    } while (err != E_NO_ERROR);  //TODO: add timeout
    
    // Convert total seconds to time-of-day (mod 24 hours)
    // This prevents hours > 24 after running for multiple days
    sec = rtc_readout % SECS_PER_DAY;
    
    timeinfo->tm_hour = sec / SECS_PER_HR;
    sec -= timeinfo->tm_hour * SECS_PER_HR;

    timeinfo->tm_min = sec / SECS_PER_MIN;
    sec -= timeinfo->tm_min * SECS_PER_MIN;

    timeinfo->tm_sec = sec;

    // Format time as string
    sprintf(time_str, "%02d:%02d:%02d.%03d", 
            timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec, timeinfo->tm_subsec);

    return E_NO_ERROR;
}


static void print_time_with_milliseconds(void)
{
    int year, month, day, hr, min, err;
    uint32_t sec, rtc_readout;
    double subsec;

    do {
        err = MXC_RTC_GetSubSeconds(&rtc_readout);
    } while (err != E_NO_ERROR);
    subsec = rtc_readout / 4096.0;

    do {
        err = MXC_RTC_GetSeconds(&rtc_readout);
    } while (err != E_NO_ERROR);
    sec = rtc_readout;

    hr = sec / SECS_PER_HR;
    sec -= hr * SECS_PER_HR;

    min = sec / SECS_PER_MIN;
    sec -= min * SECS_PER_MIN;

    subsec += sec;

    year = ds3231_datetime.tm_year + 1900; 
    month = ds3231_datetime.tm_mon + 1;
    day = ds3231_datetime.tm_mday;

    printf("Internal RTC DateTime: %d/%02d/%02d %02d:%02d:%06.3fZ\n", year, month, day, hr, min, subsec);

    if (E_NO_ERROR != DS3231_RTC.read_datetime(&ds3231_datetime, ds3231_datetime_str)) {
        printf("DS3231 read datetime error\n");
    } else {
        strftime((char*)output_msgBuffer, OUTPUT_MSG_BUFFER_SIZE, "DS3231 RTC DateTime: %F %TZ\n\n", &ds3231_datetime);
        printf((char*)output_msgBuffer);
    }
}

///////////////////////////SCHEDULE FILE LOADING ////////////////////////////////////

/**
 * @brief Parses a line from the schedule.sch file and extracts the value after '='
 * @param line The line to parse
 * @param key The key to look for (e.g., "Sampling Rate [kHz]")
 * @param value Output buffer for the value string
 * @param value_size Size of the value buffer
 * @return true if key was found and value extracted, false otherwise
 */
static bool parse_schedule_line(const char *line, const char *key, char *value, size_t value_size)
{
    const char *key_pos = strstr(line, key);
    if (key_pos == NULL) {
        return false;
    }
    
    const char *eq_pos = strchr(key_pos, '=');
    if (eq_pos == NULL) {
        return false;
    }
    
    // Skip the '=' and any leading whitespace
    eq_pos++;
    while (*eq_pos == ' ' || *eq_pos == '\t') {
        eq_pos++;
    }
    
    // Copy the value, trimming trailing whitespace/newlines
    size_t i = 0;
    while (*eq_pos != '\0' && *eq_pos != '\r' && *eq_pos != '\n' && i < value_size - 1) {
        value[i++] = *eq_pos++;
    }
    value[i] = '\0';
    
    // Trim trailing whitespace
    while (i > 0 && (value[i-1] == ' ' || value[i-1] == '\t')) {
        value[--i] = '\0';
    }
    
    return true;
}

/**
 * @brief Loads configuration from schedule.sch file on SD_CARD_BANK_CARD_SLOT_5
 * 
 * Schedule file format:
 *   Sampling Rate [kHz] (24/48/96/192/384)=384
 *   Bit Rate (16/24)=16
 *   Mode (Mono/Stereo)=Mono
 *   Channel (0/1)=0
 *   Audio Duration [s]=3600
 * 
 * @return E_NO_ERROR if schedule loaded successfully, error code otherwise
 *         If no schedule file found, returns E_NO_ERROR and uses defaults
 */
static int load_schedule_from_slot5(void)
{
    printf("\n[INFO]--> Checking for schedule file on SD slot 5...\n");
    
    // Enable slot 5
    if (sd_card_bank_ctl_enable_slot(SYS_CONFIG_SCHEDULE_SD_SLOT) != E_NO_ERROR) {
        printf("[INFO]--> Could not enable SD slot 5, using default config\n");
        return E_NO_ERROR;  // Not an error, just use defaults
    }
    
    // Check if card is inserted
    sd_card_bank_ctl_read_and_cache_detect_pins();
    if (!sd_card_bank_ctl_active_card_is_inserted()) {
        printf("[INFO]--> No SD card in slot 5, using default config\n");
        sd_card_bank_ctl_disable_all();
        return E_NO_ERROR;
    }
    
    // Initialize and mount the card
    if (sd_card_init() != E_NO_ERROR) {
        printf("[INFO]--> Could not init SD card in slot 5, using default config\n");
        sd_card_bank_ctl_disable_all();
        return E_NO_ERROR;
    }
    
    MXC_Delay(100000);  // Brief delay before mount
    
    if (sd_card_mount() != E_NO_ERROR) {
        printf("[INFO]--> Could not mount SD card in slot 5, using default config\n");
        sd_card_bank_ctl_disable_all();
        return E_NO_ERROR;
    }
    
    // Check if schedule file exists
    if (!sd_card_file_exists(SYS_CONFIG_SCHEDULE_FILENAME)) {
        printf("[INFO]--> No %s file found on slot 5, using default config\n", SYS_CONFIG_SCHEDULE_FILENAME);
        sd_card_unmount();
        sd_card_bank_ctl_disable_all();
        return E_NO_ERROR;
    }
    
    // Open and read the schedule file
    if (sd_card_fopen(SYS_CONFIG_SCHEDULE_FILENAME, POSIX_FILE_MODE_READ) != E_NO_ERROR) {
        printf("[WARNING]--> Could not open %s, using default config\n", SYS_CONFIG_SCHEDULE_FILENAME);
        sd_card_unmount();
        sd_card_bank_ctl_disable_all();
        return E_NO_ERROR;
    }
    
    printf("[SUCCESS]--> Found %s on slot 5, loading configuration...\n", SYS_CONFIG_SCHEDULE_FILENAME);
    
    // Read the file content
    static char file_buffer[512];
    uint32_t bytes_read = 0;
    
    if (sd_card_fread(file_buffer, sizeof(file_buffer) - 1, &bytes_read) != E_NO_ERROR || bytes_read == 0) {
        printf("[WARNING]--> Could not read %s, using default config\n", SYS_CONFIG_SCHEDULE_FILENAME);
        sd_card_fclose();
        sd_card_unmount();
        sd_card_bank_ctl_disable_all();
        return E_NO_ERROR;
    }
    
    file_buffer[bytes_read] = '\0';  // Null terminate
    
    sd_card_fclose();
    sd_card_unmount();
    sd_card_bank_ctl_disable_all();
    
    // Parse the schedule file
    char value_str[32];
    bool config_changed = false;
    
    // Parse Sampling Rate [kHz]
    if (parse_schedule_line(file_buffer, "Sampling Rate", value_str, sizeof(value_str))) {
        int sample_rate_khz = atoi(value_str);
        switch (sample_rate_khz) {
            case 24:
                SYS_CONFIG_SAMPLE_RATE = AUDIO_SAMPLE_RATE_24kHz;
                config_changed = true;
                break;
            case 48:
                SYS_CONFIG_SAMPLE_RATE = AUDIO_SAMPLE_RATE_48kHz;
                config_changed = true;
                break;
            case 96:
                SYS_CONFIG_SAMPLE_RATE = AUDIO_SAMPLE_RATE_96kHz;
                config_changed = true;
                break;
            case 192:
                SYS_CONFIG_SAMPLE_RATE = AUDIO_SAMPLE_RATE_192kHz;
                config_changed = true;
                break;
            case 384:
                SYS_CONFIG_SAMPLE_RATE = AUDIO_SAMPLE_RATE_384kHz;
                config_changed = true;
                break;
            default:
                printf("[WARNING]--> Invalid sample rate %d kHz in schedule, using default\n", sample_rate_khz);
                break;
        }
        if (config_changed) {
            printf("[CONFIG]--> Sample Rate: %lu kHz\n", (unsigned long)(SYS_CONFIG_SAMPLE_RATE / 1000));
        }
    }
    
    // Parse Bit Rate
    if (parse_schedule_line(file_buffer, "Bit Rate", value_str, sizeof(value_str))) {
        int bit_depth = atoi(value_str);
        if (bit_depth == 16) {
            SYS_CONFIG_NUM_BIT_DEPTH = AUDIO_BIT_DEPTH_16_BITS_PER_SAMPLE;
            config_changed = true;
            printf("[CONFIG]--> Bit Depth: 16 bits\n");
        } else if (bit_depth == 24) {
            SYS_CONFIG_NUM_BIT_DEPTH = AUDIO_BIT_DEPTH_24_BITS_PER_SAMPLE;
            config_changed = true;
            printf("[CONFIG]--> Bit Depth: 24 bits\n");
        } else {
            printf("[WARNING]--> Invalid bit depth %d in schedule, using default\n", bit_depth);
        }
    }
    
    // Parse Mode (Mono/Stereo)
    if (parse_schedule_line(file_buffer, "Mode", value_str, sizeof(value_str))) {
        if (strstr(value_str, "Mono") != NULL || strstr(value_str, "mono") != NULL) {
            SYS_CONFIG_NUM_CHANNEL = WAVE_HEADER_MONO;
            config_changed = true;
            printf("[CONFIG]--> Mode: Mono\n");
        } else if (strstr(value_str, "Stereo") != NULL || strstr(value_str, "stereo") != NULL) {
            SYS_CONFIG_NUM_CHANNEL = WAVE_HEADER_STEREO;
            config_changed = true;
            printf("[CONFIG]--> Mode: Stereo\n");
        } else {
            printf("[WARNING]--> Invalid mode '%s' in schedule, using default\n", value_str);
        }
    }
    
    // Parse Channel (0/1) - only relevant for mono mode
    if (parse_schedule_line(file_buffer, "Channel", value_str, sizeof(value_str))) {
        int channel = atoi(value_str);
        if (channel == 0) {
            SYS_CONFIG_CHANNEL_TO_USE_FOR_MONO_MODE = AUDIO_CHANNEL_0;
            config_changed = true;
            printf("[CONFIG]--> Mono Channel: 0\n");
        } else if (channel == 1) {
            SYS_CONFIG_CHANNEL_TO_USE_FOR_MONO_MODE = AUDIO_CHANNEL_1;
            config_changed = true;
            printf("[CONFIG]--> Mono Channel: 1\n");
        } else {
            printf("[WARNING]--> Invalid channel %d in schedule, using default\n", channel);
        }
    }
    
    // Parse Audio Duration [s]
    if (parse_schedule_line(file_buffer, "Audio Duration", value_str, sizeof(value_str))) {
        int duration = atoi(value_str);
        if (duration > 0 && duration <= 4000) {  // Max ~70 minutes as per original comment
            SYS_CONFIG_AUDIO_FILE_LEN_IN_SECONDS = (uint32_t)duration;
            config_changed = true;
            printf("[CONFIG]--> Audio Duration: %lu seconds\n", (unsigned long)SYS_CONFIG_AUDIO_FILE_LEN_IN_SECONDS);
        } else {
            printf("[WARNING]--> Invalid duration %d in schedule (must be 1-4000), using default\n", duration);
        }
    }
    
    if (config_changed) {
        printf("[SUCCESS]--> Schedule configuration loaded from slot 5\n");
    } else {
        printf("[INFO]--> No valid configuration found in schedule file, using defaults\n");
    }
    
    return E_NO_ERROR;
}

///////////////////////////FOLDER MANAGEMENT FUNCTIONS ////////////////////////////////////

static int setup_session_folder(void)
{
    int highest_num = -1;
    
    // Find the highest existing session folder number
    if (sd_card_find_highest_session_folder(SYS_CONFIG_RECORDER_ID, &highest_num) != E_NO_ERROR) {
        printf("[WARNING]--> Could not scan for existing session folders\n");
        highest_num = -1;
    }
    
    // Create new session folder with next number
    int new_num = highest_num + 1;
    if (new_num > 999) {
        printf("[ERROR]--> Session folder number exceeded 999\n");
        return E_OVERFLOW;
    }
    
    sprintf(currentSessionFolder, "%s_%03d", SYS_CONFIG_RECORDER_ID, new_num);
    
    // Create the session folder
    if (sd_card_mkdir(currentSessionFolder) != E_NO_ERROR) {
        // Check if it already exists (shouldn't happen but handle gracefully)
        if (!sd_card_dir_exists(currentSessionFolder)) {
            printf("[ERROR]--> Failed to create session folder: %s\n", currentSessionFolder);
            return E_COMM_ERR;
        }
    }
    
    // Set folder timestamp to current RTC time
    set_file_timestamp(currentSessionFolder, 
                       ds3231_datetime.tm_year + 1900,
                       ds3231_datetime.tm_mon + 1,
                       ds3231_datetime.tm_mday,
                       ds3231_datetime.tm_hour,
                       ds3231_datetime.tm_min,
                       ds3231_datetime.tm_sec);
    
    printf("[SUCCESS]--> Created session folder: %s\n", currentSessionFolder);
    
    // Reset day tracking for new session
    currentDayOfYear = -1;
    currentDayFolder[0] = '\0';
    
    return E_NO_ERROR;
}

static int setup_day_folder(int year, int month, int day, uint32_t sample_rate, uint32_t bit_depth, uint32_t num_channels)
{
    // Calculate unique identifiers
    uint32_t sampleRateKHz = sample_rate / 1000;
    int dayId = year * 10000 + month * 100 + day;
    
    // Format: SessionFolder/YYYYMMDD_XXXkHz_XXbit_CHXX - use static to avoid stack issues
    static char newDayFolder[96];
    
    // Ensure session folder exists (may have been wiped from SD card)
    if (currentSessionFolder[0] != '\0' && !sd_card_dir_exists(currentSessionFolder)) {
        printf("[INFO]--> Session folder missing, recreating: %s\n", currentSessionFolder);
        if (sd_card_mkdir(currentSessionFolder) != E_NO_ERROR) {
            printf("[ERROR]--> Failed to recreate session folder: %s\n", currentSessionFolder);
            return E_COMM_ERR;
        }
        // Set folder timestamp to current RTC time
        set_file_timestamp(currentSessionFolder, 
                           ds3231_datetime.tm_year + 1900,
                           ds3231_datetime.tm_mon + 1,
                           ds3231_datetime.tm_mday,
                           ds3231_datetime.tm_hour,
                           ds3231_datetime.tm_min,
                           ds3231_datetime.tm_sec);
        printf("[SUCCESS]--> Recreated session folder: %s\n", currentSessionFolder);
    }
    
    // Build full path: SessionFolder/20251126_384kHz_24bit_CH02
    snprintf(newDayFolder, sizeof(newDayFolder), "%s/%d%02d%02d_%lukHz_%lubit_CH%02lu", 
             currentSessionFolder, year, month, day, 
             (unsigned long)sampleRateKHz,
             (unsigned long)bit_depth,
             (unsigned long)num_channels);
    
    // Check if we need to create a new folder (any parameter changed)
    if (dayId != currentDayOfYear || 
        sampleRateKHz != currentSampleRateKHz ||
        bit_depth != currentBitDepth ||
        num_channels != currentNumChannels) {
        
        // Check if folder already exists
        if (!sd_card_dir_exists(newDayFolder)) {
            // Create the day folder
            if (sd_card_mkdir(newDayFolder) != E_NO_ERROR) {
                printf("[ERROR]--> Failed to create day folder: %s\n", newDayFolder);
                return E_COMM_ERR;
            }
            
            // Set folder timestamp to current RTC time
            set_file_timestamp(newDayFolder, 
                               ds3231_datetime.tm_year + 1900,
                               ds3231_datetime.tm_mon + 1,
                               ds3231_datetime.tm_mday,
                               ds3231_datetime.tm_hour,
                               ds3231_datetime.tm_min,
                               ds3231_datetime.tm_sec);
            
            printf("[SUCCESS]--> Created day folder: %s\n", newDayFolder);
        } else {
            printf("[INFO]--> Using existing day folder: %s\n", newDayFolder);
        }
        
        // Update tracking variables
        currentDayOfYear = dayId;
        currentSampleRateKHz = sampleRateKHz;
        currentBitDepth = bit_depth;
        currentNumChannels = num_channels;
        strcpy(currentDayFolder, newDayFolder);
    }
    
    // Ensure day folder exists before trying to cd into it (may have been wiped from SD card)
    if (currentDayFolder[0] != '\0' && !sd_card_dir_exists(currentDayFolder)) {
        printf("[INFO]--> Day folder missing, recreating: %s\n", currentDayFolder);
        if (sd_card_mkdir(currentDayFolder) != E_NO_ERROR) {
            printf("[ERROR]--> Failed to recreate day folder: %s\n", currentDayFolder);
            return E_COMM_ERR;
        }
        // Set folder timestamp to current RTC time
        set_file_timestamp(currentDayFolder, 
                           ds3231_datetime.tm_year + 1900,
                           ds3231_datetime.tm_mon + 1,
                           ds3231_datetime.tm_mday,
                           ds3231_datetime.tm_hour,
                           ds3231_datetime.tm_min,
                           ds3231_datetime.tm_sec);
        printf("[SUCCESS]--> Recreated day folder: %s\n", currentDayFolder);
    }
    
    // Always change to the day folder (FatFS resets cwd on mount)
    if (sd_card_cd(currentDayFolder) != E_NO_ERROR) {
        printf("[ERROR]--> Failed to change to day folder: %s\n", currentDayFolder);
        return E_COMM_ERR;
    }
    printf("[INFO]--> Working in day folder: %s\n", currentDayFolder);
    
    return E_NO_ERROR;
}

///////////////////////////ISP CALL BACKS ////////////////////////////////////

static void user_pushbutton_interrupt_callback(void *cbdata)
{
    printf("[INFO]--> Pushbutton ISR triggered!\n");
    
    // Get and clear interrupt flags
    uint32_t status = MXC_GPIO_GetFlags(bsp_pins_user_pushbutton_cfg.port);
    MXC_GPIO_ClearFlags(bsp_pins_user_pushbutton_cfg.port, status);

    // Disable interrupt temporarily for this specific pin only
    MXC_GPIO_DisableInt(bsp_pins_user_pushbutton_cfg.port, bsp_pins_user_pushbutton_cfg.mask);
    printf("[INFO]--> Pushbutton interrupt DISABLED (for debounce)\n");

    // Don't disable the entire GPIO0 NVIC interrupt since DS3231 might need it
    // The GPIO system will handle multiple callbacks on the same port
    // NVIC_DisableIRQ(GPIO0_IRQn);  // Removed - let other GPIO0 interrupts continue

    // Start the debounce timer which should produce "button_pressed" after some time after
    // checking the GPIO pin
    start_user_btn_debounceTimer();  //re-activate one shot timer
    printf("[INFO]--> Debounce timer started\n");
}
