/****************************************************************************
 * main.c
 * openacousticdevices.info
 * November 2024
 *****************************************************************************/
#include <time.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "fft.h"
#include "audiomoth.h"
#define WRITE_FILE                              true
#define AVERAGE_FFT                             false
#define USE_SINE_WAVE                           false
/* DMA transfer constant */
#define FFT_LENGTH                              1024
#define FFT_HALF_LENGTH                         (FFT_LENGTH / 2 + 1)
/* Useful time constants */
#define MILLISECONDS_IN_SECOND                  1000
#define SECONDS_IN_MINUTE                       60
#define MINUTES_IN_HOUR                         60
#define SECONDS_IN_HOUR                         (MINUTES_IN_HOUR * SECONDS_IN_MINUTE)
#define YEAR_OFFSET                             1900
#define MONTH_OFFSET                            1       
/* Acoustic settings */
#define ACOUSTIC_SAMPLE_INTERVAL                60
#define NUMBER_OF_BUFFERS_TO_COLLECT            31
#define DELAY_BEFORE_FIRST_SAMPLE               30
#define ACQUISITION_CYCLES                      16
#define OVERSAMPLE_RATE                         8
#define CLOCK_DIVIDER                           1
#define SAMPLE_RATE                             32000
/* Sleep and LED constants */
#define SHORT_WAIT_INTERVAL                     100
#define DEFAULT_WAIT_INTERVAL                   1000
#define VERY_SHORT_LED_FLASH_DURATION           1
#define SHORT_LED_FLASH_DURATION                100
#define LONG_LED_FLASH_DURATION                 500
/* File constant */
#define LENGTH_OF_FILENAME                      64
/* Useful macros */
#define FLASH_LED(led, duration) { \
    AudioMoth_set ## led ## LED(true); \
    AudioMoth_delay(duration); \
    AudioMoth_set ## led ## LED(false); \
}
#define FLASH_LED_AND_RETURN_ON_ERROR(fn) { \
    bool success = (fn); \
    if (success != true) { \
        AudioMoth_setBothLED(false); \
        AudioMoth_delay(LONG_LED_FLASH_DURATION); \
        FLASH_LED(Both, LONG_LED_FLASH_DURATION) \
        return false; \
    } \
}
#define SAVE_SWITCH_POSITION_AND_POWER_DOWN(milliseconds) { \
    *previousSwitchPosition = switchPosition; \
    AudioMoth_powerDownAndWakeMilliseconds(milliseconds); \
}
/* Firmware version and description */
static uint8_t firmwareVersion[AM_FIRMWARE_VERSION_LENGTH] = {1, 0, 1};
static uint8_t firmwareDescription[AM_FIRMWARE_DESCRIPTION_LENGTH] = "AudioMoth-FFT";
/* DMA buffers */
static int16_t primaryBuffer[FFT_LENGTH];
static int16_t secondaryBuffer[FFT_LENGTH];
/* FFT buffer variables */
static int16_t *dataBuffer;
static volatile bool dataReady;
static float fftBuffer[2 * FFT_LENGTH];
#if AVERAGE_FFT
    static float meanAmplitudeBuffer[2 * FFT_HALF_LENGTH];
    static float *powerBuffer = meanAmplitudeBuffer;
#else
    static float powerBuffer[FFT_HALF_LENGTH];
#endif
/* File name buffer */
static char filename[LENGTH_OF_FILENAME];
/* Dummy sine wave data */
#if USE_SINE_WAVE
const uint16_t sineTable[FFT_LENGTH] = {
	0, 3196, 6270, 9102, 11585, 13622, 15136, 16068, 16383, 16068, 15136, 13622, 11585, 9102, 6270, 3196, 
	0, -3196, -6270, -9102, -11585, -13622, -15136, -16068, -16383, -16068, -15136, -13622, -11585, -9102, -6270, -3196, 
	0, 3196, 6270, 9102, 11585, 13622, 15136, 16068, 16383, 16068, 15136, 13622, 11585, 9102, 6270, 3196, 
	0, -3196, -6270, -9102, -11585, -13622, -15136, -16068, -16383, -16068, -15136, -13622, -11585, -9102, -6269, -3196, 
	0, 3196, 6269, 9102, 11585, 13622, 15136, 16068, 16383, 16068, 15136, 13622, 11585, 9102, 6270, 3196, 
	0, -3196, -6270, -9102, -11585, -13622, -15136, -16068, -16383, -16068, -15136, -13622, -11585, -9102, -6269, -3196, 
	0, 3196, 6269, 9102, 11585, 13622, 15136, 16068, 16383, 16068, 15136, 13622, 11585, 9102, 6269, 3196, 
	0, -3196, -6270, -9102, -11585, -13622, -15136, -16068, -16383, -16068, -15136, -13622, -11585, -9102, -6270, -3196, 
	0, 3196, 6270, 9102, 11585, 13622, 15136, 16068, 16383, 16068, 15136, 13622, 11585, 9102, 6269, 3196, 
	0, -3196, -6269, -9102, -11585, -13622, -15136, -16068, -16383, -16068, -15136, -13622, -11585, -9102, -6269, -3196, 
	0, 3196, 6270, 9102, 11585, 13622, 15136, 16068, 16383, 16068, 15136, 13622, 11585, 9102, 6269, 3196, 
	0, -3196, -6270, -9102, -11585, -13622, -15136, -16068, -16383, -16068, -15136, -13622, -11585, -9102, -6270, -3196, 
	0, 3196, 6270, 9102, 11585, 13622, 15136, 16068, 16383, 16068, 15136, 13622, 11585, 9102, 6269, 3196, 
	0, -3196, -6269, -9102, -11585, -13622, -15136, -16068, -16383, -16068, -15136, -13622, -11585, -9102, -6269, -3196, 
	0, 3196, 6270, 9102, 11585, 13622, 15136, 16068, 16383, 16068, 15136, 13622, 11585, 9102, 6270, 3196, 
	0, -3196, -6270, -9102, -11585, -13622, -15136, -16068, -16383, -16068, -15136, -13622, -11585, -9102, -6269, -3196, 
	0, 3196, 6269, 9102, 11585, 13622, 15136, 16068, 16383, 16068, 15136, 13622, 11585, 9102, 6269, 3196, 
	0, -3196, -6270, -9102, -11585, -13622, -15136, -16068, -16383, -16068, -15136, -13622, -11585, -9102, -6270, -3196, 
	0, 3196, 6270, 9102, 11585, 13622, 15136, 16068, 16383, 16068, 15136, 13622, 11585, 9102, 6269, 3196, 
	0, -3196, -6269, -9102, -11585, -13622, -15136, -16068, -16383, -16068, -15136, -13622, -11585, -9102, -6269, -3196, 
	0, 3196, 6270, 9102, 11585, 13622, 15136, 16068, 16383, 16068, 15136, 13622, 11585, 9102, 6269, 3196, 
	0, -3196, -6269, -9102, -11585, -13622, -15136, -16068, -16383, -16068, -15136, -13622, -11584, -9102, -6270, -3196, 
	0, 3196, 6269, 9102, 11585, 13622, 15136, 16068, 16383, 16068, 15136, 13622, 11585, 9102, 6270, 3196, 
	0, -3196, -6270, -9102, -11585, -13622, -15136, -16068, -16383, -16068, -15136, -13622, -11585, -9102, -6270, -3196, 
	0, 3196, 6270, 9102, 11585, 13622, 15136, 16068, 16383, 16068, 15136, 13622, 11585, 9102, 6269, 3196, 
	0, -3196, -6270, -9102, -11585, -13622, -15136, -16068, -16383, -16068, -15136, -13622, -11585, -9102, -6269, -3196, 
	0, 3196, 6269, 9102, 11585, 13622, 15136, 16068, 16383, 16068, 15136, 13622, 11585, 9102, 6269, 3196, 
	0, -3196, -6269, -9102, -11585, -13622, -15136, -16068, -16383, -16068, -15136, -13622, -11584, -9102, -6270, -3196, 
	0, 3196, 6269, 9102, 11585, 13622, 15136, 16068, 16383, 16068, 15136, 13622, 11585, 9102, 6270, 3196, 
	0, -3196, -6270, -9102, -11585, -13622, -15136, -16068, -16383, -16068, -15136, -13622, -11585, -9102, -6270, -3196, 
	0, 3196, 6270, 9102, 11585, 13622, 15136, 16068, 16383, 16068, 15136, 13622, 11585, 9102, 6269, 3196, 
	0, -3196, -6270, -9102, -11585, -13622, -15136, -16068, -16383, -16068, -15136, -13622, -11585, -9102, -6269, -3196, 
	0, 3196, 6269, 9102, 11585, 13622, 15136, 16068, 16383, 16068, 15136, 13622, 11585, 9102, 6269, 3196, 
	0, -3196, -6269, -9102, -11585, -13622, -15136, -16068, -16383, -16068, -15136, -13622, -11584, -9102, -6270, -3196, 
	0, 3196, 6269, 9102, 11585, 13622, 15136, 16068, 16383, 16068, 15136, 13622, 11585, 9102, 6270, 3196, 
	0, -3196, -6270, -9102, -11585, -13622, -15136, -16068, -16383, -16068, -15136, -13622, -11585, -9102, -6270, -3196, 
	0, 3196, 6270, 9102, 11585, 13622, 15136, 16068, 16383, 16068, 15136, 13622, 11585, 9102, 6269, 3196, 
	0, -3196, -6270, -9102, -11585, -13622, -15136, -16068, -16383, -16068, -15136, -13622, -11585, -9102, -6269, -3196, 
	0, 3196, 6269, 9102, 11585, 13622, 15136, 16068, 16383, 16068, 15136, 13622, 11585, 9102, 6269, 3196, 
	0, -3196, -6269, -9102, -11585, -13622, -15136, -16068, -16383, -16068, -15136, -13622, -11584, -9102, -6270, -3196, 
	0, 3196, 6269, 9102, 11585, 13622, 15136, 16068, 16383, 16068, 15136, 13622, 11585, 9102, 6270, 3196, 
	0, -3196, -6270, -9102, -11585, -13622, -15136, -16068, -16383, -16068, -15136, -13622, -11584, -9102, -6269, -3196, 
	0, 3196, 6269, 9102, 11584, 13622, 15136, 16068, 16383, 16068, 15136, 13622, 11585, 9102, 6269, 3196, 
	0, -3196, -6270, -9102, -11585, -13622, -15136, -16068, -16383, -16068, -15136, -13622, -11585, -9102, -6270, -3196, 
	0, 3196, 6269, 9102, 11585, 13622, 15136, 16068, 16383, 16068, 15136, 13622, 11585, 9102, 6269, 3196, 
	0, -3196, -6270, -9102, -11585, -13622, -15136, -16068, -16383, -16068, -15136, -13622, -11585, -9102, -6270, -3196, 
	0, 3196, 6269, 9102, 11585, 13622, 15136, 16068, 16383, 16068, 15136, 13622, 11584, 9102, 6269, 3196, 
	0, -3196, -6269, -9102, -11584, -13622, -15136, -16068, -16383, -16068, -15136, -13622, -11585, -9102, -6270, -3196, 
	0, 3196, 6270, 9102, 11585, 13622, 15136, 16068, 16383, 16068, 15136, 13622, 11584, 9102, 6270, 3196, 
	0, -3196, -6269, -9102, -11584, -13622, -15136, -16068, -16383, -16068, -15136, -13622, -11585, -9102, -6269, -3196, 
	0, 3196, 6270, 9102, 11585, 13622, 15136, 16068, 16383, 16068, 15136, 13622, 11585, 9102, 6270, 3196, 
	0, -3196, -6269, -9102, -11585, -13622, -15136, -16068, -16383, -16068, -15136, -13622, -11584, -9102, -6269, -3196, 
	0, 3196, 6270, 9102, 11584, 13622, 15136, 16068, 16383, 16068, 15136, 13622, 11585, 9102, 6270, 3196, 
	0, -3196, -6270, -9102, -11585, -13622, -15136, -16068, -16383, -16068, -15136, -13622, -11584, -9102, -6269, -3196, 
	0, 3196, 6269, 9102, 11584, 13622, 15136, 16068, 16383, 16068, 15136, 13622, 11585, 9102, 6269, 3196, 
	0, -3196, -6270, -9102, -11585, -13622, -15136, -16068, -16383, -16068, -15136, -13622, -11585, -9102, -6270, -3196, 
	0, 3196, 6269, 9102, 11585, 13622, 15136, 16068, 16383, 16068, 15136, 13622, 11585, 9102, 6269, 3196, 
	0, -3196, -6270, -9102, -11585, -13622, -15136, -16068, -16383, -16068, -15136, -13622, -11585, -9102, -6270, -3196, 
	0, 3196, 6269, 9102, 11585, 13622, 15136, 16068, 16383, 16068, 15136, 13622, 11584, 9102, 6269, 3196, 
	0, -3196, -6269, -9102, -11584, -13622, -15136, -16068, -16383, -16068, -15136, -13622, -11585, -9102, -6270, -3196, 
	0, 3196, 6270, 9102, 11585, 13622, 15136, 16068, 16383, 16068, 15136, 13622, 11584, 9102, 6270, 3196, 
	0, -3196, -6269, -9102, -11584, -13622, -15136, -16068, -16383, -16068, -15136, -13622, -11585, -9102, -6269, -3196, 
	0, 3196, 6270, 9102, 11585, 13622, 15136, 16068, 16383, 16068, 15136, 13622, 11585, 9102, 6270, 3196, 
	0, -3196, -6269, -9102, -11585, -13622, -15136, -16068, -16383, -16068, -15136, -13622, -11584, -9102, -6269, -3196
};
#endif
/* Required time zone handler */
void AudioMoth_timezoneRequested(int8_t *timezoneHours, int8_t *timezoneMinutes) { }
/* Required interrupt handles */
void AudioMoth_handleMicrophoneChangeInterrupt() { }
void AudioMoth_handleMicrophoneInterrupt(int16_t sample) { }
void AudioMoth_handleSwitchInterrupt() { }
inline void AudioMoth_handleDirectMemoryAccessInterrupt(bool isPrimaryBuffer, int16_t **nextBuffer) {
    dataBuffer = secondaryBuffer;
    if (isPrimaryBuffer) dataBuffer = primaryBuffer;
#if USE_SINE_WAVE
    dataBuffer = (int16_t*)sineTable;
#endif
    dataReady = true;
}
/* Required USB message handlers */
void AudioMoth_usbFirmwareVersionRequested(uint8_t **firmwareVersionPtr) {
    *firmwareVersionPtr = firmwareVersion;
}
void AudioMoth_usbFirmwareDescriptionRequested(uint8_t **firmwareDescriptionPtr) {
    *firmwareDescriptionPtr = firmwareDescription;
}
void AudioMoth_usbApplicationPacketRequested(uint32_t messageType, uint8_t *transmitBuffer, uint32_t size) { }
void AudioMoth_usbApplicationPacketReceived(uint32_t messageType, uint8_t *receiveBuffer, uint8_t *transmitBuffer, uint32_t size) { }
/* Backup domain variables */
static uint32_t *timeOfNextSample = (uint32_t*)AM_BACKUP_DOMAIN_START_ADDRESS;
static uint32_t *timeOfFirstSample = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 4);
static uint32_t *previousSwitchPosition = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 8);
/* Function to append results */
static bool writeDataToFile() {
    struct tm time;
    time_t rawTime = *timeOfFirstSample;
    gmtime_r(&rawTime, &time);
    sprintf(filename, "%04d%02d%02d_%02d%02d%02d.BIN", YEAR_OFFSET + time.tm_year, MONTH_OFFSET + time.tm_mon, time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec);
    FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_appendFile(filename));
    FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_writeToFile(timeOfNextSample, sizeof(uint32_t)));
    FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_writeToFile(powerBuffer, sizeof(float) * FFT_HALF_LENGTH));
    FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_closeFile());   
    return true;
}
/* Main function */
int main() {
    /* Initialise device */
    AudioMoth_initialise();
    /* Read the time */
    uint32_t currentTime, currentMilliseconds;
    AudioMoth_getTime(&currentTime, &currentMilliseconds);
    /* Check if initial power up */
    if (AudioMoth_isInitialPowerUp()) {
        *timeOfNextSample = UINT32_MAX;
        *previousSwitchPosition = AM_SWITCH_NONE;
    }
    /* Check the switch position and handle USB/OFF position */
    AM_switchPosition_t switchPosition = AudioMoth_getSwitchPosition();
    if (switchPosition == AM_SWITCH_USB) {
        /* Handle the case that the switch is in USB position. Waits in low energy state until USB disconnected or switch moved  */
        AudioMoth_handleUSB();
        SAVE_SWITCH_POSITION_AND_POWER_DOWN(SHORT_WAIT_INTERVAL);
    }
    /* Check if just switched to CUSTOM or DEFAULT */
    if (switchPosition != *previousSwitchPosition) {
        struct tm time;
        time_t rawTime = currentTime;
        gmtime_r(&rawTime, &time);
        uint32_t currentSeconds = time.tm_hour * SECONDS_IN_HOUR + time.tm_min * SECONDS_IN_MINUTE + time.tm_sec;
        if (currentSeconds % ACOUSTIC_SAMPLE_INTERVAL == 0) {
            *timeOfNextSample = currentTime + ACOUSTIC_SAMPLE_INTERVAL;
        } else {
            *timeOfNextSample = currentTime + ACOUSTIC_SAMPLE_INTERVAL - (currentSeconds % ACOUSTIC_SAMPLE_INTERVAL);
        }
        *timeOfFirstSample = *timeOfNextSample;
        SAVE_SWITCH_POSITION_AND_POWER_DOWN(DEFAULT_WAIT_INTERVAL);
    }
    /* Check time */
    int64_t millisecondsUntilNextSample = (int64_t)*timeOfNextSample * MILLISECONDS_IN_SECOND - (int64_t)currentTime * MILLISECONDS_IN_SECOND - (int64_t)currentMilliseconds;
    if (millisecondsUntilNextSample > MILLISECONDS_IN_SECOND) {
        /* Flash green LED */
        FLASH_LED(Green, VERY_SHORT_LED_FLASH_DURATION)        
        /* Power down and wake up */
        SAVE_SWITCH_POSITION_AND_POWER_DOWN(DEFAULT_WAIT_INTERVAL);
    } 
    /* Slow down the processor */
    AudioMoth_setClockDivider(AM_HF_CLK_DIV4);
    /* Wait final period before sample */
    if (millisecondsUntilNextSample > 0) {
        AudioMoth_delay(millisecondsUntilNextSample);
    }
    /* Enable the microphone and collect samples */
    dataReady = false;
    uint32_t numberOfBuffers = 0;
    AudioMoth_enableMicrophone(AM_NORMAL_GAIN_RANGE, AM_GAIN_MEDIUM, CLOCK_DIVIDER, ACQUISITION_CYCLES, OVERSAMPLE_RATE);
    AudioMoth_initialiseDirectMemoryAccess(primaryBuffer, secondaryBuffer, FFT_LENGTH);
    AudioMoth_delay(DELAY_BEFORE_FIRST_SAMPLE);
    AudioMoth_startMicrophoneSamples(SAMPLE_RATE);
    while (true) { 
        if (dataReady) {
            if (numberOfBuffers == NUMBER_OF_BUFFERS_TO_COLLECT - 1) AudioMoth_disableMicrophone();
            AudioMoth_setGreenLED(true);
            FFT_realTransform(dataBuffer, fftBuffer);
            AudioMoth_setGreenLED(false);
            /* Update average FFT buffer or power buffer */
            if (numberOfBuffers == 0) {
                for (uint32_t i = 0; i < FFT_HALF_LENGTH; i += 1) {
#if AVERAGE_FFT
                    meanAmplitudeBuffer[2*i] = fftBuffer[2*i];
                    meanAmplitudeBuffer[2*i+1] = fftBuffer[2*i+1];
#else
                    powerBuffer[i] = fftBuffer[2*i] * fftBuffer[2*i] + fftBuffer[2*i+1] * fftBuffer[2*i+1];
#endif
                }
            } else {
                for (uint32_t i = 0; i < FFT_HALF_LENGTH; i += 1) {
#if AVERAGE_FFT
                    meanAmplitudeBuffer[2*i] += fftBuffer[2*i];
                    meanAmplitudeBuffer[2*i+1] += fftBuffer[2*i+1];
#else
                    powerBuffer[i] += fftBuffer[2*i] * fftBuffer[2*i] + fftBuffer[2*i+1] * fftBuffer[2*i+1];
#endif
                }
            }
            /* Update counter and reset flag */
            numberOfBuffers += 1;
            dataReady = false;
        }
        if (numberOfBuffers == NUMBER_OF_BUFFERS_TO_COLLECT) break;
        /* Go to sleep */
        AudioMoth_sleep();
    }
    /* Speed up the processor */
    AudioMoth_setClockDivider(AM_HF_CLK_DIV1);
    /* Calculate and normalise the mean power */
    uint32_t amplitudeNormalisingConstant = (1 << 11) * OVERSAMPLE_RATE;
#if AVERAGE_FFT
    for (uint32_t i = 0; i < FFT_HALF_LENGTH; i += 1) {
        powerBuffer[i] = meanAmplitudeBuffer[2*i] * meanAmplitudeBuffer[2*i] + meanAmplitudeBuffer[2*i+1] * meanAmplitudeBuffer[2*i+1];
        powerBuffer[i] *= 4.0f / (float)amplitudeNormalisingConstant / (float)amplitudeNormalisingConstant / (float)NUMBER_OF_BUFFERS_TO_COLLECT / (float)NUMBER_OF_BUFFERS_TO_COLLECT;
    }
#else
    for (uint32_t i = 0; i < FFT_HALF_LENGTH; i += 1) {
        powerBuffer[i] *= 4.0f / (float)amplitudeNormalisingConstant / (float)amplitudeNormalisingConstant / (float)NUMBER_OF_BUFFERS_TO_COLLECT;
    }
#endif
    /* Append the file */
    if (WRITE_FILE) {
        AudioMoth_setRedLED(true);
        AudioMoth_enableFileSystem(AM_SD_CARD_HIGH_SPEED);
        bool success = writeDataToFile();
        if (success == false) *timeOfFirstSample = *timeOfNextSample + ACOUSTIC_SAMPLE_INTERVAL;
        AudioMoth_disableFileSystem(); 
        AudioMoth_setRedLED(false);
    }
    /* Schedule next sample */
    *timeOfNextSample += ACOUSTIC_SAMPLE_INTERVAL;
    /* Power down and wake up */
    SAVE_SWITCH_POSITION_AND_POWER_DOWN(DEFAULT_WAIT_INTERVAL);
}
