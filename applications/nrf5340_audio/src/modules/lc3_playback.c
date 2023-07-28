#include "lc3_playback.h"
#include <stdint.h>
#include <zephyr/sys/ring_buffer.h>
#include "sd_card.h"
#include "hw_codec.h"
#include "sw_codec_lc3.h"
#include "sw_codec_select.h"
#include "pcm_stream_channel_modifier.h"
#include "pcm_mix.h"
#include "audio_i2s.h"
#include "sw_codec_lc3.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lc3_playback, 4); /* For draft PR feedback: going to change this line */

#define RING_BUF_SIZE		1920 /* This can be modified */
#define lc3_playback_STACK_SIZE 4096

/*File structure of the LC3 encoded files*/
struct lc3BinaryHdr_t {
	uint16_t fileId;		/* Constant value, 0xCC1C */
	uint16_t hdrSize;		/* Header size, 0x0012 */
	uint16_t sampleRate_divided100; /* Sample frequency / 100 */
	uint16_t bitRate_divided100;	/* Bit rate / 100 (total for all channels) */
	uint16_t channels;		/* Number of channels */
	uint16_t frameMs_times100;	/* Frame duration in ms * 100 */
	uint16_t rfu;			/* RFU value */
	uint16_t signalLen;		/* Number of samples in signal, 16 LSB */
	uint16_t signalLenRed;		/* Number of samples in signal, 16 MSB (>> 16) */
};

static struct lc3BinaryHdr_t header;
static size_t header_size = sizeof(header);
static uint16_t lc3_frame_length;
static uint16_t lc3_frames_num;
static uint16_t pcm_mono_frame_size;
static bool lc3_playback_active;

RING_BUF_DECLARE(m_ringbuf_sound_data_lc3, RING_BUF_SIZE);
K_SEM_DEFINE(m_sem_load_from_buf_lc3, 1, 1);
K_MUTEX_DEFINE(mtx_ringbuf);

static struct k_thread lc3_playback_thread_data;
static k_tid_t lc3_playback_thread_id;

K_THREAD_STACK_DEFINE(lc3_playback_thread_stack, lc3_playback_STACK_SIZE);

bool lc3_playback_is_active(void)
{
	return lc3_playback_active;
}

int lc3_playback_buffer_set(uint8_t *buf, size_t size)
{
	uint16_t size_gotten;

	k_mutex_lock(&mtx_ringbuf, K_FOREVER);
	size_gotten = ring_buf_get(&m_ringbuf_sound_data_lc3, buf, size);
	k_mutex_unlock(&mtx_ringbuf);
	if (ring_buf_space_get(&m_ringbuf_sound_data_lc3) >= pcm_mono_frame_size) {
		k_sem_give(&m_sem_load_from_buf_lc3);
	}
	return 0;
}

int lc3_playback_mix_with_stream(void *const pcm_a, size_t pcm_a_size)
{
	int ret;
	uint8_t pcm_b[pcm_mono_frame_size];

	ret = lc3_playback_buffer_set(pcm_b, pcm_mono_frame_size);
	if (ret < 0) {
		LOG_ERR("Error when loading pcm data into buffer. Ret: %d", ret);
		return ret;
	}
	pcm_mix(pcm_a, pcm_a_size, pcm_b, pcm_mono_frame_size, B_MONO_INTO_A_STEREO_L);
	return 0;
}

static int lc3_playback_buffer_to_ringbuffer(uint8_t *buffer, size_t numbytes)
{
	/* Burde kanskje ha en sjekk her for å sjekke at numbytes gir mening */
	static uint8_t *buf_ptr;

	k_mutex_lock(&mtx_ringbuf, K_FOREVER);
	numbytes = ring_buf_put_claim(&m_ringbuf_sound_data_lc3, &buf_ptr, numbytes);

	for (int i = 0; i < numbytes; i++) {
		buf_ptr[i] = buffer[i];
	}
	ring_buf_put_finish(&m_ringbuf_sound_data_lc3, numbytes);
	k_mutex_unlock(&mtx_ringbuf);
	return numbytes;
}

static int lc3_playback_header_read(const char *filename, const char *path_to_file)
{
	LOG_DBG("In header read");
	int ret;
	size_t lc3_frame_length_size = sizeof(lc3_frame_length);
	uint32_t num_samples;

	ret = sd_card_segment_open(filename, path_to_file);
	if (ret < 0) {
		LOG_ERR("Error when trying to open file on SD card. Return value: %d", ret);
		return ret;
	}

	/* Read the header */
	ret = sd_card_segment_read((char *)&header, &header_size);
	if (ret < 0) {
		LOG_ERR("Error when trying to peek at file segment on SD card. Return value: %d",
			ret);
		return ret;
	}

	pcm_mono_frame_size = 2 * header.sampleRate_divided100 * header.frameMs_times100 / 1000;
	num_samples = (header.signalLenRed << 16) + header.signalLen;
	lc3_frames_num = 2 * num_samples / pcm_mono_frame_size;

	/* Read the frame length */
	ret = sd_card_segment_peek((char *)&lc3_frame_length, &lc3_frame_length_size);
	if (ret < 0) {
		LOG_ERR("Error when trying to peek at file  segment on SD card. Return value: %d",
			ret);
		return ret;
	}

	ret = sd_card_segment_close();
	if (ret < 0) {
		LOG_ERR("Error when trying to close file on SD card. Return value: %d", ret);
		return ret;
	}

	return 0;
}

static int lc3_playback_play(const char *filename, const char *path_to_file)
{
	int ret;

	ret = lc3_playback_header_read(filename, path_to_file);
	if (ret < 0) {
		LOG_ERR("Audio Lc3 header read failed. Return value: %d", ret);
		return ret;
	}

	uint16_t pcm_mono_frame[pcm_mono_frame_size / 2];
	uint16_t pcm_mono_write_size;
	uint16_t lc3_frame[lc3_frame_length];
	size_t lc3_fr_len = lc3_frame_length; /* Converting to size_t */
	size_t lc3_fr_len_size = sizeof(lc3_frame_length);

	/* First, open file on SD card */
	ret = sd_card_segment_open(filename, path_to_file);
	if (ret < 0) {
		LOG_ERR("Error when trying to open file on SD card. Return value: %d", ret);
		return ret;
	}

	/* Then, skip the header */
	ret = sd_card_segment_skip(&header_size);
	if (ret < 0) {
		LOG_ERR("Error when trying to open file on SD card. Return value: %d", ret);
		return ret;
	}
	lc3_playback_active = true;
	for (uint32_t i = 0; i < lc3_frames_num; i++) {
		/* Skip the frame length info to get to the audio data */
		ret = sd_card_segment_skip(&lc3_fr_len_size);
		if (ret < 0) {
			LOG_ERR("Error when trying to skip file on SD card. Return value: %d", ret);
			return ret;
		}

		/* Read the audio data frame to be encoded */
		ret = sd_card_segment_read((char *)lc3_frame, &lc3_fr_len);
		if (ret < 0) {
			LOG_ERR("Something went wrong when reading from SD card. Return value: %d",
				ret);
			return ret;
		}

		/* Decode audio data frame*/

		ret = sw_codec_lc3_dec_run((char *)lc3_frame, lc3_frame_length, pcm_mono_frame_size,
					   1, pcm_mono_frame, &pcm_mono_write_size, false);
		if (ret < 0) {
			LOG_ERR("Error when running decoder. Return value: %d\n", ret);
			return ret;
		}

		/* Wait until there is enough space in the ringbuffer */
		k_sem_take(&m_sem_load_from_buf_lc3, K_FOREVER);
		lc3_playback_buffer_to_ringbuffer((char *)pcm_mono_frame, pcm_mono_write_size);
	}
	ret = sd_card_segment_close();
	if (ret < 0) {
		LOG_ERR("Error when closing file. Return value: %d", ret);
		return ret;
	}
	lc3_playback_active = false;
	return 0;
}

static void lc3_playback_thread(void *arg1, void *arg2, void *arg3)
{
	while (!sw_codec_is_initialized()) {
		k_msleep(100);
	}
	lc3_playback_play("enc_3.bin", "");
}

int lc3_playback_init(void)
{
	int ret;

	lc3_playback_thread_id =
		k_thread_create(&lc3_playback_thread_data, lc3_playback_thread_stack,
				lc3_playback_STACK_SIZE, (k_thread_entry_t)lc3_playback_thread,
				NULL, NULL, NULL, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
	ret = k_thread_name_set(lc3_playback_thread_id, "lc3_playback");
	if (ret < 0) {
		LOG_ERR("Failed");
		return ret;
	}
	return 0;
}
