#include "sd_card_playback.h"
#include "sd_card.h"
#include "sw_codec_lc3.h"
#include "sw_codec_select.h"
#include "pcm_mix.h"
#include <stdint.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/shell/shell.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sd_card_playback, CONFIG_MODULE_SD_CARD_PLAYBACK_LOG_LEVEL);

#define RING_BUF_SIZE_BYTES	    1920
#define SD_CARD_PLAYBACK_STACK_SIZE 4096
#define MAX_FILENAME_LEN	    12
#define MAX_PATH_LEN		    (CONFIG_FS_FATFS_MAX_LFN - MAX_FILENAME_LEN)
#define LIST_FILES_BUF_SIZE	    512
#define SIZEOF_FR_LEN_INFO	    2

/* File structure of the LC3 encoded files */
struct lc3_binary_hdr_t {
	uint16_t file_id;	      /* Constant value, 0xCC1C */
	uint16_t hdr_size;	      /* Header size, 0x0012 */
	uint16_t sample_rate;	      /* Sample frequency / 100 */
	uint16_t bit_rate;	      /* Bit rate / 100 (total for all channels) */
	uint16_t channels;	      /* Number of channels */
	uint16_t frame_ms;	      /* Frame duration in ms * 100 */
	uint16_t reserved_future_use; /* RFU value */
	uint16_t signal_len_lsb;      /* Number of samples in signal, 16 LSB */
	uint16_t signal_len_msb;      /* Number of samples in signal, 16 MSB (>> 16) */
};

struct wav_playback_config {
	uint8_t frame_duration_ms;
	uint8_t bit_depth;
	enum sd_playback_sample_rates sample_rate;
};

struct lc3_playback_config {
	uint16_t lc3_frames_num;
	uint16_t lc3_frame_length;
	size_t lc3_frame_length_size;
};

enum audio_formats {
	SD_CARD_PLAYBACK_WAV,
	SD_CARD_PLAYBACK_LC3,
};

RING_BUF_DECLARE(m_ringbuf_sound_data_lc3, RING_BUF_SIZE_BYTES);
K_SEM_DEFINE(m_sem_load_from_buf_lc3, 1, 1);
K_MUTEX_DEFINE(mtx_ringbuf);
K_SEM_DEFINE(m_sem_playback, 0, 2);
K_THREAD_STACK_DEFINE(sd_card_playback_thread_stack, SD_CARD_PLAYBACK_STACK_SIZE);

/* Thread */
static struct k_thread sd_card_playback_thread_data;
static k_tid_t sd_card_playback_thread_id;

/* Playback */
static bool sd_card_playback_active;
static char *playback_file_name;
static enum audio_formats playback_file_format;
static uint16_t pcm_frame_size;
static uint16_t pcm_mono_write_size;
static char playback_file_path[MAX_PATH_LEN] = "";
static struct lc3_binary_hdr_t lc3_file_header;
static struct wav_playback_config wav_playback_cfg = {.frame_duration_ms = 10, .bit_depth = 16};
static struct lc3_playback_config lc3_playback_cfg = {.lc3_frame_length_size = SIZEOF_FR_LEN_INFO};

static struct fs_file_t f_seg_read_entry;

static int sd_card_playback_buffer_set(uint8_t *buf, size_t *size)
{
	uint16_t write_size;

	k_mutex_lock(&mtx_ringbuf, K_FOREVER);
	write_size = ring_buf_get(&m_ringbuf_sound_data_lc3, buf, *size);
	k_mutex_unlock(&mtx_ringbuf);
	if (ring_buf_space_get(&m_ringbuf_sound_data_lc3) >= pcm_frame_size) {
		k_sem_give(&m_sem_load_from_buf_lc3);
	}

	*size = write_size;

	return 0;
}

static int sd_card_playback_buffer_to_ringbuffer(uint8_t *buffer, size_t numbytes)
{
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

static int sd_card_playback_header_read(const char *filename)
{
	int ret;
	size_t lc3_file_header_size = sizeof(lc3_file_header);

	ret = sd_card_open(filename, &f_seg_read_entry);
	if (ret < 0) {
		LOG_ERR("Error when trying to open file on SD card. Return value: %d", ret);
		return ret;
	}

	/* Read the header */
	ret = sd_card_read((char *)&lc3_file_header, &lc3_file_header_size, &f_seg_read_entry);
	if (ret < 0) {
		LOG_ERR("Error when trying to peek at file segment on SD card. Return value: %d",
			ret);
		return ret;
	}

	return 0;
}

static int sd_card_playback_thread_play_lc3(void)
{
	int ret;

	ret = sd_card_playback_header_read(playback_file_name);
	if (ret < 0) {
		LOG_ERR("Audio Lc3 header read failed. Return value: %d", ret);
		return ret;
	}

	pcm_frame_size = 2 * lc3_file_header.sample_rate * lc3_file_header.frame_ms / 1000;
	lc3_playback_cfg.lc3_frames_num =
		2 * ((lc3_file_header.signal_len_msb << 16) + lc3_file_header.signal_len_lsb) /
		pcm_frame_size;

	uint8_t pcm_mono_frame[pcm_frame_size];

	sd_card_playback_active = true;
	for (uint32_t i = 0; i < lc3_playback_cfg.lc3_frames_num; i++) {
		/* Skip the frame length info to get to the audio data */
		ret = sd_card_read((char *)&lc3_playback_cfg.lc3_frame_length,
				   &lc3_playback_cfg.lc3_frame_length_size, &f_seg_read_entry);
		if (ret < 0) {
			LOG_ERR("Error when trying to skip file on SD card. Return value: %d", ret);
			return ret;
		}
		uint8_t lc3_frame[lc3_playback_cfg.lc3_frame_length];
		size_t lc3_fr_len = lc3_playback_cfg.lc3_frame_length;
		/* Read the audio data frame to be encoded */
		ret = sd_card_read((char *)lc3_frame, &lc3_fr_len, &f_seg_read_entry);
		if (ret < 0) {
			LOG_ERR("Something went wrong when reading from SD card. Return value: %d",
				ret);
			return ret;
		}
		/* Decode audio data frame */
		ret = sw_codec_lc3_dec_run((char *)lc3_frame, lc3_playback_cfg.lc3_frame_length,
					   pcm_frame_size, 1, pcm_mono_frame, &pcm_mono_write_size,
					   false);
		if (ret < 0) {
			LOG_ERR("Error when running decoder. Return value: %d\n", ret);
			return ret;
		}
		/* Wait until there is enough space in the ringbuffer */
		k_sem_take(&m_sem_load_from_buf_lc3, K_FOREVER);
		sd_card_playback_buffer_to_ringbuffer((char *)pcm_mono_frame, pcm_mono_write_size);
	}
	ret = sd_card_close(&f_seg_read_entry);
	if (ret < 0) {
		LOG_ERR("Error when closing file. Return value: %d", ret);
		return ret;
	}
	sd_card_playback_active = false;
	return 0;
}

static int sd_card_playback_thread_play_wav(void)
{
	int ret;

	size_t pcm_wav_mono_frame_size = wav_playback_cfg.frame_duration_ms *
					 wav_playback_cfg.sample_rate / 1000 *
					 wav_playback_cfg.bit_depth / 8;
	pcm_frame_size = pcm_wav_mono_frame_size;
	uint8_t pcm_wav_mono_frame[pcm_wav_mono_frame_size];

	ret = sd_card_open(playback_file_name, &f_seg_read_entry);
	if (ret < 0) {
		LOG_ERR("Error when trying to open file on SD card. Return value: %d", ret);
		return ret;
	}

	sd_card_playback_active = true;
	while (sd_card_playback_active) {

		ret = sd_card_read(pcm_wav_mono_frame, &pcm_wav_mono_frame_size, &f_seg_read_entry);
		if (ret < 0) {
			LOG_ERR("Error when trying to read file on SD card. Return value: %d", ret);
			return ret;
		}
		if (pcm_wav_mono_frame_size < pcm_frame_size) {
			sd_card_playback_active = false;
		}
		/* Wait until there is enough space in the ringbuffer */
		if (sd_card_playback_active) {
			k_sem_take(&m_sem_load_from_buf_lc3, K_FOREVER);
			sd_card_playback_buffer_to_ringbuffer(pcm_wav_mono_frame,
							      pcm_wav_mono_frame_size);
		}
	}
	ret = sd_card_close(&f_seg_read_entry);
	if (ret < 0) {
		LOG_ERR("Error when closing file. Return value: %d", ret);
		return ret;
	}
	sd_card_playback_active = false;
	return 0;
}

static void sd_card_playback_thread(void *arg1, void *arg2, void *arg3)
{
	while (!sw_codec_is_initialized()) {
		k_msleep(100);
	}
	while (1) {
		k_sem_take(&m_sem_playback, K_FOREVER);
		switch (playback_file_format) {
		case SD_CARD_PLAYBACK_WAV:
			sd_card_playback_thread_play_wav();
			break;
		case SD_CARD_PLAYBACK_LC3:
			sd_card_playback_thread_play_lc3();
			break;
		}
	}
}

bool sd_card_playback_is_active(void)
{
	return sd_card_playback_active;
}

void sd_card_playback_wav(char *filename, uint32_t frame_duration_ms, uint8_t bit_depth,
			  enum sd_playback_sample_rates sample_rate)
{
	playback_file_format = SD_CARD_PLAYBACK_WAV;
	playback_file_name = filename;
	wav_playback_cfg.frame_duration_ms = frame_duration_ms;
	wav_playback_cfg.bit_depth = bit_depth;
	wav_playback_cfg.sample_rate = sample_rate;
	k_sem_give(&m_sem_playback);
}

void sd_card_playback_lc3(char *filename)
{
	playback_file_format = SD_CARD_PLAYBACK_LC3;
	playback_file_name = filename;
	k_sem_give(&m_sem_playback);
}

int sd_card_playback_mix_with_stream(void *const pcm_a, size_t pcm_a_size)
{
	int ret;
	uint8_t pcm_b[pcm_frame_size];
	size_t write_size = pcm_frame_size;

	ret = sd_card_playback_buffer_set(pcm_b, &write_size);
	if (ret < 0) {
		LOG_ERR("Error when loading pcm data into buffer. Ret: %d", ret);
		return ret;
	}
	if (write_size == pcm_frame_size) {
		pcm_mix(pcm_a, pcm_a_size, pcm_b, pcm_frame_size, B_MONO_INTO_A_STEREO_L);
	} else {
		LOG_WRN("Size written to buffer does not equal the requested size");
	}
	return 0;
}

int sd_card_playback_init(void)
{
	int ret;

	sd_card_playback_thread_id = k_thread_create(
		&sd_card_playback_thread_data, sd_card_playback_thread_stack,
		SD_CARD_PLAYBACK_STACK_SIZE, (k_thread_entry_t)sd_card_playback_thread, NULL, NULL,
		NULL, K_PRIO_PREEMPT(4), 0, K_NO_WAIT);
	ret = k_thread_name_set(sd_card_playback_thread_id, "sd_card_playback");
	if (ret < 0) {
		return ret;
	}
	return 0;
}

/* Shell functions */
static int cmd_play_wav_file(const struct shell *shell, size_t argc, char **argv)
{
	char file_loc[MAX_PATH_LEN + MAX_FILENAME_LEN] = "";

	strcat(file_loc, playback_file_path);
	strcat(file_loc, argv[1]);
	sd_card_playback_wav(file_loc, wav_playback_cfg.frame_duration_ms,
			     wav_playback_cfg.bit_depth, SAMPLE_RATE_48K);
	return 0;
}

static int cmd_play_lc3_file(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		LOG_ERR("Number of arguments given is incorrect. Number of arguments received: %d. "
			"Expected: 2",
			(int)argc);
		return -EINVAL;
	}
	char file_loc[MAX_PATH_LEN + MAX_FILENAME_LEN] = "";

	strcat(file_loc, playback_file_path);
	strcat(file_loc, argv[1]);
	sd_card_playback_lc3(file_loc);
	return 0;
}

static int cmd_change_dir(const struct shell *shell, size_t argc, char **argv)
{
	/* Remember to change this to make sure you dont navigate to a nonexistent dir */
	if (argv[1][0] == '/') {
		playback_file_path[0] = '\0';
		shell_print(shell, "Current directory: root");
	} else {
		strcat(playback_file_path, argv[1]);
		strcat(playback_file_path, "/");
		shell_print(shell, "Current directory: %s", playback_file_path);
	}
	return 0;
}

/* This function needs a lot of stack size. Shell stack size has to be 4096 instead of 1024 bc of
 * this function. The reason for this is probably the sd_card_list_files function
 */
static int cmd_list_files(const struct shell *shell, size_t argc, char **argv)
{
	int ret;
	char buf[LIST_FILES_BUF_SIZE];
	size_t buf_size = LIST_FILES_BUF_SIZE;

	ret = sd_card_list_files(playback_file_path, buf, &buf_size);
	if (ret < 0) {
		shell_print(shell, "Something went wrong!\n");
		return ret;
	}
	shell_print(shell, "%s", buf);
	return 0;
}

/* Creating subcommands (level 1 command) array for command "demo". */
SHELL_STATIC_SUBCMD_SET_CREATE(
	sd_card_playback_cmd,
	SHELL_COND_CMD(CONFIG_SHELL, play_lc3, NULL, "Play lc3 file.", cmd_play_lc3_file),
	SHELL_COND_CMD(CONFIG_SHELL, play_wav, NULL, "Play wav file.", cmd_play_wav_file),
	SHELL_COND_CMD(CONFIG_SHELL, cd, NULL, "Change directory. ", cmd_change_dir),
	SHELL_COND_CMD(CONFIG_SHELL, list_files, NULL, "List files ", cmd_list_files),
	SHELL_SUBCMD_SET_END);
/* Creating root (level 0) command "demo" without a handler */
SHELL_CMD_REGISTER(sd_card_playback, &sd_card_playback_cmd, "Play audio files from SD card", NULL);
