/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _SD_CARD_PLAYBACK_H_
#define _SD_CARD_PLAYBACK_H_

#include <zephyr/kernel.h>

/**
 * \brief	Sample rates supported by the sd_card_playback_wav-function.
 */
enum sd_playback_sample_rates {
	SAMPLE_RATE_48K = 48000
};

/**@brief	Figure out whether or not the sd_card_playback module is active.
 *
 * @retval	true, if active. false, if not active.
 */
bool sd_card_playback_is_active(void);

/**@brief	Play audio from .wav file from SD card. Only support for mono files.
 *
 * @param[in]	filename Name of file to be played. Path from root of SD card is accepted.
 */
void sd_card_playback_wav(char *filename, uint32_t frame_duration_ms, uint8_t bit_depth,
			  enum sd_playback_sample_rates sample_rate);

/**@brief	Play audio from .wav file from SD card.
 *
 * @param[in]	filename	Name of file to be played. Path from root of SD card is accepted.
 */
void sd_card_playback_lc3(char *filename);

/**@brief	Mix pcm data from sd_card_playback module with audio stream out.
 *
 * @param[in, out]	pcm_a	Buffer into which to mix pcm data from lc3_module.
 * @param[in]		size	Size of input buffer.
 *
 * @retval	0 on success.
 *		Otherwise, error from underlying drivers.
 */
int sd_card_playback_mix_with_stream(void *const pcm_a, size_t pcm_a_size);

/**@brief	Initialize sd_card_playback module. Create sd_card_playback thread.
 *
 * @retval	0 on success.
 *		Otherwise, error from underlying drivers.
 */
int sd_card_playback_init(void);

#endif
