
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "usb_device.h"
#include "LEPTON_ErrorCodes.h"
#include "LEPTON_AGC.h"

#include "pt.h"
#include "lepton.h"
#include "lepton_i2c.h"
#include "tmp007_i2c.h"
#include "usbd_uvc.h"
#include "usbd_uvc_if.h"
#include "circ_buf.h"

#include "tasks.h"
#include "project_config.h"

extern volatile uint8_t g_lepton_type_3;
extern struct uvc_streaming_control videoCommitControl;
extern LEP_CAMERA_PORT_DESC_T hport_desc;

lepton_buffer *completed_buffer;
uint32_t completed_frame_count;

uint8_t lepton_i2c_buffer[36];

#define RING_SIZE (4)
lepton_buffer read_lepton_buffer;
lepton_buffer lepton_AGC_buffers[RING_SIZE];

lepton_buffer *completed_frames_buf[RING_SIZE] = {0};
DECLARE_CIRC_BUF_HANDLE(completed_frames_buf);

struct rgb_to_yuv_state
{
	struct pt pt;
	lepton_buffer *restrict rgb;
};

#if defined(USART_DEBUG) || defined(GDB_SEMIHOSTING)
#define DEBUG_PRINTF(...) printf(__VA_ARGS__);
#else
#define DEBUG_PRINTF(...)
#endif

uint32_t get_lepton_buffer(lepton_buffer **buffer)
{
	if (buffer != NULL)
		*buffer = completed_buffer;
	return completed_frame_count;
}

lepton_buffer *dequeue_lepton_buffer()
{
	if (empty(CIRC_BUF_HANDLE(completed_frames_buf)))
		return NULL;
	else
		return shift(CIRC_BUF_HANDLE(completed_frames_buf));
}

uint16_t ctov(uint16_t temperature)
{
	//for high gain
	return temperature*100 + 27315;
}


uint16_t lowTh;

void init_lepton_task()
{
  int i;
  for (i = 0; i < RING_SIZE; i++)
  {
    lepton_AGC_buffers[i].number = i;
    lepton_AGC_buffers[i].status = LEPTON_STATUS_OK;
    DEBUG_PRINTF("Initialized lepton buffer %d @ %p\r\n", i, &lepton_AGC_buffers[i]);
  }

  lowTh=ctov(30);
}

static float k_to_c(uint16_t unitsKelvin)
{
	return (((float)(unitsKelvin / 100) + ((float)(unitsKelvin % 100) * 0.01f)) - 273.15f);
}

static void print_telemetry_temps(telemetry_data_l2 *telemetry)
{
	//
	uint16_t fpa_temperature_k = telemetry->fpa_temp_100k[0];
	uint16_t aux_temperature_k = telemetry->housing_temp_100k[0];

	float fpa_c = k_to_c(fpa_temperature_k);
	float aux_c = k_to_c(aux_temperature_k);

	DEBUG_PRINTF("fpa %d.%d°c, aux/housing: %d.%d°c\r\n",
				 (int)(fpa_c), (int)((fpa_c - (int)fpa_c) * 100),
				 (int)(aux_c), (int)((aux_c - (int)aux_c) * 100));
}

static lepton_buffer *current_buffer = NULL;
static lepton_buffer *current_AGC_buffer = NULL;

// AGC Reference temperature in centi Kelvin

static uint8_t AGCmethod=1;

//method 3 zones
static uint16_t miniTemp=1500+27315;
static uint16_t startZone=3200+27315;
static uint16_t endZone=4200+27315;
static uint16_t maxTemp=6000+27315;


//method ironblack
static uint16_t midtempIB=3000+27315;
static uint16_t lowtempIB=1600+27315;
static uint16_t hightempIB=4000+27315;

void applyAGC()
{
	if (transferRGB)
	{
		

		for (int i = 0; i < IMAGE_NUM_LINES; i++)
		{

			for (int j = 0; j < FRAME_LINE_LENGTH; j++)
			{
				if (AGCmethod == 0) // 3 zones
				{
					current_AGC_buffer->lines.rgb[i].data.image_data[j].r = 0;
					current_AGC_buffer->lines.rgb[i].data.image_data[j].g = 0;
					current_AGC_buffer->lines.rgb[i].data.image_data[j].b = 0;

					if (current_buffer->lines.y16[i].data.image_data[j] > miniTemp && current_buffer->lines.y16[i].data.image_data[j] < startZone)
					{
						float fact = (float)(current_buffer->lines.y16[i].data.image_data[j] - miniTemp) / (float)(startZone - miniTemp);

						current_AGC_buffer->lines.rgb[i].data.image_data[j].r = 0; // current_buffer->lines.rgb[i].data.image_data[j].r;
						current_AGC_buffer->lines.rgb[i].data.image_data[j].g = 0; // current_buffer->lines.rgb[i].data.image_data[j].g;
						current_AGC_buffer->lines.rgb[i].data.image_data[j].b = 30 + fact * 128;
					}
					else if (current_buffer->lines.y16[i].data.image_data[j] > startZone && current_buffer->lines.y16[i].data.image_data[j] < endZone)
					{
						float fact = (float)(current_buffer->lines.y16[i].data.image_data[j] - startZone) / (float)(endZone - startZone);

						current_AGC_buffer->lines.rgb[i].data.image_data[j].r = 0;				 // current_buffer->lines.rgb[i].data.image_data[j].r;
						current_AGC_buffer->lines.rgb[i].data.image_data[j].g = 90 + fact * 128; // current_buffer->lines.rgb[i].data.image_data[j].g;
						current_AGC_buffer->lines.rgb[i].data.image_data[j].b = 90 + fact * 128;
					}
					else if (current_buffer->lines.y16[i].data.image_data[j] > endZone && current_buffer->lines.y16[i].data.image_data[j] < maxTemp)
					{
						float fact = (float)(current_buffer->lines.y16[i].data.image_data[j] - endZone) / (float)(maxTemp - endZone);

						current_AGC_buffer->lines.rgb[i].data.image_data[j].r = 128 + fact * 128; // current_buffer->lines.rgb[i].data.image_data[j].r;
						current_AGC_buffer->lines.rgb[i].data.image_data[j].g = 0;				  // current_buffer->lines.rgb[i].data.image_data[j].g;
						current_AGC_buffer->lines.rgb[i].data.image_data[j].b = 0;
					}
				}
				else if (AGCmethod == 1) // IronBlack mapping
				{
					current_AGC_buffer->lines.rgb[i].data.image_data[j].r = colormap_ironblack.colormap[0];
					current_AGC_buffer->lines.rgb[i].data.image_data[j].g = colormap_ironblack.colormap[0 + 1];
					current_AGC_buffer->lines.rgb[i].data.image_data[j].b = colormap_ironblack.colormap[0 + 2];

					if (current_buffer->lines.y16[i].data.image_data[j] >= lowtempIB && current_buffer->lines.y16[i].data.image_data[j] < midtempIB)
					{

						uint16_t loc = 128 * (float)(current_buffer->lines.y16[i].data.image_data[j] - lowtempIB) / (float)(midtempIB - lowtempIB);
						current_AGC_buffer->lines.rgb[i].data.image_data[j].r = colormap_ironblack.colormap[loc*3];
						current_AGC_buffer->lines.rgb[i].data.image_data[j].g = colormap_ironblack.colormap[loc*3 + 1];
						current_AGC_buffer->lines.rgb[i].data.image_data[j].b = colormap_ironblack.colormap[loc*3 + 2];
					}
					else if (current_buffer->lines.y16[i].data.image_data[j] >= midtempIB && current_buffer->lines.y16[i].data.image_data[j] < hightempIB)
					{

						uint16_t loch =  128 *(float)(current_buffer->lines.y16[i].data.image_data[j] - midtempIB) / (float)(hightempIB - midtempIB);
						current_AGC_buffer->lines.rgb[i].data.image_data[j].r = colormap_ironblack.colormap[3*128+loch*3];
						current_AGC_buffer->lines.rgb[i].data.image_data[j].g = colormap_ironblack.colormap[3*128+loch*3 + 1];
						current_AGC_buffer->lines.rgb[i].data.image_data[j].b = colormap_ironblack.colormap[3*128+loch*3 + 2];
					}
					else
					{
						current_AGC_buffer->lines.rgb[i].data.image_data[j].r = colormap_ironblack.colormap[255 * 3];
						current_AGC_buffer->lines.rgb[i].data.image_data[j].g = colormap_ironblack.colormap[255 * 3 + 1];
						current_AGC_buffer->lines.rgb[i].data.image_data[j].b = colormap_ironblack.colormap[255 * 3 + 2];
					}
				}
			}
		}
		current_AGC_buffer->number = current_buffer->number;
		current_AGC_buffer->segment = current_buffer->segment;
		current_AGC_buffer->status = current_buffer->status;
	}
	else
	{
		for (int i = 0; i < IMAGE_NUM_LINES; i++)
		{
			for (int j = 0; j < FRAME_LINE_LENGTH; j++)
			{

				current_AGC_buffer->lines.y16[i].data.image_data[j] = current_buffer->lines.y16[i].data.image_data[j];
			}
		}
		current_AGC_buffer->number = current_buffer->number;
		current_AGC_buffer->segment = current_buffer->segment;
		current_AGC_buffer->status = current_buffer->status;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	static int current_buffer_index = 0;
	lepton_buffer *buffer = &read_lepton_buffer;
	current_buffer = buffer;
	lepton_buffer *bufferAGC = &lepton_AGC_buffers[current_buffer_index];
	current_AGC_buffer = bufferAGC;
	current_buffer_index = ((current_buffer_index + 1) % RING_SIZE);
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
}


	PT_THREAD(lepton_task(struct pt * pt))
	{
		PT_BEGIN(pt);

		static uint32_t curtick = 0;
		static uint32_t last_tick = 0;
		static uint32_t last_logged_count = 0;
		static uint32_t current_frame_count = 0;
		static int transferring_timer = 0;
		static uint8_t current_segment = 0;
		static uint8_t last_end_line = 0;
		static uint8_t has_started_a_stream = 0;
		curtick = last_tick = HAL_GetTick();

#ifdef THERMAL_DATA_UART
	enable_telemetry();
	enable_raw14();
#endif

	while (1)
	{
#ifndef THERMAL_DATA_UART
		if (g_uvc_stream_status == 0)
		{
			lepton_low_power();
			if (has_started_a_stream)
			{
				if (g_format_y16)
				{
					// no cleanup after y16
				}
				else
				{
					//	disable_rgb888();
				}
			}

			// Start slow blink (1 Hz)
			while (g_uvc_stream_status == 0)
			{
				HAL_GPIO_TogglePin(SYSTEM_LED_GPIO_Port, SYSTEM_LED_Pin);

				transferring_timer = HAL_GetTick();
				PT_YIELD_UNTIL(pt, g_uvc_stream_status != 0 || (HAL_GetTick() - transferring_timer) > 500);
			}

			g_format_y16 = (videoCommitControl.bFormatIndex == VS_FMT_INDEX(Y16));

			if (g_format_y16)
			{
				if (videoCommitControl.bFrameIndex == VS_FRAME_INDEX_TELEMETRIC)
					enable_telemetry();
				else
					disable_telemetry();
				disable_lepton_agc();
				enable_raw14();
				transferRGB=0;
			}
			else
			{
				disable_telemetry();
				disable_lepton_agc();
				enable_raw14();
				transferRGB=1;
				//enable_rgb888((LEP_PCOLOR_LUT_E)-1); // -1 means attempt to continue using the current palette (PcolorLUT)
			}

			// Reinitialize limits AGC
			LEP_SetAgcHeqClipLimitHigh(&hport_desc, 4000 + 27315);
			LEP_SetAgcHeqClipLimitLow(&hport_desc, 1600 + 27315);
			LEP_SetAgcHeqMidPoint(&hport_desc, 3000 + 27315);

			has_started_a_stream = 1;

			// flush out any old data
			while (dequeue_lepton_buffer() != NULL)
			{
			}

			// Make sure we're not about to service an old irq when the interrupts are re-enabled
			__HAL_GPIO_EXTI_CLEAR_IT(EXTI15_10_IRQn);

			lepton_power_on();
		}
#endif

		HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

		PT_WAIT_UNTIL(pt, current_buffer != NULL);

		lepton_transfer(current_buffer, IMAGE_NUM_LINES + g_telemetry_num_lines);

		transferring_timer = HAL_GetTick();
		PT_YIELD_UNTIL(pt, current_buffer->status != LEPTON_STATUS_TRANSFERRING || ((HAL_GetTick() - transferring_timer) > 200));

		if (complete_lepton_transfer(current_buffer) != LEPTON_STATUS_OK)
		{
			DEBUG_PRINTF("Lepton transfer failed: %d\r\n", current_buffer->status);
			current_buffer = NULL;
			continue;
		}

		current_frame_count++;

		if (g_format_y16 || transferRGB)
		{
			current_segment = ((current_buffer->lines.y16[IMAGE_OFFSET_LINES + 20].header[0] & 0x7000) >> 12);
			last_end_line = (current_buffer->lines.y16[IMAGE_OFFSET_LINES + IMAGE_NUM_LINES + g_telemetry_num_lines - 1].header[0] & 0x00ff);
		}
		else
		{
			current_segment = ((current_buffer->lines.rgb[IMAGE_OFFSET_LINES + 20].header[0] & 0x7000) >> 12);
			last_end_line = (current_buffer->lines.rgb[IMAGE_OFFSET_LINES + IMAGE_NUM_LINES + g_telemetry_num_lines - 1].header[0] & 0x00ff);
		}

		current_buffer->segment = current_segment;

		if (last_end_line != (IMAGE_NUM_LINES + g_telemetry_num_lines - 1))
		{
			// flush out any old data since it's no good
			while (dequeue_lepton_buffer() != NULL)
			{
			}

			if (current_frame_count > 2)
			{
				uint16_t last_header;

				DEBUG_PRINTF("Synchronization lost, status: %d, last end line %d\r\n",
							 current_buffer->status, last_end_line);

				transferring_timer = HAL_GetTick();
				PT_WAIT_UNTIL(pt, (HAL_GetTick() - transferring_timer) > 185);

				// transfer packets until we've actually re-synchronized
				do
				{
					lepton_transfer(current_buffer, 1);

					transferring_timer = HAL_GetTick();
					PT_YIELD_UNTIL(pt, current_buffer->status != LEPTON_STATUS_TRANSFERRING || ((HAL_GetTick() - transferring_timer) > 200));

					last_header = ((g_format_y16 || transferRGB) ?
							current_buffer->lines.y16[0].header[0] :
							current_buffer->lines.rgb[0].header[0]);

				} while (current_buffer->status == LEPTON_STATUS_OK && (last_header & 0x0f00) == 0x0f00);

				// we picked up the start of a new packet, so read the rest of it in
				lepton_transfer(current_buffer, IMAGE_NUM_LINES + g_telemetry_num_lines - 1);

				transferring_timer = HAL_GetTick();
				PT_YIELD_UNTIL(pt, current_buffer->status != LEPTON_STATUS_TRANSFERRING || ((HAL_GetTick() - transferring_timer) > 200));

				// Make sure we're not about to service an old irq when the interrupts are re-enabled
				__HAL_GPIO_EXTI_CLEAR_IT(EXTI15_10_IRQn);

				current_frame_count = 0;
			}

			current_buffer = NULL;

			continue;
		}

		if (((curtick = HAL_GetTick()) - last_tick) > 3000)
		{
#ifdef PRINT_FPS
			DEBUG_PRINTF("fps: %lu, last end line: %d, frame #%lu, buffer %p\r\n",
						 (current_frame_count - last_logged_count) / 3,
						 last_end_line,
						 current_frame_count, current_buffer);
#endif

			if (g_telemetry_num_lines > 0 && g_lepton_type_3 == 0)
			{
				if (g_format_y16 || transferRGB)
					print_telemetry_temps(&current_buffer->lines.y16[TELEMETRY_OFFSET_LINES].data.telemetry_data);
				else
					print_telemetry_temps(&current_buffer->lines.rgb[TELEMETRY_OFFSET_LINES].data.telemetry_data);
			}

#if defined(TMP007)
			read_tmp007_regs();
#endif

			last_tick = curtick;
			last_logged_count = current_frame_count;
		}


	applyAGC();
		// Need to update completed buffer for clients?
		if (g_lepton_type_3 == 0 || (current_segment > 0 && current_segment <= 4))
		{
			static int row;

			completed_buffer = current_AGC_buffer;
			completed_frame_count = current_frame_count;

			HAL_GPIO_TogglePin(SYSTEM_LED_GPIO_Port, SYSTEM_LED_Pin);

		/*	if (!g_format_y16)
			{
				for (row = 0; row < (IMAGE_NUM_LINES + g_telemetry_num_lines); row++)
				{
					uint16_t *lineptr = (uint16_t *)completed_buffer->lines.rgb[IMAGE_OFFSET_LINES + row].data.image_data;
					while (lineptr < (uint16_t *)&completed_buffer->lines.rgb[IMAGE_OFFSET_LINES + row].data.image_data[FRAME_LINE_LENGTH])
					{
						uint8_t *bytes = (uint8_t *)lineptr;
						*lineptr++ = bytes[0] << 8 | bytes[1];
					}
					PT_YIELD(pt);
				}
			}
*/
			if (!full(CIRC_BUF_HANDLE(completed_frames_buf)))
				push(CIRC_BUF_HANDLE(completed_frames_buf), completed_buffer);
		}
		if (current_frame_count % 100 == 0)
		{
			LEP_GetAgcHeqClipLimitHigh(&hport_desc, &hightempIB);
			
		}
		if (current_frame_count % 100 == 30)
		{
			
			LEP_GetAgcHeqClipLimitLow(&hport_desc, &lowtempIB);
		
		}
		if (current_frame_count % 100 == 60)
		{
		
			LEP_GetAgcHeqMidPoint(&hport_desc, &midtempIB);
		}
		current_buffer = NULL;
		current_AGC_buffer=NULL;
	}
	PT_END(pt);
}

static inline uint8_t clamp(float x)
{
	if (x < 0)
		return 0;
	else if (x > 255)
		return 255;
	else
		return (uint8_t)x;
}

void rgb2yuv(const rgb_t val, uint8_t *y, uint8_t *u, uint8_t *v)
{
	float r = val.r, g = val.g, b = val.b;

	float y1 = 0.299f * r + 0.587f * g + 0.114f * b;

	*y = clamp(0.859f * y1 + 16.0f);
	if (u)
		*u = clamp(0.496f * (b - y1) + 128.0f);
	if (v)
		*v = clamp(0.627f * (r - y1) + 128.0f);
}
