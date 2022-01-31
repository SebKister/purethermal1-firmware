#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

#include "lepton.h"
#include "lepton_i2c.h"
#include "project_config.h"

#include "LEPTON_SDK.h"
#include "LEPTON_SYS.h"
#include "LEPTON_AGC.h"
#include "LEPTON_VID.h"
#include "LEPTON_OEM.h"
#include "LEPTON_RAD.h"

#if defined(USART_DEBUG) || defined(GDB_SEMIHOSTING)
#define DEBUG_PRINTF(...) printf( __VA_ARGS__);
#else
#define DEBUG_PRINTF(...)
#endif

// HAL_OK       = 0x00,
//  HAL_ERROR    = 0x01,
//  HAL_BUSY     = 0x02,
//  HAL_TIMEOUT  = 0x03

extern I2C_HandleTypeDef hi2c1;
LEP_CAMERA_PORT_DESC_T hport_desc;

extern volatile uint8_t g_lepton_type_3;

static void set_lepton_type()
{
  LEP_RESULT result;
  LEP_OEM_PART_NUMBER_T part_number;
  result = LEP_GetOemFlirPartNumber(&hport_desc, &part_number);

  // 500-0643-00 : 50 deg (l2)
  // 500-0659-01 : shuttered 50 deg (l2)
  // 500-0690-00 : 25 deg (l2)
  // 500-0763-01 : shuttered 50 deg + radiometric (l2.5)
  // 500-0726-01 : shuttered 50 deg (l3)

  if (result != LEP_OK ||
      strncmp(part_number.value, "500-06xx", 6) == 0 ||
      strncmp(part_number.value, "500-0763", 8) == 0)
  {
    g_lepton_type_3 = 0;
  }
  else
  {
    // let default case be l3, because this will be more likely to cover new products
    g_lepton_type_3 = 1;
  }

  LEP_SetOemGpioVsyncPhaseDelay(&hport_desc,LEP_OEM_VSYNC_DELAY_PLUS_2);
  LEP_SetOemGpioMode(&hport_desc, LEP_OEM_GPIO_MODE_VSYNC);
}
typedef struct { const uint8_t colormap[256 * 3]; } colormap_t;
const colormap_t colormap_ironblack = { {255, 255, 255, 253, 253, 253, 251, 251, 251, 249, 249, 249, 247, 247, 247, 245, 245, 245, 243, 243, 243, 241, 241, 241, 239, 239, 239, 237, 237, 237, 235, 235, 235, 233, 233, 233, 231, 231, 231, 229, 229, 229, 227, 227, 227, 225, 225, 225, 223, 223, 223, 221, 221, 221, 219, 219, 219, 217, 217, 217, 215, 215, 215, 213, 213, 213, 211, 211, 211, 209, 209, 209, 207, 207, 207, 205, 205, 205, 203, 203, 203, 201, 201, 201, 199, 199, 199, 197, 197, 197, 195, 195, 195, 193, 193, 193, 191, 191, 191, 189, 189, 189, 187, 187, 187, 185, 185, 185, 183, 183, 183, 181, 181, 181, 179, 179, 179, 177, 177, 177, 175, 175, 175, 173, 173, 173, 171, 171, 171, 169, 169, 169, 167, 167, 167, 165, 165, 165, 163, 163, 163, 161, 161, 161, 159, 159, 159, 157, 157, 157, 155, 155, 155, 153, 153, 153, 151, 151, 151, 149, 149, 149, 147, 147, 147, 145, 145, 145, 143, 143, 143, 141, 141, 141, 139, 139, 139, 137, 137, 137, 135, 135, 135, 133, 133, 133, 131, 131, 131, 129, 129, 129, 126, 126, 126, 124, 124, 124, 122, 122, 122, 120, 120, 120, 118, 118, 118, 116, 116, 116, 114, 114, 114, 112, 112, 112, 110, 110, 110, 108, 108, 108, 106, 106, 106, 104, 104, 104, 102, 102, 102, 100, 100, 100, 98, 98, 98, 96, 96, 96, 94, 94, 94, 92, 92, 92, 90, 90, 90, 88, 88, 88, 86, 86, 86, 84, 84, 84, 82, 82, 82, 80, 80, 80, 78, 78, 78, 76, 76, 76, 74, 74, 74, 72, 72, 72, 70, 70, 70, 68, 68, 68, 66, 66, 66, 64, 64, 64, 62, 62, 62, 60, 60, 60, 58, 58, 58, 56, 56, 56, 54, 54, 54, 52, 52, 52, 50, 50, 50, 48, 48, 48, 46, 46, 46, 44, 44, 44, 42, 42, 42, 40, 40, 40, 38, 38, 38, 36, 36, 36, 34, 34, 34, 32, 32, 32, 30, 30, 30, 28, 28, 28, 26, 26, 26, 24, 24, 24, 22, 22, 22, 20, 20, 20, 18, 18, 18, 16, 16, 16, 14, 14, 14, 12, 12, 12, 10, 10, 10, 8, 8, 8, 6, 6, 6, 4, 4, 4, 2, 2, 2, 0, 0, 0, 0, 0, 9, 2, 0, 16, 4, 0, 24, 6, 0, 31, 8, 0, 38, 10, 0, 45, 12, 0, 53, 14, 0, 60, 17, 0, 67, 19, 0, 74, 21, 0, 82, 23, 0, 89, 25, 0, 96, 27, 0, 103, 29, 0, 111, 31, 0, 118, 36, 0, 120, 41, 0, 121, 46, 0, 122, 51, 0, 123, 56, 0, 124, 61, 0, 125, 66, 0, 126, 71, 0, 127, 76, 1, 128, 81, 1, 129, 86, 1, 130, 91, 1, 131, 96, 1, 132, 101, 1, 133, 106, 1, 134, 111, 1, 135, 116, 1, 136, 121, 1, 136, 125, 2, 137, 130, 2, 137, 135, 3, 137, 139, 3, 138, 144, 3, 138, 149, 4, 138, 153, 4, 139, 158, 5, 139, 163, 5, 139, 167, 5, 140, 172, 6, 140, 177, 6, 140, 181, 7, 141, 186, 7, 141, 189, 10, 137, 191, 13, 132, 194, 16, 127, 196, 19, 121, 198, 22, 116, 200, 25, 111, 203, 28, 106, 205, 31, 101, 207, 34, 95, 209, 37, 90, 212, 40, 85, 214, 43, 80, 216, 46, 75, 218, 49, 69, 221, 52, 64, 223, 55, 59, 224, 57, 49, 225, 60, 47, 226, 64, 44, 227, 67, 42, 228, 71, 39, 229, 74, 37, 230, 78, 34, 231, 81, 32, 231, 85, 29, 232, 88, 27, 233, 92, 24, 234, 95, 22, 235, 99, 19, 236, 102, 17, 237, 106, 14, 238, 109, 12, 239, 112, 12, 240, 116, 12, 240, 119, 12, 241, 123, 12, 241, 127, 12, 242, 130, 12, 242, 134, 12, 243, 138, 12, 243, 141, 13, 244, 145, 13, 244, 149, 13, 245, 152, 13, 245, 156, 13, 246, 160, 13, 246, 163, 13, 247, 167, 13, 247, 171, 13, 248, 175, 14, 248, 178, 15, 249, 182, 16, 249, 185, 18, 250, 189, 19, 250, 192, 20, 251, 196, 21, 251, 199, 22, 252, 203, 23, 252, 206, 24, 253, 210, 25, 253, 213, 27, 254, 217, 28, 254, 220, 29, 255, 224, 30, 255, 227, 39, 255, 229, 53, 255, 231, 67, 255, 233, 81, 255, 234, 95, 255, 236, 109, 255, 238, 123, 255, 240, 137, 255, 242, 151, 255, 244, 165, 255, 246, 179, 255, 248, 193, 255, 249, 207, 255, 251, 221, 255, 253, 235, 255, 255, 24} };


 LEP_VID_LUT_BUFFER_T userLUTBuffer;

 static void create_user_LUT()
 {

   for (int i = 0; i < 256; i++)
   {
     LEP_VID_LUT_PIXEL_T pixel;
     pixel.blue = colormap_ironblack.colormap[i*3+2];
     pixel.red = colormap_ironblack.colormap[i*3];
     pixel.green = colormap_ironblack.colormap[i*3+1];
     userLUTBuffer.bin[i] = pixel;
   }
 }

static void set_startup_defaults()
{
  create_user_LUT();
  LEP_RESULT resultUser;
  LEP_RESULT result;

  resultUser = LEP_SetVidUserLut(&hport_desc, &userLUTBuffer);
  if (resultUser != LEP_OK)
  {
    DEBUG_PRINTF("Could not set user color lut: %d\r\n", resultUser);
  }
  /* set a default color lut so radiometric parts produce reasonable pseudocolor images */
  result = LEP_SetVidPcolorLut(&hport_desc, PSUEDOCOLOR_LUT);
  if (result != LEP_OK)
  {
    DEBUG_PRINTF("Could not set default color lut: %d\r\n", result);
  }
}

static HAL_StatusTypeDef print_cust_serial_number()
{
  LEP_RESULT result;
  LEP_SYS_CUST_SERIAL_NUMBER_T cust_serial_number;
  int i;

  result = LEP_GetSysCustSerialNumber(&hport_desc, &cust_serial_number);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not query camera customer serial number! %d\r\n", result);
    return HAL_ERROR;
  }

  DEBUG_PRINTF("SYS Customer Serial Number:\r\n");
  for (i = 0; i < LEP_SYS_MAX_SERIAL_NUMBER_CHAR_SIZE; i++)
    DEBUG_PRINTF("%x ", cust_serial_number.value[i]);
  DEBUG_PRINTF("\r\n");

  return HAL_OK;
}

static HAL_StatusTypeDef print_flir_serial_number()
{
  LEP_RESULT result;
  LEP_SYS_FLIR_SERIAL_NUMBER_T flir_serial_number;

  result = LEP_GetSysFlirSerialNumber(&hport_desc, &flir_serial_number);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not query flir serial number! %d\r\n", result);
    return HAL_ERROR;
  }

  DEBUG_PRINTF("SYS FLIR Serial Number:\r\n");
  DEBUG_PRINTF("%08llX\r\n", flir_serial_number);

  return HAL_OK;
}

static HAL_StatusTypeDef print_camera_uptime()
{
  LEP_RESULT result;
  LEP_SYS_UPTIME_NUMBER_T uptime;

  result = LEP_GetSysCameraUpTime(&hport_desc, &uptime);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not query camera uptime! %d\r\n", result);
    return HAL_ERROR;
  }

  DEBUG_PRINTF("SYS camera uptime:\r\n");
  DEBUG_PRINTF("%lu\r\n", uptime);

  return HAL_OK;
}

static HAL_StatusTypeDef print_fpa_temp_celcius()
{
  LEP_RESULT result;
  LEP_SYS_FPA_TEMPERATURE_CELCIUS_T temp;

  result = LEP_GetSysFpaTemperatureCelcius(&hport_desc, &temp);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not query fpa temp! %d\r\n", result);
    return HAL_ERROR;
  }

  DEBUG_PRINTF("SYS fpa temp celcius:\r\n");
  DEBUG_PRINTF("%d.%d\r\n", (int)temp, (int)((temp-(int)temp)*100.0f));

  return HAL_OK;
}

static HAL_StatusTypeDef print_aux_temp_celcius()
{
  LEP_RESULT result;
  LEP_SYS_AUX_TEMPERATURE_CELCIUS_T temp;

  result = LEP_GetSysAuxTemperatureCelcius(&hport_desc, &temp);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not query fpa temp! %d\r\n", result);
    return HAL_ERROR;
  }

  DEBUG_PRINTF("SYS aux temp celcius:\r\n");
  DEBUG_PRINTF("%d.%d\r\n", (int)temp, (int)((temp-(int)temp)*100.0f));

  return HAL_OK;
}

static HAL_StatusTypeDef print_sdk_version()
{
  LEP_RESULT result;
  LEP_SDK_VERSION_T version;

  result = LEP_GetSDKVersion(&hport_desc, &version);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not query fpa temp! %d\r\n", result);
    return HAL_ERROR;
  }

  DEBUG_PRINTF("Lepton SDK version:\r\n");
  DEBUG_PRINTF("%u.%u.%u\r\n", version.major, version.minor, version.build);

  return HAL_OK;
}

HAL_StatusTypeDef get_scene_stats(uint16_t *min, uint16_t *max, uint16_t *avg)
{
  LEP_RESULT result;
  LEP_SYS_SCENE_STATISTICS_T stats;

  result = LEP_GetSysSceneStatistics(&hport_desc, &stats);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not get scene statistics! %d\r\n", result);
    return HAL_ERROR;
  }

  *min = stats.minIntensity;
  *max = stats.maxIntensity;
  *avg = stats.meanIntensity;

  return HAL_OK;
}

HAL_StatusTypeDef enable_lepton_agc()
{
  LEP_RESULT result;
  LEP_AGC_ENABLE_E enabled;

  result = LEP_GetAgcEnableState(&hport_desc, &enabled);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not query AGC value %d\r\n", result);
    return HAL_ERROR;
  }
  DEBUG_PRINTF("Initial AGC value: %d\r\n", enabled);

  result = LEP_SetAgcCalcEnableState(&hport_desc, LEP_AGC_ENABLE);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not enable AGC calc\r\n");
    return HAL_ERROR;
  }

  LEP_SetAgcPolicy(&hport_desc, LEP_AGC_HEQ);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not set AGC policy\r\n");
    return HAL_ERROR;
  }

  result = LEP_SetAgcEnableState(&hport_desc, LEP_AGC_ENABLE);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not enable AGC\r\n");
    return HAL_ERROR;
  }

  result = LEP_GetAgcEnableState(&hport_desc, &enabled);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not query AGC value %d\r\n", result);
    return HAL_ERROR;
  }
  DEBUG_PRINTF("Current AGC value: %d\r\n", enabled);

  // result = LEP_SetAgcHeqScaleFactor(&hport_desc, LEP_AGC_SCALE_TO_14_BITS);
  // if (result != LEP_OK) {
  //   DEBUG_PRINTF("Could not set AGC scale factor\r\n");
  //   return HAL_ERROR;
  // }

  return HAL_OK;
}

HAL_StatusTypeDef disable_lepton_agc()
{
  LEP_RESULT result;

  result = LEP_SetAgcEnableState(&hport_desc, LEP_AGC_DISABLE);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not enable AGC\r\n");
    return HAL_ERROR;
  }

  return HAL_OK;
}

HAL_StatusTypeDef disable_telemetry(void)
{
  LEP_RESULT result;

  result = LEP_SetSysTelemetryEnableState(&hport_desc, LEP_TELEMETRY_DISABLED);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not disable telemetry %d\r\n", result);
    return HAL_ERROR;
  }

  g_telemetry_num_lines = 0;

  return HAL_OK;
}

HAL_StatusTypeDef enable_telemetry(void)
{
  LEP_RESULT result;

  result = LEP_SetSysTelemetryLocation(&hport_desc, LEP_TELEMETRY_LOCATION_FOOTER);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not set telemetry location %d\r\n", result);
    return HAL_ERROR;
  }

  result = LEP_SetSysTelemetryEnableState(&hport_desc, LEP_TELEMETRY_ENABLED);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not enable telemetry %d\r\n", result);
    return HAL_ERROR;
  }

  g_telemetry_num_lines = g_lepton_type_3 ? 1 : 3;

  return HAL_OK;
}

static LEP_RAD_ENABLE_E rgb888_cached_tlinear_state;

HAL_StatusTypeDef disable_rgb888()
{
  LEP_RESULT result;

  result = LEP_SetRadTLinearEnableState(&hport_desc, rgb888_cached_tlinear_state);
  if (result == LEP_UNDEFINED_FUNCTION_ERROR) {
    DEBUG_PRINTF("LEP_SetRadTLinearEnableState() not available on this lepton\r\n");
  } else if (result != LEP_OK) {
    DEBUG_PRINTF("Could not restore tlinear setting %d\r\n", result);
    return HAL_ERROR;
  }

  return HAL_OK;
}


HAL_StatusTypeDef enable_rgb888(LEP_PCOLOR_LUT_E pcolor_lut)
{
  LEP_RESULT result;
  LEP_OEM_VIDEO_OUTPUT_FORMAT_E fmt;

  LEP_GetOemVideoOutputFormat(&hport_desc, &fmt);
  DEBUG_PRINTF("Current format: %d\r\n", fmt);

  // save the tlinear state to restore when we end rgb888
  result = LEP_GetRadTLinearEnableState(&hport_desc, &rgb888_cached_tlinear_state);
  if (result == LEP_UNDEFINED_FUNCTION_ERROR) {
    DEBUG_PRINTF("LEP_GetRadTLinearEnableState() not available on this lepton\r\n");
  } else if (result != LEP_OK) {
    DEBUG_PRINTF("Could not get tlinear state %d\r\n", result);
    return HAL_ERROR;
  }

  // disable tlinear because it messes with the AGC
  result = LEP_SetRadTLinearEnableState(&hport_desc, LEP_RAD_DISABLE);
  if (result == LEP_UNDEFINED_FUNCTION_ERROR) {
    DEBUG_PRINTF("LEP_SetRadTLinearEnableState() not available on this lepton\r\n");
  } else if (result != LEP_OK) {
    DEBUG_PRINTF("Could not set tlinear state %d\r\n", result);
    return HAL_ERROR;
  }

  result = LEP_SetOemVideoOutputFormat(&hport_desc, LEP_VIDEO_OUTPUT_FORMAT_RGB888);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not set output format %d\r\n", result);
    return HAL_ERROR;
  }

  LEP_GetOemVideoOutputFormat(&hport_desc, &fmt);
  DEBUG_PRINTF("New format: %d\r\n", fmt);

  if (pcolor_lut == -1) {
    // due to what I believe is a lepton bug,
    // even if we don't want to change the palette
    // we need to set it or we'll get noise on the
    // video stream
    result = LEP_GetVidPcolorLut(&hport_desc, &pcolor_lut);
    if (result != LEP_OK) {
      DEBUG_PRINTF("Could not get color lut: %d\r\n", result);
      pcolor_lut = PSUEDOCOLOR_LUT;
    }
  }

  result = LEP_SetVidPcolorLut(&hport_desc, pcolor_lut);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not set color lut: %d\r\n", result);
    return HAL_ERROR;
  }

  return HAL_OK;
}

HAL_StatusTypeDef enable_raw14()
{
  LEP_RESULT result;
  LEP_OEM_VIDEO_OUTPUT_FORMAT_E fmt;

  LEP_GetOemVideoOutputFormat(&hport_desc, &fmt);
  DEBUG_PRINTF("Current format: %d\r\n", fmt);

  result = LEP_SetOemVideoOutputFormat(&hport_desc, LEP_VIDEO_OUTPUT_FORMAT_RAW14);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not set output format %d\r\n", result);
    return HAL_ERROR;
  }

  LEP_GetOemVideoOutputFormat(&hport_desc, &fmt);
  DEBUG_PRINTF("New format: %d\r\n", fmt);

  return HAL_OK;
}

// HAL_OK       = 0x00,
//  HAL_ERROR    = 0x01,
//  HAL_BUSY     = 0x02,
//  HAL_TIMEOUT  = 0x03
HAL_StatusTypeDef init_lepton_command_interface(void)
{
  LEP_RESULT result;

  result = LEP_OpenPort(0, LEP_CCI_TWI, 400, &hport_desc);
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not open Lepton I2C port! %d\r\n", result);
    return HAL_ERROR;
  }

  DEBUG_PRINTF("Lepton I2C command interface opened, device %02x\r\n", hport_desc.deviceAddress);

  if (print_sdk_version() != HAL_OK)
    return HAL_ERROR;

  if (print_cust_serial_number() != HAL_OK)
    return HAL_ERROR;

  if (print_flir_serial_number() != HAL_OK)
    return HAL_ERROR;

  if (print_camera_uptime() != HAL_OK)
    return HAL_ERROR;

  if (print_fpa_temp_celcius() != HAL_OK)
    return HAL_ERROR;

  if (print_aux_temp_celcius() != HAL_OK)
    return HAL_ERROR;

  set_lepton_type();

  set_startup_defaults();

  return HAL_OK;
}

HAL_StatusTypeDef lepton_low_power()
{
  LEP_RESULT result;

  result = LEP_RunOemLowPowerMode2( &hport_desc );
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not set low power mode 2: %d\r\n", result);
    return result;
  }

  return HAL_OK;
}

HAL_StatusTypeDef lepton_power_on()
{
  LEP_RESULT result;

  result = LEP_RunOemPowerOn( &hport_desc );
  if (result != LEP_OK) {
    DEBUG_PRINTF("Could not set power on: %d\r\n", result);
    return result;
  }

  return HAL_OK;
}
