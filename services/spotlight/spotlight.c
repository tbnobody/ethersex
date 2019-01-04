/*
 *
 * Copyright (c) 2011-2012 by Maximilian Güntner <maximilian.guentner@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 3
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * For more information on the GPL, please go to:
 * http://www.gnu.org/copyleft/gpl.html
 */

/* Module description: spotlight is a forwarder of mqtt to various i2c PWM chips*/
#include <avr/pgmspace.h>
#include "config.h"
#include "spotlight.h"
#include "core/eeprom.h"
#include "hardware/i2c/master/i2c_pca9685.h"
#include "protocols/mqtt/mqtt.h"
#include "protocols/uip/uip.h"
#include "services/dmx-storage/dmx_storage.h"

#ifdef SPOTLIGHT_DEBUG
#include "core/debug.h"
#define SPOTDEBUG(s, ...) debug_printf("Spotlight " s "\n", ## __VA_ARGS__)
#else
#define SPOTDEBUG(...) do { } while(0)
#endif

#define SPOTLIGHT_PCA9685_ADDRESS_1 0x40
#define SPOTLIGHT_PCA9685_ADDRESS_2 0x41

spotlight_params_t spotlight_params_ram;

#define MAX_TOPIC_LENGTH 255

#define MQTT_RETAIN true
#define MQTT_SUBSCRIBE_SET_SUFFIX "/set/#"
#define MQTT_SUBSCRIBE_SET_FORMAT "/set/%2hhi/%5s"
#define MQTT_STROBO_SUFFIX "/set/strobo"
#define MQTT_WILL_TOPIC_SUFFIX "/status/lwt"
#define MQTT_WILL_MESSAGE_OFFLINE "offline"
#define MQTT_WILL_MESSAGE_ONLINE "online"

char *mqtt_subscribe_set_topic;
char *mqtt_subscribe_set_format;
char *mqtt_will_topic;
char *mqtt_strobo_topic;

bool send_online_lwt = false;
uint8_t strobo = 0;

const uint16_t cie_luminance_12bit[256] PROGMEM =
  { 0, 1, 3, 13, 14, 15, 16, 17, 18, 19, 20,
  21, 23, 24, 26, 27, 29, 30, 32, 33, 35,
  37, 39, 41, 43, 45, 47, 49, 51, 53, 56,
  58, 61, 63, 66, 68, 71, 74, 77, 80, 83,
  86, 89, 93, 96, 99, 103, 107, 110, 114, 118,
  122, 126, 130, 134, 139, 143, 148, 152, 157, 162,
  166, 171, 176, 182, 187, 192, 198, 203, 209, 215,
  221, 227, 233, 239, 245, 252, 258, 265, 272, 278,
  285, 293, 300, 307, 315, 322, 330, 338, 346, 354,
  362, 370, 379, 387, 396, 405, 414, 423, 432, 441,
  451, 460, 470, 480, 490, 500, 510, 521, 531, 542,
  553, 564, 575, 587, 598, 610, 621, 633, 645, 657,
  670, 682, 695, 708, 721, 734, 747, 761, 774, 788,
  802, 816, 830, 845, 859, 874, 889, 904, 919, 935,
  950, 966, 982, 998, 1015, 1031, 1048, 1065, 1082, 1099,
  1116, 1134, 1151, 1169, 1187, 1206, 1224, 1243, 1262, 1281,
  1300, 1319, 1339, 1359, 1379, 1399, 1420, 1440, 1461, 1482,
  1503, 1525, 1546, 1568, 1590, 1612, 1635, 1657, 1680, 1703,
  1726, 1750, 1773, 1797, 1821, 1846, 1870, 1895, 1920, 1945,
  1971, 1996, 2022, 2048, 2074, 2101, 2128, 2155, 2182, 2209,
  2237, 2265, 2293, 2321, 2350, 2379, 2408, 2437, 2466, 2496,
  2526, 2556, 2587, 2617, 2648, 2680, 2711, 2743, 2775, 2807,
  2839, 2872, 2905, 2938, 2971, 3005, 3039, 3073, 3108, 3142,
  3177, 3212, 3248, 3284, 3320, 3356, 3392, 3429, 3466, 3503,
  3541, 3579, 3617, 3655, 3694, 3733, 3772, 3812, 3851, 3891,
  3932, 3972, 4013, 4054, 4096
};

int8_t dmx_conn_id = -1;
bool dmx_connected = false;

enum spotlight_update update;

#define SPOTLIGHT_CHANNELS 10
static spotlight_channel_t channels[SPOTLIGHT_CHANNELS] =
  { {{0, 0, 0}, {0, 0, 0}, SPOTLIGHT_MODE_NORMAL, SPOTLIGHT_NOUPDATE,
     SPOTLIGHT_NOUPDATE} };

static mqtt_connection_config_t mqtt_connection_config = {
  .client_id = CONF_HOSTNAME,   // @todo: add mac address
  .user = "",
  .pass = "",
  .will_topic = NULL,
  .will_qos = 0,
  .will_retain = MQTT_RETAIN,
  .will_message = "",
  .will_message_length = 0
};

double
twoway_max(double a, double b)
{
  return a > b ? a : b;
}

double
twoway_min(double a, double b)
{
  return a < b ? a : b;
}

double
threeway_max(double a, double b, double c)
{
  return twoway_max(a, twoway_max(b, c));
}

double
threeway_min(double a, double b, double c)
{
  return twoway_min(a, twoway_min(b, c));
}

/**
 * Converts an HSV color value to RGB. Conversion formula
 * adapted from http://en.wikipedia.org/wiki/HSV_color_space.
 * Assumes h, s, and v are contained in the set [0, 1] and
 * returns r, g, and b in the set [0, 255].
 *
 * @param   Number  h       The hue
 * @param   Number  s       The saturation
 * @param   Number  v       The value
 * @return  Array           The RGB representation
 */
static void
hsvToRgb(spotlight_hsv_color_t hsv, spotlight_rgb_color_t * rgb)
{
  double r, g, b;

  int i = hsv.h * 6;
  double f = hsv.h * 6 - i;
  double p = hsv.v * (1 - hsv.s);
  double q = hsv.v * (1 - f * hsv.s);
  double t = hsv.v * (1 - (1 - f) * hsv.s);

  switch (i % 6)
  {
    case 0:
      r = hsv.v, g = t, b = p;
      break;
    case 1:
      r = q, g = hsv.v, b = p;
      break;
    case 2:
      r = p, g = hsv.v, b = t;
      break;
    case 3:
      r = p, g = q, b = hsv.v;
      break;
    case 4:
      r = t, g = p, b = hsv.v;
      break;
    case 5:
      r = hsv.v, g = p, b = q;
      break;
  }

  rgb->r = r * 255;
  rgb->g = g * 255;
  rgb->b = b * 255;
}

/**
 * Converts an RGB color value to HSV. Conversion formula
 * adapted from http://en.wikipedia.org/wiki/HSV_color_space.
 * Assumes r, g, and b are contained in the set [0, 255] and
 * returns h, s, and v in the set [0, 1].
 *
 * @param   Number  r       The red color value
 * @param   Number  g       The green color value
 * @param   Number  b       The blue color value
 * @return  Array           The HSV representation
 */
static void
rgbToHsv(spotlight_rgb_color_t rgb, spotlight_hsv_color_t * hsv)
{
  double rd = (double) rgb.r / 255;
  double gd = (double) rgb.g / 255;
  double bd = (double) rgb.b / 255;
  double max = threeway_max(rd, gd, bd), min = threeway_min(rd, gd, bd);
  double h, s, v = max;

  double d = max - min;
  s = max == 0 ? 0 : d / max;

  if (max == min)
  {
    h = 0;                      // achromatic
  }
  else
  {
    if (max == rd)
    {
      h = (gd - bd) / d + (gd < bd ? 6 : 0);
    }
    else if (max == gd)
    {
      h = (bd - rd) / d + 2;
    }
    else if (max == bd)
    {
      h = (rd - gd) / d + 4;
    }
    h /= 6;
  }

  hsv->h = h;
  hsv->s = s;
  hsv->v = v;
}

static void
spotlight_connack_cb(void)
{
  // This callback will be executed when broker connection is established
  SPOTDEBUG("MqTT Subscribe: %s", mqtt_subscribe_set_topic);
  mqtt_construct_subscribe_packet(mqtt_subscribe_set_topic);
  send_online_lwt = true;
}

static void
spotlight_publish_cb(char const *topic, uint16_t topic_length,
                     const void *payload, uint16_t payload_length,
                     uint8_t retained)
{
  // This callback will be executed when topic is received
  char *strvalue = malloc(topic_length + 1);
  if (strvalue == NULL)
    goto out;
  memcpy(strvalue, topic, topic_length);
  strvalue[topic_length] = '\0';

  uint8_t ret;
  uint8_t dest;
  char type[6];

  SPOTDEBUG("MQTT Received (%d):%s", retained, strvalue);

  if (!strcmp(strvalue, mqtt_strobo_topic))
  {
    SPOTDEBUG("MQTT set STROBO");
    if (payload_length > 0)
    {
      free(strvalue);
      strvalue = malloc(payload_length + 1);
      if (strvalue == NULL)
        goto out;
      memcpy(strvalue, payload, payload_length);
      strvalue[payload_length] = '\0';

      strobo = atoi(strvalue);
    }
    else
    {
      SPOTDEBUG("MQTT strobo unkown");
    }
    goto out;
  }

  ret = sscanf(strvalue, mqtt_subscribe_set_format, &dest, type);

  if (ret == 2 && dest <= SPOTLIGHT_CHANNELS)
  {
    SPOTDEBUG("MQTT set CHANNEL:%d,%s", dest, type);
    free(strvalue);
    strvalue = malloc(payload_length + 1);
    if (strvalue == NULL)
      goto out;
    memcpy(strvalue, payload, payload_length);
    strvalue[payload_length] = '\0';

    if (!strcmp_P(type, PSTR("color")))
    {
      uint8_t r, g, b;
      ret = sscanf_P(strvalue, PSTR("%2hhx%2hhx%2hhx"), &r, &g, &b);
      if (ret == 3)
      {
        SPOTDEBUG("MQTT set color:%d,%d,%d", r, g, b);
        if (dest > 0)
        {
          channels[dest - 1].target.r = r;
          channels[dest - 1].target.g = g;
          channels[dest - 1].target.b = b;
        }
        else if (dest == 0 && !retained)
        {
          for (uint8_t i = 0; i < SPOTLIGHT_CHANNELS; i++)
          {
            channels[i].target.r = r;
            channels[i].target.g = g;
            channels[i].target.b = b;
          }
        }
      }
      else
      {
        SPOTDEBUG("MQTT set type unkown");
        goto out;
      }
    }
    else if (!strcmp_P(type, PSTR("mode")))
    {
      uint8_t mode;
      ret = sscanf_P(strvalue, PSTR("%1hhi"), &mode);
      if (ret == 1)
      {
        SPOTDEBUG("MQTT set mode:%d", mode);
        if (dest > 0)
        {
          channels[dest - 1].mode =
            mode == 1 ? SPOTLIGHT_MODE_FADE : SPOTLIGHT_MODE_NORMAL;
        }
        else if (dest == 0 && !retained)
        {
          for (uint8_t i = 0; i < SPOTLIGHT_CHANNELS; i++)
          {
            channels[i].mode =
              mode == 1 ? SPOTLIGHT_MODE_FADE : SPOTLIGHT_MODE_NORMAL;
          }
        }
      }
      else
      {
        SPOTDEBUG("MQTT set type unkown");
        goto out;
      }
    }
    else if (!strcmp_P(type, PSTR("rand")) && !retained)
    {
      SPOTDEBUG("MQTT set random");
      if (dest > 0)
      {
        spotlight_hsv_color_t hsv;
        rgbToHsv(channels[dest - 1].value, &hsv);
        hsv.s = 1;
        if (hsv.v < 0.1 || hsv.v > 0.9)
          hsv.v = 1;
        hsv.h = (double) rand() / (double) RAND_MAX;
        hsvToRgb(hsv, &channels[dest - 1].target);
      }
      else
      {
        spotlight_hsv_color_t hsv;
        for (uint8_t i = 0; i < SPOTLIGHT_CHANNELS; i++)
        {
          rgbToHsv(channels[i].value, &hsv);
          hsv.s = 1;
          if (hsv.v < 0.1 || hsv.v > 0.9)
            hsv.v = 1;
          hsv.h = (double) rand() / (double) RAND_MAX;
          hsvToRgb(hsv, &channels[i].target);
        }
      }
    }
  }
  else
  {
    SPOTDEBUG("MQTT set type unkown");
    goto out;
  }

out:
  if (strvalue != NULL)
    free(strvalue);
}

static void
spotlight_poll_cb(void)
{
  if (send_online_lwt)
  {
    mqtt_construct_publish_packet(mqtt_will_topic, MQTT_WILL_MESSAGE_ONLINE,
                                  sizeof(MQTT_WILL_MESSAGE_ONLINE) - 1,
                                  MQTT_RETAIN);
    SPOTDEBUG("%s=%s", mqtt_will_topic, MQTT_WILL_MESSAGE_ONLINE);
    send_online_lwt = false;
  }

  for (uint8_t i = 0; i < SPOTLIGHT_CHANNELS; i++)
  {
    if (channels[i].sendUpdate == SPOTLIGHT_UPDATE)
    {
      char topic[MAX_TOPIC_LENGTH];
      char payload[7];
      sprintf_P(topic, PSTR("%s/get/%d/color"),
                spotlight_params_ram.mqtt_topic, i + 1);
      sprintf_P(payload, PSTR("%02hhX%02hhX%02hhX"), channels[i].value.r,
                channels[i].value.g, channels[i].value.b);

      mqtt_construct_publish_packet(topic, payload, sizeof(payload) - 1,
                                    MQTT_RETAIN);
      channels[i].sendUpdate = SPOTLIGHT_NOUPDATE;
    }
  }
}

static const mqtt_callback_config_t mqtt_callback_config PROGMEM = {
  .connack_callback = spotlight_connack_cb,
  .poll_callback = spotlight_poll_cb,
  .close_callback = NULL,
  .publish_callback = spotlight_publish_cb,
};

void
spotlight_netinit(void)
{
  i2c_pca9685_set_mode(SPOTLIGHT_PCA9685_ADDRESS_1, SPOTLIGHT_PCA9685_EXTDRV,
                       SPOTLIGHT_PCA9685_IVRT, SPOTLIGHT_PCA9685_PRESCALER);

  i2c_pca9685_set_mode(SPOTLIGHT_PCA9685_ADDRESS_2, SPOTLIGHT_PCA9685_EXTDRV,
                       SPOTLIGHT_PCA9685_IVRT, SPOTLIGHT_PCA9685_PRESCALER);

  SPOTDEBUG("MqTT Init");
  eeprom_restore(spotlight_params, &spotlight_params_ram,
                 sizeof(spotlight_params_t));

  uip_ipaddr(&mqtt_connection_config.target_ip,
             spotlight_params_ram.mqtt_ip[0],
             spotlight_params_ram.mqtt_ip[1],
             spotlight_params_ram.mqtt_ip[2],
             spotlight_params_ram.mqtt_ip[3]);

  mqtt_connection_config.user = spotlight_params_ram.mqtt_user;
  mqtt_connection_config.pass = spotlight_params_ram.mqtt_pass;

  // init base topic
  mqtt_subscribe_set_topic =
    malloc(strlen(spotlight_params_ram.mqtt_topic) +
           strlen_P(PSTR(MQTT_SUBSCRIBE_SET_SUFFIX)) + 1);
  mqtt_subscribe_set_topic[0] = '\0';
  strncpy(mqtt_subscribe_set_topic, spotlight_params_ram.mqtt_topic,
          strlen(spotlight_params_ram.mqtt_topic) + 1);
  strcat_P(mqtt_subscribe_set_topic, PSTR(MQTT_SUBSCRIBE_SET_SUFFIX));

  mqtt_subscribe_set_format =
    malloc(strlen(spotlight_params_ram.mqtt_topic) +
           strlen_P(PSTR(MQTT_SUBSCRIBE_SET_FORMAT)) + 1);
  mqtt_subscribe_set_format[0] = '\0';
  strncpy(mqtt_subscribe_set_format, spotlight_params_ram.mqtt_topic,
          strlen(spotlight_params_ram.mqtt_topic) + 1);
  strcat_P(mqtt_subscribe_set_format, PSTR(MQTT_SUBSCRIBE_SET_FORMAT));

  mqtt_strobo_topic =
    malloc(strlen(spotlight_params_ram.mqtt_topic) +
           strlen_P(PSTR(MQTT_STROBO_SUFFIX)) + 1);
  mqtt_strobo_topic[0] = '\0';
  strncpy(mqtt_strobo_topic, spotlight_params_ram.mqtt_topic,
          strlen(spotlight_params_ram.mqtt_topic) + 1);
  strcat_P(mqtt_strobo_topic, PSTR(MQTT_STROBO_SUFFIX));

  mqtt_will_topic =
    malloc(strlen(spotlight_params_ram.mqtt_topic) +
           strlen_P(PSTR(MQTT_WILL_TOPIC_SUFFIX)) + 1);
  mqtt_will_topic[0] = '\0';
  strncpy(mqtt_will_topic, spotlight_params_ram.mqtt_topic,
          strlen(spotlight_params_ram.mqtt_topic) + 1);
  strcat_P(mqtt_will_topic, PSTR(MQTT_WILL_TOPIC_SUFFIX));

  mqtt_connection_config.will_topic = mqtt_will_topic;
  mqtt_connection_config.will_message = MQTT_WILL_MESSAGE_OFFLINE;
  mqtt_connection_config.will_message_length =
    sizeof(MQTT_WILL_MESSAGE_OFFLINE) - 1;

  mqtt_register_callback(&mqtt_callback_config);
  mqtt_set_connection_config(&mqtt_connection_config);

  //Connect to dmx-storage
  dmx_conn_id = dmx_storage_connect(CONF_ARTNET_OUTUNIVERSE);
  if (dmx_conn_id != -1)
  {
    dmx_connected = true;
    SPOTDEBUG("Connection to dmx-storage established! id:%d", dmx_conn_id);
  }
  else
  {
    dmx_connected = false;
    SPOTDEBUG("Connection to dmx-storage couldn't be established!");
  }
}

void
spotlight_main(void)
{
  if (update == SPOTLIGHT_UPDATE)
  {                             /*Only transmit if at least one channels has been updated */
    update = SPOTLIGHT_NOUPDATE;

    uint16_t pca9685_values[SPOTLIGHT_CHANNELS * 3];    // 3 colors / channel
    for (uint8_t i = 0; i < SPOTLIGHT_CHANNELS; i++)
    {
      channels[i].update = SPOTLIGHT_NOUPDATE;
      pca9685_values[3 * i + 0] =
        pgm_read_word_near(cie_luminance_12bit + channels[i].value.r);
      pca9685_values[3 * i + 1] =
        pgm_read_word_near(cie_luminance_12bit + channels[i].value.g);
      pca9685_values[3 * i + 2] =
        pgm_read_word_near(cie_luminance_12bit + channels[i].value.b);
    }

    i2c_pca9685_set_leds_fast(SPOTLIGHT_PCA9685_ADDRESS_1, 0,
                              15, pca9685_values);

    i2c_pca9685_set_leds_fast(SPOTLIGHT_PCA9685_ADDRESS_2, 0,
                              15, &pca9685_values[15]);
  }
}

void
spotlight_dmx_update(void)
{
  if (get_dmx_slot_state(CONF_ARTNET_OUTUNIVERSE, dmx_conn_id) ==
      DMX_NEWVALUES)
  {
    /*Update values if they are really newer */
    /*Layout for starburst is CCCCMMMMS, where C is Channel, M is Mode and S is Strobe (optional) */
    for (uint8_t i = 0; i < SPOTLIGHT_CHANNELS; i++)
    {
      channels[i].mode =
        get_dmx_channel_slot_raw(CONF_ARTNET_OUTUNIVERSE,
                                 spotlight_params_ram.dmx_offset + i +
                                 SPOTLIGHT_CHANNELS * 3, dmx_conn_id) == 0 ?
        SPOTLIGHT_MODE_NORMAL : SPOTLIGHT_MODE_FADE;

      channels[i].target.r =
        get_dmx_channel_slot(CONF_ARTNET_OUTUNIVERSE,
                             spotlight_params_ram.dmx_offset + i * 3 + 0,
                             dmx_conn_id);

      channels[i].target.g =
        get_dmx_channel_slot(CONF_ARTNET_OUTUNIVERSE,
                             spotlight_params_ram.dmx_offset + i * 3 + 1,
                             dmx_conn_id);

      channels[i].target.b =
        get_dmx_channel_slot(CONF_ARTNET_OUTUNIVERSE,
                             spotlight_params_ram.dmx_offset + i * 3 + 2,
                             dmx_conn_id);
    }

    strobo =
      get_dmx_channel_slot(CONF_ARTNET_OUTUNIVERSE,
                           spotlight_params_ram.dmx_offset +
                           SPOTLIGHT_CHANNELS * 3 + SPOTLIGHT_CHANNELS,
                           dmx_conn_id) / (255 / 25);
  }
}

void
spotlight_process(void)
{
  if (dmx_connected)
  {
    spotlight_dmx_update();
  }

  for (uint8_t i = 0; i < SPOTLIGHT_CHANNELS; i++)
  {
    switch (channels[i].mode)
    {
      case SPOTLIGHT_MODE_NORMAL:
        if (channels[i].value.r != channels[i].target.r ||
            channels[i].value.g != channels[i].target.g ||
            channels[i].value.b != channels[i].target.b)
        {

          channels[i].value.r = channels[i].target.r;
          channels[i].value.g = channels[i].target.g;
          channels[i].value.b = channels[i].target.b;
          channels[i].update = SPOTLIGHT_UPDATE;
          channels[i].sendUpdate = SPOTLIGHT_UPDATE;
          update = SPOTLIGHT_UPDATE;
        }
        else
        {
          channels[i].update = SPOTLIGHT_NOUPDATE;
        }
        break;

      case SPOTLIGHT_MODE_FADE:
        channels[i].update = SPOTLIGHT_NOUPDATE;
        if (channels[i].value.r > channels[i].target.r)
        {
          channels[i].value.r--;
          channels[i].update = SPOTLIGHT_UPDATE;
          channels[i].sendUpdate = SPOTLIGHT_UPDATE;
          update = SPOTLIGHT_UPDATE;
        }
        else if (channels[i].value.r < channels[i].target.r)
        {
          channels[i].value.r++;
          channels[i].update = SPOTLIGHT_UPDATE;
          channels[i].sendUpdate = SPOTLIGHT_UPDATE;
          update = SPOTLIGHT_UPDATE;
        }

        if (channels[i].value.g > channels[i].target.g)
        {
          channels[i].value.g--;
          channels[i].update = SPOTLIGHT_UPDATE;
          channels[i].sendUpdate = SPOTLIGHT_UPDATE;
          update = SPOTLIGHT_UPDATE;
        }
        else if (channels[i].value.g < channels[i].target.g)
        {
          channels[i].value.g++;
          channels[i].update = SPOTLIGHT_UPDATE;
          channels[i].sendUpdate = SPOTLIGHT_UPDATE;
          update = SPOTLIGHT_UPDATE;
        }

        if (channels[i].value.b > channels[i].target.b)
        {
          channels[i].value.b--;
          channels[i].update = SPOTLIGHT_UPDATE;
          channels[i].sendUpdate = SPOTLIGHT_UPDATE;
          update = SPOTLIGHT_UPDATE;
        }
        else if (channels[i].value.b < channels[i].target.b)
        {
          channels[i].value.b++;
          channels[i].update = SPOTLIGHT_UPDATE;
          channels[i].sendUpdate = SPOTLIGHT_UPDATE;
          update = SPOTLIGHT_UPDATE;
        }
        break;

    }
  }

#ifdef SPOTLIGHT_PCA9685_STROBO
  /* Hardware stroboscope support:
   * Control all channels of a PCA9685 using the output enable.
   * Value range: 1-25 (results in 1hz - 25hz)
   */
  static uint8_t pca9685_strobo_counter = 0;
  if (strobo * 2 > 0 && strobo * 2 <= 50)
  {
    if (pca9685_strobo_counter >= 50 / (strobo * 2))
    {
      i2c_pca9685_output_enable(TOGGLE);
      pca9685_strobo_counter = 0;
    }
    pca9685_strobo_counter++;
    pca9685_strobo_counter %= 50;
  }
  else
  {
    i2c_pca9685_output_enable(ON);
  }
#endif
}

/*
	-- Ethersex META --
	header(services/spotlight/spotlight.h)
	net_init(spotlight_netinit)
	mainloop(spotlight_main)
	timer(1,spotlight_process())
*/
