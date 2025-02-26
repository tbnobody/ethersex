/*
 *
 * Copyright (c) 2011 by Maximilian Güntner <maximilian.guentner@gmail.com>
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

#ifndef SPOTLIGHT_H
#define SPOTLIGHT_H

#include <stdint.h>
#include "config.h"
#include "protocols/uip/uip.h"

#ifdef SPOTLIGHT_SUPPORT

#ifdef SPOTLIGHT_PCA9685_EXTDRV
#define SPOTLIGHT_PCA9685_EXTDRV 1
#else
#define SPOTLIGHT_PCA9685_EXTDRV 0
#endif
#ifdef SPOTLIGHT_PCA9685_IVRT
#define SPOTLIGHT_PCA9685_IVRT 1
#else
#define SPOTLIGHT_PCA9685_IVRT 0
#endif

#define IPADDR_LEN sizeof(uip_ipaddr_t)

#define SPOTLIGHT_VALUESIZE 16
#define SPOTLIGHT_TOPICSIZE 32

enum spotlight_update
{
  SPOTLIGHT_UPDATE,
  SPOTLIGHT_NOUPDATE
};

enum spotlight_mode
{
  SPOTLIGHT_MODE_NORMAL,
  SPOTLIGHT_MODE_FADE
};

typedef struct
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
} spotlight_rgb_color_t;

typedef struct
{
  double h;
  double s;
  double v;
} spotlight_hsv_color_t;

typedef struct
{
  //Current value
  spotlight_rgb_color_t current_color;
  //Target value
  spotlight_rgb_color_t target_color;
  enum spotlight_mode mode;
  enum spotlight_update update;
  enum spotlight_update sendUpdate;
} spotlight_channel_t;

typedef struct
{
  uint8_t mqtt_ip[IPADDR_LEN];
  char mqtt_user[SPOTLIGHT_VALUESIZE];
  char mqtt_pass[SPOTLIGHT_VALUESIZE];
  char mqtt_topic[SPOTLIGHT_TOPICSIZE];
  uint8_t dmx_offset;
} spotlight_params_t;

extern spotlight_params_t spotlight_params_ram;

void spotlight_netinit(void);
void spotlight_main(void);
void spotlight_process(void);
uint8_t spotlight_dmx_offset(void);
#endif

#endif /* SPOTLIGHT_H */
