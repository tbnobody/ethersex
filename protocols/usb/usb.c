/*
 *
 * Copyright (c) 2009 by Stefan Riepenhausen <rhn@gmx.net>
 * Copyright (c) 2008 by Christian Dietrich <stettberger@dokucode.de>
 * Copyright (c) 2008 by Stefan Siegl <stesie@brokenpipe.de>
 * Copyright (c) 2019 by Erik Kunze <ethersex@erik-kunze.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
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

#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>

#include "config.h"
#ifdef ECMD_USB_SUPPORT
#include "ecmd_usb.h"
#endif
#ifdef USB_KEYBOARD_SUPPORT
#include "usb_hid_keyboard.h"
#endif
#ifdef USB_MOUSE_SUPPORT
#include "usb_hid_mouse.h"
#endif
#ifdef USB_NET_SUPPORT
#include "usb_net.h"
#endif
#include "usbdrv/usbdrv.h"
#include "requests.h"
#include "usb.h"

#ifdef USB_CFG_PULLUP_IOPORTNAME
#undef usbDeviceConnect
#define usbDeviceConnect()      do { \
                                    USB_PULLUP_DDR |= (1<<USB_CFG_PULLUP_BIT); \
                                    USB_PULLUP_OUT |= (1<<USB_CFG_PULLUP_BIT); \
                                    USB_INTR_ENABLE |= (1 << USB_INTR_ENABLE_BIT); \
                                   } while(0);
#undef usbDeviceDisconnect
#define usbDeviceDisconnect()   do { \
                                    USB_INTR_ENABLE &= ~(1 << USB_INTR_ENABLE_BIT); \
                                    USB_PULLUP_DDR &= ~(1<<USB_CFG_PULLUP_BIT); \
                                    USB_PULLUP_OUT &= ~(1<<USB_CFG_PULLUP_BIT); \
                                   } while(0);
#endif

#if defined(ECMD_USB_SUPPORT) || defined(USB_NET_SUPPORT)
static uint8_t setup_packet;
#endif

usbMsgLen_t
usbFunctionSetup(uchar data[8])
{
  usbRequest_t *rq = (void *) data;

  if ((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_VENDOR)
  {
#if defined(ECMD_USB_SUPPORT) || defined(USB_NET_SUPPORT)
    setup_packet = rq->bRequest;
#endif

#ifdef ECMD_USB_SUPPORT
    if (rq->bRequest == USB_REQUEST_ECMD)
      return ecmd_usb_setup(rq);
#endif
#ifdef USB_NET_SUPPORT
    if (rq->bRequest == USB_REQUEST_NET_SEND ||
        rq->bRequest == USB_REQUEST_NET_RECV)
      return usb_net_setup(rq);
#endif
#ifdef USB_KEYBOARD_SUPPORT
    return hid_usbFunctionSetup(rq);
#endif
#ifdef USB_MOUSE_SUPPORT
    return hid_usbFunctionSetup(rq);
#endif
  }

  return 0;   /* default for not implemented requests: return no data back to host */
}

/* The host sends data to the device */
#if USB_CFG_IMPLEMENT_FN_WRITE
uchar
usbFunctionWrite(uchar * data, uchar len)
{
#ifdef ECMD_USB_SUPPORT
  if (setup_packet == USB_REQUEST_ECMD)
    return ecmd_usb_write((uint8_t *) data, (uint8_t) len);
#endif

#ifdef USB_NET_SUPPORT
  if (setup_packet == USB_REQUEST_NET_SEND)
    return usb_net_write((uint8_t *) data, (uint8_t) len);
#endif

  return 1; /* This was the last chunk, also default fallback */
}
#endif

/* The host receives data from the device */
#if USB_CFG_IMPLEMENT_FN_READ
uchar
usbFunctionRead(uchar * data, uchar len)
{
#ifdef ECMD_USB_SUPPORT
  if (setup_packet == USB_REQUEST_ECMD)
    return ecmd_usb_read(data, len);
#endif

  return 0; /* 0 bytes are read, this is the default fallback */
}
#endif

#if USB_CFG_IMPLEMENT_FN_READ_FINISHED
void
usbFunctionReadFinished(void)
{
#ifdef USB_NET_SUPPORT
  if (setup_packet == USB_REQUEST_NET_RECV)
    usb_net_read_finished();
#endif
}
#endif

/* Wrapper functions to integrate the usb driver in ethersex */
void
usb_periodic(void)
{
  usbPoll();

#ifdef USB_NET_SUPPORT
  usb_net_periodic();
#endif
#ifdef USB_KEYBOARD_SUPPORT
  usb_keyboard_periodic();
#endif
#ifdef USB_MOUSE_SUPPORT
  usb_mouse_periodic();
#endif
}

void
usb_init(void)
{
#ifdef AUTOSET_USB_ENABLE_PIN_SUPPORT
  PIN_SET(USB_ENABLE);
#endif /* AUTOSET_USB_ENABLE_PIN_SUPPORT */
#ifdef USB_NET_SUPPORT
  usb_net_init();
#endif

#ifdef USB_KEYBOARD_SUPPORT
#endif

#define USB_DDR_CONFIG(pin)  DDR_CHAR( pin ## _PORT) &= ~(_BV((pin ## _PIN)) | _BV(USB_INT_PIN))
#define USB_PORT_CONFIG(pin)  PORT_CHAR( pin ## _PORT) &= ~(_BV((pin ## _PIN)) | _BV(USB_INT_PIN))
  USB_DDR_CONFIG(USB_DMINUS);
  USB_PORT_CONFIG(USB_DMINUS);
#undef USB_DDR_CONFIG
#undef USB_PORT_CONFIG

  uint8_t i;
  /* Reenummerate the device */
  usbDeviceDisconnect();
  for (i = 0; i < 20; i++)
  {                             /* 300 ms disconnect */
    _delay_ms(15);
    wdt_kick();
  }
  usbDeviceConnect();

  /* USB Initialize */
  usbInit();
}

/*
  -- Ethersex META --
  header(protocols/usb/usb.h)
  mainloop(usb_periodic)
  init(usb_init)
*/
