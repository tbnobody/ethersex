/*
 * Copyright (c) 2018-2025 by Thomas Basler <thomas@familie-basler.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License (either version 2 or
 * version 3) as published by the Free Software Foundation.
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

#include "core/eeprom.h"
#include "protocols/ecmd/ecmd-base.h"
#include "protocols/uip/parse.h"
#include "protocols/uip/uip.h"
#include "services/spotlight/spotlight.h"
#include <stdint.h>

int16_t parse_cmd_mqtt_ip(char* cmd, char* output, uint16_t len)
{
    uip_ipaddr_t hostaddr;

    while (*cmd == ' ') {
        cmd++;
    }

    if (*cmd != '\0') {
        /* try to parse ip */
        if (parse_ip(cmd, &hostaddr)) {
            return ECMD_ERR_PARSE_ERROR;
        }

        eeprom_save(spotlight_params.mqtt_ip, &hostaddr, IPADDR_LEN);
        eeprom_update_chksum();

        return ECMD_FINAL_OK;

    } else {
        eeprom_restore_ip(spotlight_params.mqtt_ip, &hostaddr);
        return ECMD_FINAL(print_ipaddr(&hostaddr, output, len));
    }
}

int16_t parse_cmd_mqtt_user(char* cmd, char* output, uint16_t len)
{
    while (*cmd == ' ') {
        cmd++;
    }

    if (*cmd != '\0') {
        strncpy(spotlight_params_ram.mqtt_user, cmd, sizeof(spotlight_params_ram.mqtt_user));
        spotlight_params_ram.mqtt_user[sizeof(spotlight_params_ram.mqtt_user) - 1] = '\0';
        eeprom_save(spotlight_params.mqtt_user, spotlight_params_ram.mqtt_user, SPOTLIGHT_VALUESIZE);
        eeprom_update_chksum();
        return ECMD_FINAL_OK;
    }

    return ECMD_FINAL(snprintf_P(output, len, PSTR("%s"), spotlight_params_ram.mqtt_user));
}

int16_t parse_cmd_mqtt_pass(char* cmd, char* output, uint16_t len)
{
    while (*cmd == ' ') {
        cmd++;
    }

    if (*cmd != '\0') {
        strncpy(spotlight_params_ram.mqtt_pass, cmd, sizeof(spotlight_params_ram.mqtt_pass));
        spotlight_params_ram.mqtt_pass[sizeof(spotlight_params_ram.mqtt_pass) - 1] = '\0';
        eeprom_save(spotlight_params.mqtt_pass, spotlight_params_ram.mqtt_pass, SPOTLIGHT_VALUESIZE);
        eeprom_update_chksum();
        return ECMD_FINAL_OK;
    }

    return ECMD_FINAL(snprintf_P(output, len, PSTR("%s"), spotlight_params_ram.mqtt_pass));
}

int16_t parse_cmd_mqtt_topic(char* cmd, char* output, uint16_t len)
{
    while (*cmd == ' ') {
        cmd++;
    }

    if (*cmd != '\0') {
        strncpy(spotlight_params_ram.mqtt_topic, cmd, sizeof(spotlight_params_ram.mqtt_topic));
        spotlight_params_ram.mqtt_topic[sizeof(spotlight_params_ram.mqtt_topic) - 1] = '\0';
        eeprom_save(spotlight_params.mqtt_topic, spotlight_params_ram.mqtt_topic, SPOTLIGHT_TOPICSIZE);
        eeprom_update_chksum();
        return ECMD_FINAL_OK;
    }

    return ECMD_FINAL(snprintf_P(output, len, PSTR("%s"), spotlight_params_ram.mqtt_topic));
}

uint16_t parse_cmd_dmx_offset(char* cmd, char* output, uint16_t len)
{
    while (*cmd == ' ') {
        cmd++;
    }

    if (*cmd != '\0') {
        spotlight_params_ram.dmx_offset = atoi(cmd);
        eeprom_save_char(spotlight_params.dmx_offset, spotlight_params_ram.dmx_offset);
        eeprom_update_chksum();
        return ECMD_FINAL_OK;
    }

    return ECMD_FINAL(snprintf_P(output, len, PSTR("%d"), spotlight_params_ram.dmx_offset));
}

/*
    -- Ethersex META --

    block([[Spotlight]])
    ecmd_feature(mqtt_ip, "spotlight mqtt ip",[IP],Display/Set the IP address.)
    ecmd_feature(mqtt_user, "spotlight mqtt user",[USERNAME],MqTT Username)
    ecmd_feature(mqtt_pass, "spotlight mqtt pass",[PASSWORD],MqTT Password)
    ecmd_feature(mqtt_topic, "spotlight mqtt topic",[TOPIC],MqTT Topic Prefix)
    ecmd_feature(dmx_offset, "spotlight dmx offset",[DMX OFFSET],DMX Offset)
*/
