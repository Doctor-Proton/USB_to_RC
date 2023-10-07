  /*
    USB to RC - Convert USB gamepads, joysticks, etc to RC (ppm, sbus, mavlink)
    Copyright (C) 2023  Greg Wood

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
  */
#include "parameters.h"
#include <string.h>

parameter_t parameters[]={
    {"OUTPUT_TYPE",0},
    {"MAVLINK_BAUD",57600},
    {"MAVLINK_SYS_ID",1},
    {"MAVLINK_COMP_ID",2},
    {"MAVLINK_TARGET_SYS_ID",1},
    {"MAVLINK_TARGET_COMP_ID",1},
    {"MAVLINK_RC_OVERRIDE_INTERVAL",20},
    {"MAVLINK_HEARTBEAT_INTERVAL",1000},
    {"PPM_INVERT",0},
    {"PPM_CHANNEL_COUNT",9},
    {"SBUS_INVERT",0},
    {"LOWER_US", 1000},
    {"UPPER_US",2000}
    };

int set_param(const char name[], float value)
{
    int i=0;
    while(strcmp(name,parameters[i].name)!=0 && i<(sizeof(parameters)/sizeof(parameter_t)))
        i++;
    if(i>=(sizeof(parameters)/sizeof(parameter_t)))
        return 0;
    parameters[i].value=value;
    return 1;
}

float get_param(const char name[])
{
    int i=0;
    while(strcmp(name,parameters[i].name)!=0 && i<(sizeof(parameters)/sizeof(parameter_t)))
        i++;
    if(i>=(sizeof(parameters)/sizeof(parameter_t)))
        return 0.0f;
    return parameters[i].value;  
}