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