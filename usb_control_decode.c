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
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>
#include "usb_control_decode.h"
#include "HIDParser/HIDParser.h"
#include "output.h"

static bool HIDReportReady=false;
HID_ReportInfo_t HIDReport;
uint32_t last_display_time=0;
bool do_decode_print=false;

SemaphoreHandle_t HID_report_mutex = NULL;
StaticSemaphore_t HID_report_mutex_buffer;

unsigned char input_buffer[256];


void init_usb_control_decode(void)
{
    HID_report_mutex = xSemaphoreCreateMutexStatic( &HID_report_mutex_buffer );
    configASSERT( HID_report_mutex );
}

bool CALLBACK_HIDParser_FilterHIDReportItem(HID_ReportItem_t* const CurrentItem)
{

    //if(CurrentItem->Attributes.Usage.Page==1 && (CurrentItem->Attributes.Usage.Usage==0 || CurrentItem->Attributes.Usage.Usage==1)) //added for DS3 controller
    //    return false;
    if(CurrentItem->ItemType>1 || CurrentItem->Attributes.Usage.Page==65280)
        return false;
        else
        printf("[HID] bit offset = %d, item type = %d, item flags = %d, Report ID = %d, Att.Usage.page = %d, Att.Usage.Usage = %d\r\n",
            CurrentItem->BitOffset,CurrentItem->ItemType,CurrentItem->ItemFlags,CurrentItem->ReportID,CurrentItem->Attributes.Usage.Page,CurrentItem->Attributes.Usage.Usage);
    return true;
}


bool load_HID_report(unsigned char report[],int length)
{
    
    enum HID_Parse_ErrorCodes_t Error;
    Error=(enum HID_Parse_ErrorCodes_t)USB_ProcessHIDReport(report,length,&HIDReport);
    printf("[HID Parse] %d %d",Error,HIDReport.TotalReportItems);
    if(Error==HID_PARSE_Successful)
        HIDReportReady=true;
        else
        HIDReportReady=false;

    new_descriptor_set_event();
    return HIDReportReady;
}

void load_XID_report(void)
{
    printf("[XID] generating fake HID descriptor");
    memset((void*)&HIDReport,0,sizeof(HIDReport));
    HIDReport.TotalReportItems=22;
    HIDReport.TotalDeviceReports=1;
    HIDReport.LargestReportSizeBits=168;
    HIDReport.ReportIDSizes[0].ReportID=0;
    HIDReport.ReportIDSizes[0].ReportSizeBits[0]=168;
    HIDReport.CollectionPaths[0].Type=0;
    HIDReport.CollectionPaths[0].Usage.Page=1;
    HIDReport.CollectionPaths[0].Usage.Usage=5;
    HIDReport.CollectionPaths[1].Type=0;
    HIDReport.CollectionPaths[1].Usage.Page=1;
    HIDReport.CollectionPaths[1].Usage.Usage=0;
    HIDReport.CollectionPaths[1].Parent=&HIDReport.CollectionPaths[0];
    for(int i=0;i<16;i++)
        {
            HIDReport.ReportItems[i].BitOffset=16+i;
            HIDReport.ReportItems[i].ItemFlags=2;
            HIDReport.ReportItems[i].Attributes.BitSize=1;
            HIDReport.ReportItems[i].Attributes.Usage.Page=9;
            HIDReport.ReportItems[i].Attributes.Usage.Usage=1;
            HIDReport.ReportItems[i].Attributes.Logical.Maximum=1;
            HIDReport.ReportItems[i].CollectionPath=&HIDReport.CollectionPaths[1];
        }
    for(int i=16;i<18;i++)
        {
            HIDReport.ReportItems[i].BitOffset=32+(i-16)*8;
            HIDReport.ReportItems[i].ItemFlags=2;
            HIDReport.ReportItems[i].Attributes.BitSize=8;
            HIDReport.ReportItems[i].Attributes.Usage.Page=1;
            HIDReport.ReportItems[i].Attributes.Usage.Usage=54;
            HIDReport.ReportItems[i].Attributes.Logical.Maximum=255;
            HIDReport.ReportItems[i].CollectionPath=&HIDReport.CollectionPaths[1];
        }
    for(int i=18;i<22;i++)
        {
            HIDReport.ReportItems[i].BitOffset=48+(i-18)*16;
            HIDReport.ReportItems[i].ItemFlags=2;
            HIDReport.ReportItems[i].Attributes.BitSize=16;
            HIDReport.ReportItems[i].Attributes.Usage.Page=1;
            HIDReport.ReportItems[i].Attributes.Usage.Usage=48+(i-18);
            HIDReport.ReportItems[i].Attributes.Logical.Minimum=-32768;
            HIDReport.ReportItems[i].Attributes.Logical.Maximum=32767;
            HIDReport.ReportItems[i].CollectionPath=&HIDReport.CollectionPaths[1];
        }

    new_descriptor_set_event();
}

void clear_HID_report(void)
{
    HIDReportReady=false;
}

void process_usb_packet(unsigned char buffer[],int length)
{
    bool IndexResult[HIDReport.TotalReportItems];
if(xSemaphoreTake(HID_report_mutex,2)==pdTRUE)
    {
    //gpio_set_level(TIMING_DEBUG_GPIO, 1);
        memcpy(input_buffer,buffer,length<sizeof(input_buffer)?length:sizeof(input_buffer));
        for(int i=0;i<HIDReport.TotalReportItems;i++)
        {
            IndexResult[i]=USB_GetHIDReportItemInfo(buffer,&HIDReport.ReportItems[i]);
        }
        for(int i=0;i<HIDReport.TotalReportItems;i++)
        {
            if(IndexResult[i]==false)
                printf("[parse] Failed to load value at %d\r\n",i);
        }
    output_set_event();
    //gpio_set_level(TIMING_DEBUG_GPIO, 0);
    xSemaphoreGive(HID_report_mutex);
    }
    else
    {
        return;
    }

    if((xTaskGetTickCount()-last_display_time)>=5000 && do_decode_print==true)
    {
        printf("[display] showing %d channels\r\n", HIDReport.TotalReportItems);
        last_display_time=xTaskGetTickCount();
        for(int i=0;i<HIDReport.TotalReportItems;i++)
        {
            printf("%d",i);
            int Tabs=1;
            if(HIDReport.ReportItems[i].Attributes.BitSize>8)
                Tabs=2; 
            for(int j=0;j<Tabs;j++)
                printf("\t");              
        }
        printf("\r\n");
        for(int i=0;i<HIDReport.TotalReportItems;i++)
        {
            HID_ReportItem_t *Item=&(HIDReport.ReportItems[i]);
            printf("%d",(int8_t)Item->Value);//HID_ALIGN_DATA(Item,int8_t));
            int Tabs=1;
            if(HIDReport.ReportItems[i].Attributes.BitSize>8)
                Tabs=2; 
            for(int j=0;j<Tabs;j++)
                printf("\t");              
        }
        printf("\r\n");        
    }
}

void *get_HID_report_pointer(void)
{
if(xSemaphoreTake(HID_report_mutex,5)==pdTRUE)
    {
    return (void*)&HIDReport;
    }
    else
    {
        return 0;
    }
}

void HID_report_done(void)
{
    xSemaphoreGive(HID_report_mutex);
}

int bit_is_set(int bitindex)
{
if(input_buffer[bitindex>>8]&(1<<(bitindex&0x07)))
    return 1;
    else
    return 0;
}

void copy_input_bits(unsigned char output[],int bitindex, int bitcount)
{
    if(bitindex>8*sizeof(input_buffer))
        return;
    int outputindex=0;
    int bitscopied=0;
    unsigned char outputbitindex=0;
    while(bitscopied<bitcount)
    {
        if(bit_is_set(bitscopied+bitindex))
            output[outputindex]|=(1<<outputbitindex);
            else
            output[outputindex]&=~(1<<outputbitindex);
        bitscopied++;
        outputbitindex++;
        if(outputbitindex>7)
            {
                outputbitindex=0;
                outputindex++;
            }
    }
    
}