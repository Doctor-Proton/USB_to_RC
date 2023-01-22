#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"
#include "HIDParser/HIDParser.h"
#include "usb_control_decode.h"
#include "expr/expr.h"
#include "sdcard.h"
#include "output.h"

#include "pico/stdlib.h"
//
#include "ff_headers.h"
#include "ff_sddisk.h"
#include "ff_stdio.h"
#include "ff_utils.h"
//
#include "hw_config.h"
#include "ppm.h"


#define OUTPUT_TAG "out"

#define DEFAULT_CONFIG_FILE "/config/default.cfg"
#define LOOKUP_CONFIG_FILE "/config/lookup.cfg"

EventGroupHandle_t output_ready_event_handle;
StaticEventGroup_t output_ready_group;

static bool do_print=true;

#define OUTPUT_READY_BIT (1<<0)
#define DESCRIPTOR_READY_BIT (1<<1)
#define DEVICE_AVAILABLE_BIT (1<<2)
#define MAX_INPUT_CHANNELS 64




float input_values[MAX_INPUT_CHANNELS]={0};
float output_values[MAX_OUTPUT_CHANNELS]={0};

/*unsigned char output_expressions[MAX_OUTPUT_CHANNELS][]={
        {"out_1=in_20*500+1500"},
        {"out_2=in_21*500+1500"},
        {"out_3=in_19*500+1500"},
        {"out_4=in_18*500+1500"},
};*/

SemaphoreHandle_t output_mutex = NULL;
StaticSemaphore_t output_mutex_buffer;

SemaphoreHandle_t vars_mutex = NULL;
StaticSemaphore_t vars_mutex_buffer;



typedef struct
{
    const int value;
    const char name[16];
    int count;
    int print_index;
} axis_lookup_t;

axis_lookup_t axis_lookups[]={
    {0x30,"X",0,0},
    {0x31,"Y",0,0},
    {0x32,"Z",0,0},
    {0x33,"Rx",0,0},
    {0x34,"Ry",0,0},
    {0x35,"Rz",0,0},
    {0x36,"slider",0,0},
    {0x37,"dial",0,0},
    {0x38,"wheel",0,0},
    {0x39,"hat_switch",0,0},
    };

axis_lookup_t* get_axis_name(int usage)
{
    for(int i=0;i<sizeof(axis_lookups)/sizeof(axis_lookup_t);i++)
        {
            if(axis_lookups[i].value==usage)
                return &axis_lookups[i];
        }
return 0;
}

struct expr_var_list vars = {0};
#define MAX_EXPRESSION_COUNT 64
struct expr *expressions[MAX_EXPRESSION_COUNT]={0};

unsigned short _VID=0,_PID=0;

uint16_t min_us=LOWER_US;   //-100%
uint16_t max_us=UPPER_US;   //100%

static float scale_us(struct expr_func *f, vec_expr_t *args, void *c) {
  float input = expr_eval(&vec_nth(args, 0));
  float min = expr_eval(&vec_nth(args, 1));
  float max = expr_eval(&vec_nth(args, 2));
  float Span=max-min;
  return Span*input+min;
}

static float scale_percent(struct expr_func *f, vec_expr_t *args, void *c) {
    float input = (expr_eval(&vec_nth(args, 0))-0.5f)/0.5f;
    float min = expr_eval(&vec_nth(args, 1))/100;
    float max = expr_eval(&vec_nth(args, 2))/100;
    float center_us=(min_us+max_us)/2;
    float Span;
    if(input<0)
        Span=center_us-min_us;
        else
        Span=max_us-center_us;
    return center_us+Span*input;
}

static float sticky_buttons_us(struct expr_func *f, vec_expr_t *args, void *c)
{
    if(args->len<2)
        return 0;
    float old_value=expr_eval(&vec_nth(args, 0));
    float default_value=expr_eval(&vec_nth(args, 1));
    if((args->len%2)==1) //odd number of arguments, return 0
        return old_value;
    
    for(int i=2;i<args->len;i+=2)
        {
            float input=expr_eval(&vec_nth(args, i));
            float output=expr_eval(&vec_nth(args, i+1));
            if(input>0.5f)
                old_value=output;
        }
    if(old_value<100 || old_value > 2900 || !isfinite(old_value))
        return default_value;
    return old_value;
}

static float sticky_buttons_percent(struct expr_func *f, vec_expr_t *args, void *c)
{
    if(args->len<2)
        return 0;
    float old_value=expr_eval(&vec_nth(args, 0));
    float center_us=(min_us+max_us)/2;
    if(old_value<center_us)
        {
            float span=center_us-min_us;
            old_value=(old_value-center_us)/span;
            old_value*=100;
        }
        else
        {
            float span=max_us-center_us;
            old_value=(old_value-center_us)/span;
            old_value*=100;
        }
    float default_value=expr_eval(&vec_nth(args, 1));
    if((args->len%2)==1) //odd number of arguments, return 0
        return old_value;
    
    for(int i=2;i<args->len;i+=2)
        {
            float input=expr_eval(&vec_nth(args, i));
            float output=expr_eval(&vec_nth(args, i+1));
            if(input>0.5f)
                old_value=output;
        }
    if(old_value<-200 || old_value > 200 || !isfinite(old_value))
        old_value= default_value;

    float Span;
    if(old_value<0)
        Span=center_us-min_us;
        else
        Span=max_us-center_us;
    return center_us+Span*(old_value/100);
}

static struct expr_func user_funcs[] = {
    {"scale_us", scale_us, NULL, 0},
    {"scale_percent", scale_percent, NULL, 0},
    {"sticky_buttons_us",sticky_buttons_us,NULL,0},
    {"sticky_buttons_percent",sticky_buttons_percent,NULL,0},
    {NULL, NULL, NULL, 0},
};


void output_task(void *arg)
{

    output_ready_event_handle = xEventGroupCreateStatic( &output_ready_group );
    configASSERT( output_ready_event_handle );

    output_mutex = xSemaphoreCreateMutexStatic( &output_mutex_buffer );
    configASSERT( output_mutex );

    vars_mutex = xSemaphoreCreateMutexStatic (&vars_mutex_buffer);
    configASSERT (vars_mutex);
    output_init();

    HID_ReportInfo_t *HID_report_p;
    printf("[Output] starting\r\n");
    uint32_t last_output_time=0;
    uint32_t print_timer=0;

    vTaskDelay(500);
    //#warning reimplement
    sd_card_init();

/*    for(int i=0;i<MAX_INPUT_CHANNELS;i++)
    {
        char input_name[7];
        int len=snprintf(input_name,sizeof(input_name),"int_%d",i);
        struct expr_var *again = expr_var(&vars, input_name, len);
        printf("var %s = %1.1f\n",input_name,again->value);
    }*/

    while(1)
    {
        uint32_t events=xEventGroupWaitBits(output_ready_event_handle,OUTPUT_READY_BIT|DESCRIPTOR_READY_BIT|DEVICE_AVAILABLE_BIT,pdTRUE,pdFALSE,1);
        if((events&DEVICE_AVAILABLE_BIT)==DEVICE_AVAILABLE_BIT && xSemaphoreTake(vars_mutex,2)==pdTRUE)
            {
            HID_report_p=(HID_ReportInfo_t *)get_HID_report_pointer();
            if(HID_report_p!=0)
                { 
                for(int i=0;i<MAX_INPUT_CHANNELS;i++)
                    {
                        char input_name[10];
                        int len=snprintf(input_name,sizeof(input_name),"in_%02d",i);
                        expr_var(&vars,input_name,len);
                        struct expr_var *again = expr_var(&vars, input_name, len);
                        again->value=0.0f;
                    }
                for(int i=0;i<MAX_OUTPUT_CHANNELS;i++)
                    {
                        char output_name[8];
                        int len=snprintf(output_name,sizeof(output_name),"out_%02d",i);
                        expr_var(&vars,output_name,len);
                        struct expr_var *again = expr_var(&vars, output_name, len);
                        again->value=0.0f;
                    }
                
                    int button_index=0;
                    for(int i=0;i<HID_report_p->TotalReportItems;i++)
                        {
                        HID_ReportItem_t *report_item=(HID_ReportItem_t*)&(HID_report_p->ReportItems[i]);
                        if(report_item->Attributes.Usage.Page==1)   //generic controls
                            {
                            axis_lookup_t *axis=get_axis_name(report_item->Attributes.Usage.Usage);
                            if(axis!=0)
                                axis->count++;
                            }
                        }
                    for(int i=0;i<HID_report_p->TotalReportItems;i++)
                        {
                            HID_ReportItem_t *report_item=(HID_ReportItem_t*)&(HID_report_p->ReportItems[i]);
                            if(report_item->Attributes.Usage.Page==9)   //button page
                                {
                                char input_name[10];
                                int len=snprintf(input_name,sizeof(input_name),"button_%02d",button_index);
                                expr_var(&vars,input_name,len);
                                printf("[OUTPUT] Added %s\r\n",input_name);
                                struct expr_var *again = expr_var(&vars, input_name, len);
                                again->value=0.0f;
                                button_index++;
                                }
                            if(report_item->Attributes.Usage.Page==1)   //generic controls
                                {
                                axis_lookup_t *axis=get_axis_name(report_item->Attributes.Usage.Usage);
                                if(axis!=0)
                                    {
                                    int len=0;
                                    char input_name[14];
                                    if(axis->count>1)
                                        {
                                        len=snprintf(input_name,sizeof(input_name),"%s_%02d",axis->name,axis->print_index);
                                        axis->print_index++;
                                        }
                                        else
                                        len=snprintf(input_name,sizeof(input_name),"%s",axis->name);
                                    expr_var(&vars,input_name,len);
                                    printf("[OUTPUT] Added axis %s\r\n",input_name);
                                    struct expr_var *again = expr_var(&vars, input_name, len);
                                    again->value=0.0f;
                                    }
                                }                            


                        }
                HID_report_done();
                //#warning reimplement 
                ppm_sbus_output_init(MODE_PPM_OUT);
                
                }
            //#warning reimplement              
            init_output_mixer();
            xSemaphoreGive(vars_mutex);
            }
        if((events&OUTPUT_READY_BIT)==OUTPUT_READY_BIT && xSemaphoreTake(vars_mutex,2)==pdTRUE)  //wait for output event to be set
        {
            HID_report_p=(HID_ReportInfo_t *)get_HID_report_pointer();
            int input_count=0;
            if(HID_report_p!=0)
            {
                
                //printf("[OUTPUT] calculating output\r\n");
                input_count=HID_report_p->TotalReportItems<MAX_INPUT_CHANNELS?HID_report_p->TotalReportItems:MAX_INPUT_CHANNELS;
                int button_index=0;
                for(int i=0;i<sizeof(axis_lookups)/sizeof(axis_lookup_t);i++)
                {
                    axis_lookups[i].print_index=0;
                }
                for(int i=0;i<input_count;i++)
                    {
                        unsigned char bit_size=HID_report_p->ReportItems[i].Attributes.BitSize;
                        unsigned char channel_signed=(int32_t)HID_report_p->ReportItems[i].Attributes.Logical.Minimum<0?1:0;
                        HID_ReportItem_t *Item=&(HID_report_p->ReportItems[i]);
                        if(bit_size<=8 && channel_signed==0)
                            input_values[i]=(float)((uint8_t)Item->Value);
                        if(bit_size<=8 && channel_signed==1)
                            input_values[i]=(float)((int8_t)Item->Value);
                        if(bit_size>8 && bit_size<=16 && channel_signed==0)
                            input_values[i]=(float)((uint16_t)Item->Value);
                        if(bit_size>8 && bit_size<=16 && channel_signed==1)
                            input_values[i]=(float)((int16_t)Item->Value);
                        if(bit_size>16 && bit_size<=32 && channel_signed==0)
                            input_values[i]=(float)((uint32_t)Item->Value);
                        if(bit_size>16 && bit_size<=32 && channel_signed==1)
                            input_values[i]=(float)((int32_t)Item->Value);
                        /*if(channel_signed==1)
                            {
                                int centerpoint=((int32_t)HID_report_p->ReportItems[i].Attributes.Logical.Minimum+(int32_t)HID_report_p->ReportItems[i].Attributes.Logical.Maximum)/2;
                                input_values[i]-=(float)centerpoint;
                                if(input_values[i]<0)
                                    input_values[i]/=(float)abs((int32_t)HID_report_p->ReportItems[i].Attributes.Logical.Minimum-centerpoint);
                                    else
                                    input_values[i]/=(float)((int32_t)HID_report_p->ReportItems[i].Attributes.Logical.Maximum-centerpoint);
                            }
                            else*/
                            {
                                int span=(HID_report_p->ReportItems[i].Attributes.Logical.Maximum-(int32_t)HID_report_p->ReportItems[i].Attributes.Logical.Minimum);
                                input_values[i]-=(float)((int32_t)HID_report_p->ReportItems[i].Attributes.Logical.Minimum);
                                    input_values[i]/=(float)span;
                            }
                        //gpio_set_level(TIMING_DEBUG_GPIO_2, 1);
                        if(Item->expr_in_p==0)
                            {
                            char input_name[7];
                            
                            int len=snprintf(input_name,sizeof(input_name),"in_%02d",i);
                            struct expr_var *v = expr_var(&vars, input_name, len);
                            v->value=input_values[i];
                            Item->expr_in_p=(void*)v;
                            if(Item->Attributes.Usage.Page==9)   //button page
                                {
                                char button_name[10];
                                int len=snprintf(button_name,sizeof(button_name),"button_%02d",button_index);
                                v = expr_var(&vars, button_name, len);
                                v->value=input_values[i];
                                Item->expr_named_p=(void*)v;
                                button_index++;
                                }
                            
                            if(Item->Attributes.Usage.Page==1)   //generic controls
                                {
                                axis_lookup_t *axis=get_axis_name(Item->Attributes.Usage.Usage);
                                if(axis!=0)
                                    {
                                    int len=0;
                                    char axis_name[16];
                                    if(axis->count>1)
                                        {
                                        len=snprintf(axis_name,sizeof(axis_name),"%s_%02d",axis->name,axis->print_index);
                                        axis->print_index++;
                                        }
                                        else
                                        {
                                        len=snprintf(axis_name,sizeof(axis_name),"%s",axis->name);
                                        }
                                    
                                    v = expr_var(&vars, axis_name, len);
                                    
                                    v->value=input_values[i];
                                    Item->expr_named_p=(void*)v;
                                    }
                                }
                            
                            }
                            else
                            {
                                struct expr_var *v_in=(struct expr_var *)Item->expr_in_p;
                                v_in->value=input_values[i];
                                if(Item->expr_named_p!=0)
                                    {
                                        v_in=(struct expr_var *)Item->expr_named_p;
                                        v_in->value=input_values[i];
                                    }
                            }
                        //gpio_set_level(TIMING_DEBUG_GPIO_2, 0);
                    }
                HID_report_done();
                

            for(int i=0;i<MAX_EXPRESSION_COUNT;i++)
                {
                    if(expressions[i]!=NULL)
                        expr_eval(expressions[i]);
                }
            }
        xSemaphoreGive(vars_mutex);
        uint16_t Channels[MAX_OUTPUT_CHANNELS]={0};
        for(int i=0;i<MAX_OUTPUT_CHANNELS && vars.head!=NULL;i++)
            {
            char output_name[12];
            int len=snprintf(output_name,sizeof(output_name),"out_%02d",i);
            struct expr_var *v = expr_var(&vars, output_name, len);
            if(v!=NULL)
                Channels[i]=(uint16_t)v->value;
            }
        if((xTaskGetTickCount()-print_timer)>500 && do_print==true)
            {
            print_timer=xTaskGetTickCount();
            printf("[OUTPUT] input values:\r\n");
            for(int i=0;i<input_count;i++)
                {
                printf("%1.2f\t",input_values[i]);
                }
            printf("\n");
            printf("[OUTPUT] output values:\r\n");
            for(int i=0;i<MAX_OUTPUT_CHANNELS;i++)
                {
                printf("%d\t",Channels[i]);
                }
            printf("\n");
            }
        //#warning reimplement
        assign_output_channels(Channels);
        }
        /*if((events&DEVICE_AVAILABLE_BIT)==DEVICE_AVAILABLE_BIT && xSemaphoreTake(vars_mutex,200)==pdTRUE)
        {
            init_output_mixer();
            xSemaphoreGive(vars_mutex);
        }*/
    //#warning reimplement
    output_tick();
    }
    
    //unreachable 
    #warning reimplement
    if(is_sd_mounted()==true)
        sd_unmount();
}

void output_set_event(void)
{
    xEventGroupSetBits(output_ready_event_handle,OUTPUT_READY_BIT);
}

void new_descriptor_set_event(void)
{
    xEventGroupSetBits(output_ready_event_handle,DESCRIPTOR_READY_BIT);
}

int get_input_count(void)
{
int retval=-1;
HID_ReportInfo_t *HID_report_p;
HID_report_p=(HID_ReportInfo_t *)get_HID_report_pointer();
if(HID_report_p!=0)
    {
    retval=HID_report_p->TotalReportItems<MAX_INPUT_CHANNELS?HID_report_p->TotalReportItems:MAX_INPUT_CHANNELS;
    HID_report_done();
    }
return retval;
}

void get_input_debug_line(int channel,char line[],int maxlen)
{
    HID_ReportInfo_t *HID_report_p;
HID_report_p=(HID_ReportInfo_t *)get_HID_report_pointer();
if(HID_report_p!=0)
    {
    if(channel<(HID_report_p->TotalReportItems<MAX_INPUT_CHANNELS?HID_report_p->TotalReportItems:MAX_INPUT_CHANNELS))
        {
        HID_ReportItem_t *Item=&(HID_report_p->ReportItems[channel]);
        snprintf(line,maxlen,"ch=%d\tUsage=%d\tPage=%d\tMin=%ld\tMax=%ld\tsize=%d\tbitindex=%d\tInval=%ld\toutput=%1.2f\trawbits=0x%02lX",
            channel,Item->Attributes.Usage.Usage,Item->Attributes.Usage.Page,Item->Attributes.Logical.Minimum,Item->Attributes.Logical.Maximum,
            Item->Attributes.BitSize,Item->BitOffset,Item->Value,input_values[channel],Item->Value);
        }
    HID_report_done();
    }
}

int print_var_line(char output[],int maxlen, int startindex, int varcount)  //returns 1 if more stuff is left to print, 0 if all vars are printed
{
    int printedchars=0;
    if(xSemaphoreTake(vars_mutex,20)==pdFALSE)
        {
        printf("[HTTP] mutex miss\r\n");
        return 1; //give up if we don't get the vars mutex
        }
    struct expr_var *v=vars.head;
    for(int i=0;i<startindex && v!=NULL;i++)
        v=v->next;
    for(int i=0;i<varcount && v!=NULL;i++)
        {
        printedchars+=snprintf(&output[printedchars],maxlen-printedchars,"%s = %1.2f ", v->name,v->value);
        if(printedchars>=maxlen)
            {
            xSemaphoreGive(vars_mutex);
            return 1;
            }
        v=v->next;
        }
    if(printedchars!=0)
        snprintf(&output[printedchars],maxlen-printedchars,"\n");
    if(v==NULL)
        {
        xSemaphoreGive(vars_mutex);
        return 0;
        }
    xSemaphoreGive(vars_mutex);
    return 1;
}


//#warning reimplement
//#if 0

void get_config_line(char *line, ssize_t max_len,FF_FILE *f)
{
    bool is_comment=false;
    do
    {
        char fchar;
        do
        {

            if(ff_fread(&fchar,1,1,f)!=1)
                {
                line[0]=0;
                return;
                }
        } while (!isprint(fchar));


        ssize_t line_len=0;
        is_comment=false;
        do
        {

            if(!isprint(fchar))
                continue;
            if(fchar == '#')
                is_comment=true;
            if(line_len<(max_len+1))
                {
                if(is_comment==false)
                    {
                    line[line_len]=fchar;
                    line[++line_len]=0;
                    }
                }
                else
                {
                return;
                }
            if(ff_fread(&fchar,1,1,f)!=1)  //failed to read
                return;
        } while (fchar!=0x0A && fchar!=0x0d);
    } while(is_comment==true);
    
}

int load_output_mixer(char filename[])
{
printf("[MIX] Attempting to load %s\r\n",filename);
char filepath[64];
strncpy(filepath,MOUNT_POINT,sizeof(filepath));
strncat(filepath,filename,sizeof(filepath)-1);
FF_FILE *f=ff_fopen(filepath,"r");
if(f==NULL)
    {
        printf("[MIX] failed to load mixer\r\n");
        return 0;
    }
char line[128];
unsigned char expr_index=0;
do
{
get_config_line(line,sizeof(line),f);
printf("[MIX] Attempting to parse expression %s\r\n",line);
struct expr *e=expr_create(line,strlen(line),&vars,user_funcs);
if(e!=NULL)
    {
    printf("[MIX] Parse successful\r\n");
    expressions[expr_index]=e;
    expr_index++;
    }
    else
    {
    printf("[MIX] Parse failed");
    }
} while (line[0]!=0 && expr_index<MAX_EXPRESSION_COUNT);
ff_fclose(f);
return 1;
}



void init_output_mixer(void)
{
char config_file[32]=DEFAULT_CONFIG_FILE;
char line[64]={0};
char filepath[64];
printf("[MIX] Attempting to load mixers\r\n");
strncpy(filepath,MOUNT_POINT,sizeof(filepath));
strncat(filepath,LOOKUP_CONFIG_FILE,sizeof(filepath)-1);
FF_FILE *f=ff_fopen(filepath,"r");
if(f==NULL)
    {
    printf("[MIX] No lookup file found %s\r\n",LOOKUP_CONFIG_FILE);
    load_output_mixer(config_file);
    return;
    }
printf("[MIX] Found mixer lookup file\r\n");
get_config_line(line,sizeof(line),f);
while(line[0]!=0)
    {
        printf("[MIX] Read lookup line %s\r\n", line);
        short int VID,PID;
        int i=0;
        int items=sscanf(line,"%hX:%hX %s",&VID,&PID,config_file);
        if(items!=3)
            {
            get_config_line(line,sizeof(line),f);
            continue;                
            }
        if(VID!=_VID || PID!=_PID)
            {
                printf("[MIX] PID/VID mismatch on line");
                get_config_line(line,sizeof(line),f);
                continue;
            }
        FF_Stat_t file_stats;
        strncpy(filepath,MOUNT_POINT,sizeof(filepath));
        strncat(filepath,config_file,sizeof(filepath)-1);
        int exists=ff_stat(filepath,&file_stats);
        if(exists==0 && load_output_mixer(config_file)==1)
            {
            ff_fclose(f);
            return;
            }
        get_config_line(line,sizeof(line),f);
    }
printf("[MIX] done\r\n");
ff_fclose(f);
}
//#endif

void signal_new_device(unsigned short VID, unsigned short PID)
{
_VID=VID;
_PID=PID;
printf("[MIX] new device %d %d\r\n",_VID,_PID);
xEventGroupSetBits(output_ready_event_handle,DEVICE_AVAILABLE_BIT);
}


void signal_device_gone(void)
{
    _VID=0;
    _PID=0;
    #warning reimplement
    //output_clear();
    if(xSemaphoreTake(vars_mutex,100)==pdTRUE)
        {
        for(int i=0;i<sizeof(axis_lookups)/sizeof(axis_lookup_t);i++)
            {
                axis_lookups[i].count=0;
            }

        for(int i=0;i<MAX_EXPRESSION_COUNT;i++)
            {
                if(expressions[i]!=NULL)
                    {
                    expr_destroy(expressions[i],0);
                    expressions[i]=0;
                    }
            }
        expr_destroy(NULL,&vars);
        vars.head=NULL;
        xSemaphoreGive(vars_mutex);
        }
    

}