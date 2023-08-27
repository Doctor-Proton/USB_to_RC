

typedef struct
{
    char name[32];
    float value;

} parameter_t;

int set_param(const char name[], float value);
float get_param(const char name[]);

#define UPPER_US ((uint32_t)get_param("UPPER_US"))
#define LOWER_US ((uint32_t)get_param("LOWER_US"))