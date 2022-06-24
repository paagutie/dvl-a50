#include "dvl_protocol.h"


// CR8 function taked from https://waterlinked.github.io/dvl/dvl-protocol/#serial-protocol
uint8_t crc8(uint8_t *message, int message_length) {
    uint8_t checksum = 0;
    while (message_length > 0) {
        checksum = lookup_table[*message ^ checksum];
        message++;
        message_length--;
        //printf("mess: %c checksum: %03x\n", *message, checksum);
    }
    return checksum;
}

void create_wrp_struct(struct Element *arr_wrp)
{
    for (int i = 0; i < 9; i++)
    {
        if(i==8)
        {
            arr_wrp[i].type = et_int;
            arr_wrp[i].data = malloc(sizeof(long));
        }
        else
        {
            arr_wrp[i].type = et_dbl;
            arr_wrp[i].data = malloc(sizeof(double));
        }
    }
}

void create_wru_struct(struct Element *arr_wru)
{
    for (int i = 0; i < 5; i++)
    {
        if(i==0 || i==3 || i==4)
        {
            arr_wru[i].type = et_int;
            arr_wru[i].data = malloc(sizeof(long));
        }
        if(i==1 || i==2)
        {
            arr_wru[i].type = et_dbl;
            arr_wru[i].data = malloc(sizeof(double));
        }
    }
}

void create_wrz_struct(struct Element *arr_wrz)
{
    /* Initialize element's struct */
    for (int i = 0; i < 19; i++)
    {
        if(i == 3) //valid (3)
        {
            arr_wrz[i].type = et_str;
            arr_wrz[i].data = '\0';
        }
        else if(i == 15 || i == 16 || i == 18) //time_of_validity (15), time_of_transmissionb(16) and status (18)
        {
            arr_wrz[i].type = et_int;
            arr_wrz[i].data = malloc(sizeof(long));
        }
        else // Velocity (0,1,2), altitude (4), fom (5), covariance (6-14) and time (17)
        {
            arr_wrz[i].type = et_dbl;
            arr_wrz[i].data = malloc(sizeof(double));
        }
    }
}


void free_struct(struct Element *array, int size)
{
    /* All data was dynamically allocated, so free each item's data */
    for(int i=0; i<size; i++)
        free(array[i].data);
    free(array);
}

uint8_t split_message(char *str, int size, char *type, struct Element *output)
{
    char *ptr = strstr(str, type);
    if(ptr != NULL)
    {
        fprintf(stderr, "Incoming data: %s", ptr);
        
        //fprintf(stderr, "time: %ld got %s", time,  str);

        //char *ptr = strtok(buffer, "\n");
        char data[180];
        memset(data, '\0', sizeof(data));
        for(int i=4; i <size-5 ; i++ )
        {
            sprintf(&data[i-4], "%c", str[i]);
        }

        printf("count: %s\n", data);

        /*
        char *ptr = strtok(data, ";");
        while(ptr != NULL)
        {
            printf("%s\n", ptr);
            ptr = strtok(NULL, ";"); 
        }
        */

        char *token, *subtoken;
        char *saveptr1, *saveptr2;
        int count = 0;

        for (token = strtok_r(data, ";", &saveptr1); 
            token != NULL; 
            token = strtok_r(NULL, ";", &saveptr1)) {   
            //printf("token:%s\n", token);
            for (subtoken = strtok_r(token, ",", &saveptr2); 
                subtoken != NULL; 
                subtoken = strtok_r(NULL, ",", &saveptr2))
            {

                double ftemp = 0.0;
                unsigned long itemp = 0;
                switch (output[count].type)
                {
                case et_dbl:
                    ftemp = atof(subtoken);
                    *((double*)(output[count].data)) = ftemp;
                    //printf("float: %4.11f\n", *((double*)(output[count].data)));
                    break;
                
                case et_int:
                    itemp = strtol(subtoken, NULL, 10);
                    *((long*)(output[count].data)) = itemp;
                    //printf("int: %ld\n", *((long*)(output[count].data)));
                    break;

                case et_str:
                    output[count].data = (char*)subtoken;
                    //printf("str: %s\n", (char*)output[count].data);
                    break;
                
                default:
                    break;
                }

                count++;
            }
        }

        return 1;
    }
    else
        return 0;

}

uint8_t is_dead_reckoning_reset_successful(char *str)
{
    char *ptr;
    ptr = strstr(str, "wra");
    if(ptr != NULL)
        return 1;
    ptr = strstr(str, "wrn");
    if(ptr != NULL)
        return 0; 

    return 2;
}   