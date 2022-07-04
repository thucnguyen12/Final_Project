#include "utilities.h"
#include <string.h>
#include <stdlib.h>

uint32_t utilities_get_number_from_string(uint32_t index, char *buffer)
{
    // assert(buffer);

    uint32_t value = 0;
    uint16_t tmp = index;

    while (buffer[tmp] && tmp < 1024)
    {
        if (buffer[tmp] >= '0' && buffer[tmp] <= '9')
        {
            value *= 10;
            value += buffer[tmp] - 48;
        }
        else
        {
            break;
        }
        tmp++;
    }

    return value;
}

uint32_t utilities_get_number_from_hex_string(uint32_t index, char *buffer)
{
    // assert(buffer);

	char *p = buffer;
    uint32_t value;

    // Remove 0x
    p = strstr(buffer, "0x");
    if (p)
    {
    	p += 2;
    }
    else
    {
    	p = strstr(buffer, "0X");
    	if (p)
    	{
    		p += 2;
    	}
    }
    if (!p)
    {
    	p = buffer;
    }

    value = (uint32_t)strtol(p, NULL, 16);

    return value;
}
