/*
 * string_parse.c
 *
 *  Created on: 27 Jun 2022
 *      Author: nguyenphuonglinh
 */
#include "string_parse.h"
void getSubStrig(char *source, const char *start, const char *end, char *out)
{
	int j = 0, k = 0, index_start = 0, index_end = 0;
	for(int i = 0; i < strlen(source); i++)
	{
		if (source[i] == start[j])
		{
			j++;
			if (j == strlen(start))
			{
			    index_start = i + 1;
			}
		}
		else j = 0;

		if( source[i] == end[k])
		{
			k++;
			if (k == strlen(end))
			{
				index_end = i;
			}
		}
		else
		{
		    k = 0;
		}
	}
	strncpy(out, source + index_start, index_end - index_start);
}


