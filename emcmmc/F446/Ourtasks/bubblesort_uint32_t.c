/*******************************************************************************
* File Name          : bubblesort_uint32_t.c
* Date First Issued  : 09/08/2024
* Board              : 
* Description        : Basic bubble sort Rosetta code
*******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "stringchgr_items.h"

/* **************************************************************************************
 * uint32_t bubble_sort_uint32t(struct BMSTABLE *a, uint32_t n) ;
 * @brief   : Sort array with pointers to BMSTABLE entries on CAN id
 * @param   : pc = pointer to array of struct pointers
 * @param   : n  = size of array
 * @return  : number of compares
 * ************************************************************************************** */
uint32_t bubble_sort_uint32t(struct BMSTABLE* a[], uint32_t n) 
{
	uint32_t cmpct = 0;
    int i, j = n, s = 1;
    struct BMSTABLE* t;
    while (s) 
    {
        s = 0;
        for (i = 1; i < j; i++) 
        {
            if (a[i]->id < a[i - 1]->id) 
            {
                t = a[i];
                a[i] = a[i - 1];
                a[i - 1] = t;
                s = 1;
            }
        }
        cmpct += j;
        j--;
    }
    return cmpct;
}
