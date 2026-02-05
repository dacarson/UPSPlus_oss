#include <stdlib.h>

/*
 * Override the exit function to prevent the system from linking stdc that 
 * contains the _exit function. Saving flash.
 */
void exit(int status)
{
    extern void _exit(int);
    _exit(status);
}
