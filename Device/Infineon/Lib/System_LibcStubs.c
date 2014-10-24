/****************************************************************************/
/**
* @file     System_LibcStubs.c
*           XMC4000 Device Series
* @version  V1.3
* @date     Jan 2014
*
* Copyright (C) 2012-2014 Infineon Technologies AG. All rights reserved.
*
*
* @par
* Infineon Technologies AG (Infineon) is supplying this software for use with 
* Infineon's microcontrollers.  This file can be freely distributed within
* development tools that are supporting such microcontrollers.
*
* @par
* THIS SOFTWARE IS PROVIDED AS IS.  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
* OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
* ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
* CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
*
******************************************************************************/

/* ============================ REVISION HISTORY ==============================
   1. Prakash Kalanjeri Balasubramanian , V0.1 , Initial draft
   2. Prakash Kalanjeri Balasubramanian , V0.2 , Label updates
   3. Prakash Kalanjeri Balasubramanian , V1.0 , Made _Sbrk device agnostic
   3. Prakash Kalanjeri Balasubramanian , V1.1 , C++ support
   3. Prakash Kalanjeri Balasubramanian , V1.2 , Restored compatibilty with old
                                                 project files
   4. Prakash Kalanjeri Balasubramanian, V1.3 ,  Encapsulating everything in
                                                 this file for use only with
                                                 GCC
   ========================================================================= */

/*
 * This file contains stubs for standard C library functionality that must
 * typically be provided for the underlying platform.
 *
 * All routines are WEAKLY defined. This creates an opportunity for application
 * developers to override the provided implementation and define a final
 * implementation for their platforms.
 */
#if   defined ( __GNUC__ )

#include <sys/stat.h>
#include <sys/times.h>
#include <unistd.h>

/* ========================================================================= */
/* =========================== File I/O related ============================ */
/* ========================================================================= */
/*
 * File open
 */
__attribute__((weak)) int _open(const char *name, int flags, int mode)
{
 flags = flags;
 mode = mode;
 return -1;
}

/*
 * File position seek
 */
__attribute__((weak)) int _lseek(int file, int offset, int whence)
{
 file = file;
 offset = offset;
 whence = whence;
 return -1;
}

/*
 * File read
 */
__attribute__((weak)) int _read(int file, char *ptr, int len)
{
 file = file;
 len  = len;
 return 0;
}

/*
 * File write
 */
__attribute__((weak)) int _write(int file, char *buf, int nbytes)
{
 return -1;
}

/*
 * File close
 */
__attribute__((weak)) int _close(void)
{
 return -1;
}

/*
 * File status
 */
__attribute__((weak)) int _fstat(int file, struct stat *st)
{
 file = file;
 if(st)
  return -1;
 else
  return -2;
}
/*
 * File linking
 */
__attribute__((weak)) int _link (char *old, char *new)
{
 if (old == new)
  return -1;
 else
  return -2;
}

/*
 * Unlinking directory entry
 */
__attribute__((weak)) int _unlink(char *name)
{
 return -1;
}
/* ========================================================================= */
/* =================== Dynamic memory management related =================== */
/* ========================================================================= */
/*
 * Heap break (position)
 */
__attribute__((weak)) void *_sbrk(int RequestedSize)
{
 /* Heap limits from linker script file */
 extern unsigned int Heap_Bank1_Start;
 extern unsigned int Heap_Bank1_Size;

 unsigned char *CurrBreak, *NextBreak; 
 unsigned int  HeapSize;
 static unsigned char *HeapBound;
 static unsigned char * heap= (unsigned char *)NULL;


 HeapSize   = (unsigned int)(&Heap_Bank1_Size);

 /*
  * If this is the first time malloc() was invoked, we start with the
  * begining of the heap.
  */
 if(heap == (unsigned char *)NULL)
  {
   heap = (unsigned char *)&Heap_Bank1_Start;
   HeapBound  = (unsigned char *) (heap + HeapSize);
  }

 /* Super duper algo to find out if we have memory for the latest request */
 /* Given conditions are: */
 /* 1. Latest break */
 CurrBreak = heap;

 /* And 2. Potential break based on requested size */
 NextBreak = (unsigned char *)( (((unsigned int)(heap)) + RequestedSize + 7)
                                          & 0xFFFFFFF8);

 /* Return no memory condition if we sense we are crossing the limit */
 if (NextBreak >=  HeapBound )
  return ((unsigned char *)NULL);
 else
 {
  heap = NextBreak;
  return CurrBreak;
 }
}

/* ========================================================================= */
/* ====================== Process related ================================== */
/* ========================================================================= */
/*
 * Process timing information
 */
__attribute__((weak)) int _times(struct tms *buf)
{
 return -1;
}
/*
 * Waiting for a child process to complete
 */
__attribute__((weak)) int _wait(int *status)
{
 return -1;
}

/*
 * Kill a process
 */
__attribute__((weak)) int _kill(int pid,int sig)
{
 pid = pid;
 sig = sig;
 return -1;
}

/*
 * Forking a child process
 */
__attribute__((weak)) int _fork(void)
{
 return -1;
}

/*
 * Process ID
 */
__attribute__((weak)) int _getpid(void)
{
 return -1;
}

/*
 * Program/process exit
 */
__attribute__((weak)) void _exit(int rc)
{
 rc = rc;
 while(1){}
}

/* Init */
__attribute__((weak)) void _init(void)
{}

/* ========================================================================= */
/* ======================= TERMIO related ================================== */
/* ========================================================================= */

/*
 * Terminal type evaluation
 */
__attribute__((weak)) int _isatty(int file)
{
 file = file;
 return -1;
}

/* ========================================================================= */
/* ================================= C++ =================================== */
/* ========================================================================= */
void *__dso_handle = (void *)0;

#endif /* __GNUC__ */
