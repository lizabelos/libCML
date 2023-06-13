/*
 * jerror.c
 *
 * Copyright (C) 1991-1998, Thomas G. Lane.
 * This file is part of the Independent JPEG Group's software.
 * For conditions of distribution and use, see the accompanying README file.
 *
 * This file contains simple error-reporting and trace-message routines.
 * These are suitable for Unix-like systems and others where writing to
 * stderr is the right thing to do.  Many applications will want to replace
 * some or all of these routines.
 *
 * If you define USE_WINDOWS_MESSAGEBOX in jconfig.h or in the makefile,
 * you get a Windows-specific hack to display error messages in a dialog box.
 * It ain't much, but it beats dropping error messages into the bit bucket,
 * which is what happens to output to stderr under most Windows C compilers.
 *
 * These routines are used by both the compression and decompression code.
 */

/* this is not a core library module, so it doesn't define JPEG_INTERNALS */
#include "jinclude.h"
#include "jpeglib.h"
#include "jversion.h"
#include "jerror.h"

#ifdef USE_WINDOWS_MESSAGEBOX
#include <windows.h>
#endif

#ifndef EXIT_FAILURE    /* define exit() codes if not provided */
#define EXIT_FAILURE  1
#endif


/*
 * Create the message string table.
 * We do this from the master message list in jerror.h by re-reading
 * jerror.h with a suitable definition for macro JMESSAGE.
 * The message table is made an external symbol just in case any applications
 * want to refer to it directly.
 */

#ifdef NEED_SHORT_EXTERNAL_NAMES
#define jpeg_std_message_table  jMsgTable
#endif

#define JMESSAGE(code,string)  string ,

const char * const jpeg_std_message_table[] = {
#include "jerror.h"
  NULL
};

