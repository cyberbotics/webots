/*  minIni - Multi-Platform INI file parser, suitable for embedded systems
 *
 *  These routines are in part based on the article "Multiplatform .INI Files"
 *  by Joseph J. Graf in the March 1994 issue of Dr. Dobb's Journal.
 *
 *  Copyright (c) ITB CompuPhase, 2008-2010
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may not
 *  use this file except in compliance with the License. You may obtain a copy
 *  of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
 *
 *  Version: $Id: minIni.c 29 2010-07-06 13:38:05Z thiadmer.riemersma $
 */

#if (defined _UNICODE || defined __UNICODE__ || defined UNICODE) && !defined MININI_ANSI
# if !defined UNICODE   /* for Windows */
#   define UNICODE
# endif
# if !defined _UNICODE  /* for C library */
#   define _UNICODE
# endif
#endif

#define MININI_IMPLEMENTATION
#include "minIni.h"
#if defined NDEBUG
  #define assert(e)
#else
  #include <assert.h>
#endif

#if !defined __T
  #include <string.h>
  #include <stdlib.h>
  /* definition of TCHAR already in minIni.h */
  #define __T(s)    s
  #define _tcscat   strcat
  #define _tcschr   strchr
  #define _tcscmp   strcmp
  #define _tcscpy   strcpy
  #define _tcsicmp  stricmp
  #define _tcslen   strlen
  #define _tcsncpy  strncpy
  #define _tcsnicmp strnicmp
  #define _tcsrchr  strrchr
  #define _tcstol   strtol
  #define _tfgets   fgets
  #define _tfputs   fputs
  #define _tfopen   fopen
  #define _tremove  remove
  #define _trename  rename
#endif

#if defined __linux || defined __linux__
  #define __LINUX__
#endif
#if defined FREEBSD && !defined __FreeBSD__
  #define __FreeBSD__
#endif
#if !defined strnicmp
  #if defined __LINUX__ || defined __FreeBSD__ || defined __OpenBSD__ || defined __APPLE__
    #define strnicmp  strncasecmp
  #endif
#endif

#if !defined INI_LINETERM
  #define INI_LINETERM    __T("\n")
#endif
#if !defined INI_FILETYPE
  #define INI_FILETYPE    FILE*
#endif

#if !defined sizearray
  #define sizearray(a)    (sizeof(a) / sizeof((a)[0]))
#endif

typedef enum {
  QUOTE_NONE,
  QUOTE_ENQUOTE,
  QUOTE_DEQUOTE,
}quote_option;

static TCHAR *skipleading(const TCHAR *str)
{
  assert(str != NULL);
  while (*str != '\0' && *str <= ' ')
    str++;
  return (TCHAR *)str;
}

static TCHAR *skiptrailing(const TCHAR *str, const TCHAR *base)
{
  assert(str != NULL);
  assert(base != NULL);
  while (str > base && *(str-1) <= ' ')
    str--;
  return (TCHAR *)str;
}

static TCHAR *striptrailing(TCHAR *str)
{
  TCHAR *ptr = skiptrailing(_tcschr(str, '\0'), str);
  assert(ptr != NULL);
  *ptr='\0';
  return str;
}

static TCHAR *save_strncpy(TCHAR *dest, const TCHAR *source, size_t maxlen, quote_option option)
{
  size_t d, s;

  assert(maxlen>0);
  if (option == QUOTE_ENQUOTE && maxlen < 3)
    option = QUOTE_NONE;  /* cannot store two quotes and a terminating zero in less than 3 characters */

  switch (option) {
  case QUOTE_NONE:
    _tcsncpy(dest,source,maxlen);
    dest[maxlen-1]='\0';
    break;
  case QUOTE_ENQUOTE:
    d = 0;
    dest[d++] = '"';
    for (s = 0; source[s] != '\0' && d < maxlen - 2; s++, d++) {
      if (source[s] == '"') {
        if (d >= maxlen - 3)
          break;  /* no space to store the escape character plus the one that follows it */
        dest[d++] = '\\';
      } /* if */
      dest[d] = source[s];
    } /* for */
    dest[d++] = '"';
    dest[d] = '\0';
    break;
  case QUOTE_DEQUOTE:
    for (d = s = 0; source[s] != '\0' && d < maxlen - 1; s++, d++) {
      if ((source[s] == '"' || source[s] == '\\') && source[s + 1] == '"')
        s++;
      dest[d] = source[s];
    } /* for */
    dest[d] = '\0';
    break;
  default:
    assert(0);
  } /* switch */

  return dest;
}

static int getkeystring(INI_FILETYPE *fp, const TCHAR *Section, const TCHAR *Key,
                        int idxSection, int idxKey, TCHAR *Buffer, int BufferSize)
{
  TCHAR *sp, *ep;
  int len, idx, isstring;
  TCHAR LocalBuffer[INI_BUFFERSIZE];

  assert(fp != NULL);
  /* Move through file 1 line at a time until a section is matched or EOF. If
   * parameter Section is NULL, only look at keys above the first section. If
   * idxSection is postive, copy the relevant section name.
   */
  len = (Section != NULL) ? _tcslen(Section) : 0;
  if (len > 0 || idxSection >= 0) {
    idx = -1;
    do {
      if (!ini_read(LocalBuffer, INI_BUFFERSIZE, fp))
        return 0;
      sp = skipleading(LocalBuffer);
      ep = _tcschr(sp, ']');
    } while (*sp != '[' || ep == NULL || (((int)(ep-sp-1) != len || _tcsnicmp(sp+1,Section,len) != 0) && ++idx != idxSection));
    if (idxSection >= 0) {
      if (idx == idxSection) {
        assert(ep != NULL);
        assert(*ep == ']');
        *ep = '\0';
        save_strncpy(Buffer, sp + 1, BufferSize, QUOTE_NONE);
        return 1;
      } /* if */
      return 0; /* no more section found */
    } /* if */
  } /* if */

  /* Now that the section has been found, find the entry.
   * Stop searching upon leaving the section's area.
   */
  assert(Key != NULL || idxKey >= 0);
  len = (Key != NULL) ? (int)_tcslen(Key) : 0;
  idx = -1;
  do {
    if (!ini_read(LocalBuffer,INI_BUFFERSIZE,fp) || *(sp = skipleading(LocalBuffer)) == '[')
      return 0;
    sp = skipleading(LocalBuffer);
    ep = _tcschr(sp, '='); /* Parse out the equal sign */
    if (ep == NULL)
      ep = _tcschr(sp, ':');
  } while (*sp == ';' || *sp == '#' || ep == NULL || (((int)(skiptrailing(ep,sp)-sp) != len || _tcsnicmp(sp,Key,len) != 0) && ++idx != idxKey));
  if (idxKey >= 0) {
    if (idx == idxKey) {
      assert(ep != NULL);
      assert(*ep == '=' || *ep == ':');
      *ep = '\0';
      striptrailing(sp);
      save_strncpy(Buffer, sp, BufferSize, QUOTE_NONE);
      return 1;
    } /* if */
    return 0;   /* no more key found (in this section) */
  } /* if */

  /* Copy up to BufferSize chars to buffer */
  assert(ep != NULL);
  assert(*ep == '=' || *ep == ':');
  sp = skipleading(ep + 1);
  /* Remove a trailing comment */
  isstring = 0;
  for (ep = sp; *ep != '\0' && ((*ep != ';' && *ep != '#') || isstring); ep++) {
    if (*ep == '"') {
      if (*(ep + 1) == '"')
        ep++;                 /* skip "" (both quotes) */
      else
        isstring = !isstring; /* single quote, toggle isstring */
    } else if (*ep == '\\' && *(ep + 1) == '"') {
      ep++;                   /* skip \" (both quotes */
    } /* if */
  } /* for */
  assert(ep != NULL && (*ep == '\0' || *ep == ';' || *ep == '#'));
  *ep = '\0';                 /* terminate at a comment */
  striptrailing(sp);
  /* Remove double quotes surrounding a value */
  isstring = QUOTE_NONE;
  if (*sp == '"' && (ep = _tcschr(sp, '\0')) != NULL && *(ep - 1) == '"') {
    sp++;
    *--ep = '\0';
    isstring = QUOTE_DEQUOTE; /* this is a string, so remove escaped characters */
  } /* if */
  save_strncpy(Buffer, sp, BufferSize, (quote_option)isstring);
  return 1;
}

/** ini_gets()
 * \param Section     the name of the section to search for
 * \param Key         the name of the entry to find the value of
 * \param DefValue    default string in the event of a failed read
 * \param Buffer      a pointer to the buffer to copy into
 * \param BufferSize  the maximum number of characters to copy
 * \param Filename    the name and full path of the .ini file to read from
 *
 * \return            the number of characters copied into the supplied buffer
 */
int ini_gets(const TCHAR *Section, const TCHAR *Key, const TCHAR *DefValue,
             TCHAR *Buffer, int BufferSize, const TCHAR *Filename)
{
  INI_FILETYPE fp;
  int ok = 0;

  if (Buffer == NULL || BufferSize <= 0 || Key == NULL)
    return 0;
  if (ini_openread(Filename, &fp)) {
    ok = getkeystring(&fp, Section, Key, -1, -1, Buffer, BufferSize);
    ini_close(&fp);
  } /* if */
  if (!ok)
    save_strncpy(Buffer, DefValue, BufferSize, QUOTE_NONE);
  return _tcslen(Buffer);
}

/** ini_getl()
 * \param Section     the name of the section to search for
 * \param Key         the name of the entry to find the value of
 * \param DefValue    the default value in the event of a failed read
 * \param Filename    the name of the .ini file to read from
 *
 * \return            the value located at Key
 */
long ini_getl(const TCHAR *Section, const TCHAR *Key, long DefValue, const TCHAR *Filename)
{
  TCHAR buff[64];
  int len = ini_gets(Section, Key, __T(""), buff, sizearray(buff), Filename);
  return (len == 0) ? DefValue : _tcstol(buff,NULL,10);
}

/** ini_getd()
 * \param Section     the name of the section to search for
 * \param Key         the name of the entry to find the value of
 * \param DefValue    the default value in the event of a failed read
 * \param Filename    the name of the .ini file to read from
 *
 * \return            the value located at Key
 */
double ini_getd(const TCHAR *Section, const TCHAR *Key, double DefValue, const TCHAR *Filename)
{
  TCHAR buff[64];
  int len = ini_gets(Section, Key, __T(""), buff, sizearray(buff), Filename);
  return (len == 0) ? DefValue : atof(buff);
}

/** ini_getsection()
 * \param idx         the zero-based sequence number of the section to return
 * \param Buffer      a pointer to the buffer to copy into
 * \param BufferSize  the maximum number of characters to copy
 * \param Filename    the name and full path of the .ini file to read from
 *
 * \return            the number of characters copied into the supplied buffer
 */
int  ini_getsection(int idx, TCHAR *Buffer, int BufferSize, const TCHAR *Filename)
{
  INI_FILETYPE fp;
  int ok = 0;

  if (Buffer == NULL || BufferSize <= 0 || idx < 0)
    return 0;
  if (ini_openread(Filename, &fp)) {
    ok = getkeystring(&fp, NULL, NULL, idx, -1, Buffer, BufferSize);
    ini_close(&fp);
  } /* if */
  if (!ok)
    *Buffer = '\0';
  return _tcslen(Buffer);
}

/** ini_getkey()
 * \param Section     the name of the section to browse through, or NULL to
 *                    browse through the keys outside any section
 * \param idx         the zero-based sequence number of the key to return
 * \param Buffer      a pointer to the buffer to copy into
 * \param BufferSize  the maximum number of characters to copy
 * \param Filename    the name and full path of the .ini file to read from
 *
 * \return            the number of characters copied into the supplied buffer
 */
int  ini_getkey(const TCHAR *Section, int idx, TCHAR *Buffer, int BufferSize, const TCHAR *Filename)
{
  INI_FILETYPE fp;
  int ok = 0;

  if (Buffer == NULL || BufferSize <= 0 || idx < 0)
    return 0;
  if (ini_openread(Filename, &fp)) {
    ok = getkeystring(&fp, Section, NULL, -1, idx, Buffer, BufferSize);
    ini_close(&fp);
  } /* if */
  if (!ok)
    *Buffer = '\0';
  return _tcslen(Buffer);
}


#if ! defined INI_READONLY
static void ini_tempname(TCHAR *dest, const TCHAR *source, int maxlength)
{
  TCHAR *p;

  save_strncpy(dest, source, maxlength, QUOTE_NONE);
  p = _tcsrchr(dest, '\0');
  assert(p != NULL);
  *(p - 1) = '~';
}

static quote_option check_enquote(const TCHAR *Value)
{
  const TCHAR *p;

  /* run through the value, if it has trailing spaces, or '"', ';' or '#'
   * characters, enquote it
   */
  assert(Value != NULL);
  for (p = Value; *p != '\0' && *p != '"' && *p != ';' && *p != '#'; p++)
    /* nothing */;
  return (*p != '\0' || (p > Value && *(p - 1) == ' ')) ? QUOTE_ENQUOTE : QUOTE_NONE;
}

static void writesection(TCHAR *LocalBuffer, const TCHAR *Section, INI_FILETYPE *fp)
{
  TCHAR *p;

  if (Section != NULL && _tcslen(Section) > 0) {
    LocalBuffer[0] = '[';
    save_strncpy(LocalBuffer + 1, Section, INI_BUFFERSIZE - 4, QUOTE_NONE);  /* -1 for '[', -1 for ']', -2 for '\r\n' */
    p = _tcsrchr(LocalBuffer, '\0');
    assert(p != NULL);
    *p++ = ']';
    _tcscpy(p, INI_LINETERM); /* copy line terminator (typically "\n") */
    ini_write(LocalBuffer, fp);
  } /* if */
}

static void writekey(TCHAR *LocalBuffer, const TCHAR *Key, const TCHAR *Value, INI_FILETYPE *fp)
{
  TCHAR *p;
  quote_option option = check_enquote(Value);
  save_strncpy(LocalBuffer, Key, INI_BUFFERSIZE - 3, QUOTE_NONE);  /* -1 for '=', -2 for '\r\n' */
  p = _tcsrchr(LocalBuffer, '\0');
  assert(p != NULL);
  *p++ = '=';
  save_strncpy(p, Value, INI_BUFFERSIZE - (p - LocalBuffer) - 2, option); /* -2 for '\r\n' */
  p = _tcsrchr(LocalBuffer, '\0');
  assert(p != NULL);
  _tcscpy(p, INI_LINETERM); /* copy line terminator (typically "\n") */
  ini_write(LocalBuffer, fp);
}

static void write_quoted(const TCHAR *Value, INI_FILETYPE *fp)
{
  TCHAR s[3];
  int idx;
  if (check_enquote(Value) == QUOTE_NONE) {
    ini_write(Value, fp);
  } else {
    ini_write("\"", fp);
    for (idx = 0; Value[idx] != '\0'; idx++) {
      if (Value[idx] == '"') {
        s[0] = '\\';
        s[1] = Value[idx];
        s[2] = '\0';
      } else {
        s[0] = Value[idx];
        s[1] = '\0';
      } /* if */
      ini_write(s, fp);
    } /* for */
    ini_write("\"", fp);
  } /* if */
}

/** ini_puts()
 * \param Section     the name of the section to write the string in
 * \param Key         the name of the entry to write, or NULL to erase all keys in the section
 * \param Value       a pointer to the buffer the string, or NULL to erase the key
 * \param Filename    the name and full path of the .ini file to write to
 *
 * \return            1 if successful, otherwise 0
 */
int ini_puts(const TCHAR *Section, const TCHAR *Key, const TCHAR *Value, const TCHAR *Filename)
{
  INI_FILETYPE rfp;
  INI_FILETYPE wfp;
  TCHAR *sp, *ep;
  TCHAR LocalBuffer[INI_BUFFERSIZE];
  int len, match, count;

  assert(Filename!=NULL);
  if (!ini_openread(Filename, &rfp)) {
    /* If the .ini file doesn't exist, make a new file */
    if (Key!=NULL && Value!=NULL) {
      if (!ini_openwrite(Filename, &wfp))
        return 0;
      writesection(LocalBuffer, Section, &wfp);
      writekey(LocalBuffer, Key, Value, &wfp);
      ini_close(&wfp);
    } /* if */
    return 1;
  } /* if */

  /* If parameters Key and Value are valid (so this is not an "erase" request)
   * and the setting already exists and it already has the correct value, do
   * nothing. This early bail-out avoids rewriting the INI file for no reason.
   */
  if (Key!=NULL && Value!=NULL) {
    match = getkeystring(&rfp, Section, Key, -1, -1, LocalBuffer, sizearray(LocalBuffer));
    if (match && _tcscmp(LocalBuffer,Value)==0) {
      ini_close(&rfp);
      return 1;
    } /* if */
    /* key not found, or different value -> proceed (but rewind the input file first) */
    ini_rewind(&rfp);
  } /* if */

  /* Get a temporary file name to copy to. Use the existing name, but with
   * the last character set to a '~'.
   */
  ini_tempname(LocalBuffer, Filename, INI_BUFFERSIZE);
  if (!ini_openwrite(LocalBuffer, &wfp)) {
    ini_close(&rfp);
    return 0;
  } /* if */

  /* Move through the file one line at a time until a section is
   * matched or until EOF. Copy to temp file as it is read.
   */
  count = 0;
  len = (Section != NULL) ? _tcslen(Section) : 0;
  if (len > 0) {
    do {
      if (!ini_read(LocalBuffer, INI_BUFFERSIZE, &rfp)) {
        /* Failed to find section, so add one to the end */
        if (Key!=NULL && Value!=NULL) {
            ini_write(INI_LINETERM, &wfp);  /* force a new line (there may not have been one) behind the last line of the INI file */
            writesection(LocalBuffer, Section, &wfp);
            writekey(LocalBuffer, Key, Value, &wfp);
        } /* if */
        /* Clean up and rename */
        ini_close(&rfp);
        ini_close(&wfp);
        ini_remove(Filename);
        ini_tempname(LocalBuffer, Filename, INI_BUFFERSIZE);
        ini_rename(LocalBuffer, Filename);
        return 1;
      } /* if */
      /* Copy the line from source to dest, but not if this is the section that
       * we are looking for and this section must be removed
       */
      sp = skipleading(LocalBuffer);
      ep = _tcschr(sp, ']');
      match = (*sp == '[' && ep != NULL && (int)(ep-sp-1) == len && _tcsnicmp(sp + 1,Section,len) == 0);
      if (!match || Key!=NULL) {
        /* Remove blank lines, but insert a blank line (possibly one that was
         * removed on the previous iteration) before a new section. This creates
         * "neat" INI files.
         */
        if (_tcslen(sp) > 0) {
          if (*sp == '[' && count > 0)
            ini_write(INI_LINETERM, &wfp);
          ini_write(sp, &wfp);
          count++;
        } /* if */
      } /* if */
    } while (!match);
  } /* if */

  /* Now that the section has been found, find the entry. Stop searching
   * upon leaving the section's area. Copy the file as it is read
   * and create an entry if one is not found.
   */
  len = (Key!=NULL) ? _tcslen(Key) : 0;
  for( ;; ) {
    if (!ini_read(LocalBuffer, INI_BUFFERSIZE, &rfp)) {
      /* EOF without an entry so make one */
      if (Key!=NULL && Value!=NULL) {
          ini_write(INI_LINETERM, &wfp);  /* force a new line (there may not have been one) behind the last line of the INI file */
          writekey(LocalBuffer, Key, Value, &wfp);
      } /* if */
      /* Clean up and rename */
      ini_close(&rfp);
      ini_close(&wfp);
      ini_remove(Filename);
      ini_tempname(LocalBuffer, Filename, INI_BUFFERSIZE);
      ini_rename(LocalBuffer, Filename);
      return 1;
    } /* if */
    sp = skipleading(LocalBuffer);
    ep = _tcschr(sp, '='); /* Parse out the equal sign */
    if (ep == NULL)
      ep = _tcschr(sp, ':');
    match = (ep != NULL && (int)(skiptrailing(ep,sp)-sp) == len && _tcsnicmp(sp,Key,len) == 0);
    if ((Key!=NULL && match) || *sp == '[')
      break;  /* found the key, or found a new section */
    /* in the section that we re-write, do not copy empty lines */
    if (Key!=NULL && _tcslen(sp) > 0)
      ini_write(sp, &wfp);
  } /* for */
  if (*sp == '[') {
    /* found start of new section, the key was not in the specified
     * section, so we add it just before the new section
     */
    if (Key!=NULL && Value!=NULL) {
      /* We cannot use "writekey()" here, because we need to preserve the
       * contents of LocalBuffer.
       */
      ini_write(Key, &wfp);
      ini_write("=", &wfp);
      write_quoted(Value, &wfp);
      ini_write(INI_LINETERM INI_LINETERM, &wfp); /* put a blank line between the current and the next section */
    } /* if */
    /* write the new section header that we read previously */
    ini_write(sp, &wfp);
  } else {
    /* We found the key; ignore the line just read (with the key and
     * the current value) and write the key with the new value.
     */
    if (Key!=NULL && Value!=NULL)
      writekey(LocalBuffer, Key, Value, &wfp);
  } /* if */
  /* Copy the rest of the INI file (removing empty lines, except before a section) */
  while (ini_read(LocalBuffer, INI_BUFFERSIZE, &rfp)) {
    sp = skipleading(LocalBuffer);
    if (_tcslen(sp) > 0) {
      if (*sp == '[')
        ini_write(INI_LINETERM, &wfp);
      ini_write(sp, &wfp);
    } /* if */
  } /* while */
  /* Clean up and rename */
  ini_close(&rfp);
  ini_close(&wfp);
  ini_remove(Filename);
  ini_tempname(LocalBuffer, Filename, INI_BUFFERSIZE);
  ini_rename(LocalBuffer, Filename);
  return 1;
}

/* Ansi C "itoa" based on Kernighan & Ritchie's "Ansi C" book. */
#define ABS(v)  ((v) < 0 ? -(v) : (v))

static void strreverse(TCHAR *str)
{
  TCHAR t;
  int i, j;

  for (i = 0, j = _tcslen(str) - 1; i < j; i++, j--) {
    t = str[i];
    str[i] = str[j];
    str[j] = t;
  } /* for */
}

static void long2str(long value, TCHAR *str)
{
  int i = 0;
  long sign = value;
  int n;

  /* generate digits in reverse order */
  do {
    n = (int)(value % 10);              /* get next lowest digit */
    str[i++] = (TCHAR)(ABS(n) + '0');   /* handle case of negative digit */
  } while (value /= 10);                /* delete the lowest digit */
  if (sign < 0)
    str[i++] = '-';
  str[i] = '\0';

  strreverse(str);
}

/** ini_putl()
 * \param Section     the name of the section to write the value in
 * \param Key         the name of the entry to write, or NULL to erase all keys in the section
 * \param Value       the value to write
 * \param Filename    the name and full path of the .ini file to write to
 *
 * \return            1 if successful, otherwise 0
 */
int ini_putl(const TCHAR *Section, const TCHAR *Key, long Value, const TCHAR *Filename)
{
  TCHAR str[32];
  long2str(Value, str);
  return ini_puts(Section, Key, str, Filename);
}

int ini_putd(const TCHAR *Section, const TCHAR *Key, double Value, const TCHAR *Filename)
{
  TCHAR str[32];
  sprintf(str, "%f", Value);
  return ini_puts(Section, Key, str, Filename);
}
#endif /* !INI_READONLY */


#if defined PORTABLE_STRNICMP
int strnicmp(const TCHAR *s1, const TCHAR *s2, size_t n)
{
  register unsigned TCHAR c1, c2;

  while (n-- != 0 && (*s1 || *s2)) {
    c1 = *(const unsigned TCHAR *)s1++;
    if ('a' <= c1 && c1 <= 'z')
      c1 += ('A' - 'a');
    c2 = *(const unsigned TCHAR *)s2++;
    if ('a' <= c2 && c2 <= 'z')
      c2 += ('A' - 'a');
    if (c1 != c2)
      return c1 - c2;
  } /* while */
  return 0;
}
#endif /* PORTABLE_STRNICMP */
