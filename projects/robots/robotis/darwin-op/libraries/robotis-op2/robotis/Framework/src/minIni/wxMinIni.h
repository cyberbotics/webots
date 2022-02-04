/*  minIni - Multi-Platform INI file parser, wxWidgets interface
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
 *  Version: $Id: wxMinIni.h 30 2010-07-06 15:49:17Z thiadmer.riemersma $
 */
#ifndef WXMININI_H
#define WXMININI_H

#include <wx/wx.h>

class minIni
{
public:
  minIni(const wxString& filename) : iniFilename(filename)
    { }

  long getl(const wxString& Section, const wxString& Key, long DefValue=0) const
    { return ini_getl(Section.utf8_str(), Key.utf8_str(), DefValue, iniFilename.utf8_str()); }

  int geti(const wxString& Section, const wxString& Key, int DefValue=0) const
    { return static_cast<int>ini_getl(Section.utf8_str(), Key.utf8_str(), (long)DefValue, iniFilename.utf8_str()); }

  wxString gets(const wxString& Section, const wxString& Key, const wxString& DefValue=wxT("")) const
    {
    char buffer[INI_BUFFERSIZE];
    ini_gets(Section.utf8_str(), Key.utf8_str(), DefValue.utf8_str(), buffer, INI_BUFFERSIZE, iniFilename.utf8_str());
    wxString result = wxString::FromUTF8(buffer);
    return result;
    }

  wxString getsection(int idx) const
    {
    char buffer[INI_BUFFERSIZE];
    ini_getsection(idx, buffer, INI_BUFFERSIZE, iniFilename.utf8_str());
    wxString result = wxString::FromUTF8(buffer);
    return result;
    }

  wxString getkey(const wxString& Section, int idx) const
    {
    char buffer[INI_BUFFERSIZE];
    ini_getkey(Section.c_str(), idx, buffer, INI_BUFFERSIZE, iniFilename.utf8_str());
    wxString result = wxString::FromUTF8(buffer);
    return result;
    }

#if ! defined INI_READONLY
  bool put(const wxString& Section, const wxString& Key, double Value) const
    { return (bool)ini_putd(Section.utf8_str(), Key.utf8_str(), Value, iniFilename.utf8_str()); }

  bool put(const wxString& Section, const wxString& Key, long Value) const
    { return (bool)ini_putl(Section.utf8_str(), Key.utf8_str(), Value, iniFilename.utf8_str()); }

  bool put(const wxString& Section, const wxString& Key, int Value) const
    { return (bool)ini_putl(Section.utf8_str(), Key.utf8_str(), (long)Value, iniFilename.utf8_str()); }

  bool put(const wxString& Section, const wxString& Key, const wxString& Value) const
    { return (bool)ini_puts(Section.utf8_str(), Key.utf8_str(), Value.utf8_str(), iniFilename.utf8_str()); }

  bool del(const wxString& Section, const wxString& Key) const
    { return (bool)ini_puts(Section.utf8_str(), Key.utf8_str(), 0, iniFilename.utf8_str()); }

  bool del(const wxString& Section) const
    { return (bool)ini_puts(Section.utf8_str(), 0, 0, iniFilename.utf8_str()); }
#endif

private:
  wxString iniFilename;
};

#endif /* WXMININI_H */
