// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "WbMacAddress.hpp"

#include <QtCore/QObject>

static WbMacAddress *gInstance = NULL;

WbMacAddress *WbMacAddress::instance() {
  if (gInstance == NULL)
    gInstance = new WbMacAddress();
  return gInstance;
}

bool WbMacAddress::check(const QString &macAddress) const {
  return macAddress.compare(address(), Qt::CaseInsensitive) == 0;
}

QString WbMacAddress::address() const {
  return QString::asprintf("%02X%02X%02X%02X%02X%02X", mAddress[0], mAddress[1], mAddress[2], mAddress[3], mAddress[4],
                           mAddress[5]);
}

#ifdef _WIN32

#include <winsock2.h>

#include <iphlpapi.h>

WbMacAddress::WbMacAddress() {
  for (int i = 0; i < 6; i++)
    mAddress[i] = 0;
  DWORD ni;
  GetNumberOfInterfaces(&ni);
  IP_ADAPTER_INFO AdapterInfo[ni];
  DWORD dwBufLen = ni * sizeof(IP_ADAPTER_INFO);
  DWORD dwStatus = GetAdaptersInfo(AdapterInfo, &dwBufLen);
  bool found = false;
  if (dwStatus == ERROR_SUCCESS) {
    bool found_ethernet = false;
    PIP_ADAPTER_INFO pAdapterInfo = AdapterInfo;
    do {
      bool vmware = (pAdapterInfo->Address[0] == 0x00 && pAdapterInfo->Address[1] == 0x50 && pAdapterInfo->Address[2] == 0x56);
      bool ppp = (pAdapterInfo->Address[0] == 0x44 && pAdapterInfo->Address[1] == 0x45 && pAdapterInfo->Address[2] == 0x53 &&
                  pAdapterInfo->Address[3] == 0x54 && pAdapterInfo->Address[4] == 0x00 && pAdapterInfo->Address[5] == 0x00);
      bool ppp_98 = (pAdapterInfo->Address[0] == 0x44 && pAdapterInfo->Address[1] == 0x45 && pAdapterInfo->Address[2] == 0x53 &&
                     pAdapterInfo->Address[3] == 0x54 && pAdapterInfo->Address[4] == 0x42 && pAdapterInfo->Address[5] == 0x00);
      bool ppp_xp = (pAdapterInfo->Address[0] == 0x00 && pAdapterInfo->Address[1] == 0x53 && pAdapterInfo->Address[2] == 0x45 &&
                     pAdapterInfo->Address[3] == 0x00 && pAdapterInfo->Address[4] == 0x00 && pAdapterInfo->Address[5] == 0x00);
      if (!(vmware || ppp || ppp_98 || ppp_xp) &&
          (pAdapterInfo->Type == MIB_IF_TYPE_ETHERNET || pAdapterInfo->Type == IF_TYPE_IEEE80211)) {
        if (found_ethernet == false) {
          // we get preferably the MAC address of the ethernet card (not wifi, unless no ethernet is found)
          found_ethernet = (pAdapterInfo->Type == MIB_IF_TYPE_ETHERNET);
          for (int i = 0; i < 6; i++)
            mAddress[i] = pAdapterInfo->Address[i];
          found = true;
        }
      }
      pAdapterInfo = pAdapterInfo->Next;
    } while (pAdapterInfo);
  }
  if (!found)
    mError = QObject::tr("No active network adapter found.") + "\n" +
             QObject::tr("An active network adapter is required to run Webots.");
}

#elif defined(__linux__)

#include <net/if.h>  // contains the struct ifreq definition
#include <netdb.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>

WbMacAddress::WbMacAddress() {
  for (int i = 0; i < 6; i++)
    mAddress[i] = 0;
  int fd;
  struct ifreq ifr;

  // list all network interfaces from /proc/net/dev except "lo"
  // and store them in interfaces
  QByteArrayList interfaces;
  FILE *f = fopen("/proc/net/dev", "r");
  if (f == NULL)
    return;
  size_t n = 1024;
  char *buffer = NULL;
  while (getline(&buffer, &n, f) != -1) {
    char *name = NULL;
    int found = 0;
    size_t i;
    for (i = 0; i < n; i++) {
      if (buffer[i] == 32)
        continue;
      if (buffer[i] == '\0' || buffer[i] == '\n')
        break;
      if (name == NULL)
        name = &buffer[i];
      if (buffer[i] == ':') {
        buffer[i] = '\0';
        found = 1;
        break;
      }
    }
    if (found && strncmp(name, "lo", 2) != 0)
      interfaces.append(name);
  }
  free(buffer);
  fclose(f);

  if (interfaces.size() == 0)
    return;

  // try to find the Ethernet interface which usually matches "eth*" or "en*"
  const QByteArray *interface = NULL;
  foreach (const QByteArray &i, interfaces) {
    if (i.startsWith("eth")) {
      interface = &i;
      break;
    }
  }
  if (interface == NULL) {
    foreach (const QByteArray &i, interfaces) {
      if (i.startsWith("en")) {
        interface = &i;
        break;
      }
    }
  }
  // if none matches the standard Ethernet template name, then use the first one
  if (interface == NULL)
    interface = &interfaces[0];

  fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
  if (fd == -1)
    return;
  memset(&ifr, 0, sizeof(ifr));
  strcpy(ifr.ifr_name, *interface);
  if (ioctl(fd, SIOCGIFHWADDR, &ifr) == 0) {
    unsigned char *ch = reinterpret_cast<unsigned char *>(ifr.ifr_hwaddr.sa_data);
    for (int i = 0; i < 6; i++)
      mAddress[i] = *ch++;
  }
  close(fd);
}

#elif defined(__APPLE__)

#include <ifaddrs.h>
#include <net/if_dl.h>
#include <sys/socket.h>
#include <sys/types.h>

WbMacAddress::WbMacAddress() {
  for (int i = 0; i < 6; i++)
    mAddress[i] = 0;
  struct ifaddrs *ifap, *ifaphead;
  int rtnerr;
  const struct sockaddr_dl *sdl;
  caddr_t ap;
  int alen;
  rtnerr = getifaddrs(&ifaphead);
  if (rtnerr) {
    perror(NULL);
    return;
  }
  for (ifap = ifaphead; ifap; ifap = ifap->ifa_next) {
    if (ifap->ifa_addr->sa_family == AF_LINK) {
      sdl = reinterpret_cast<const struct sockaddr_dl *>(ifap->ifa_addr);
      ap = ((caddr_t)((sdl)->sdl_data + (sdl)->sdl_nlen));
      alen = sdl->sdl_alen;
      if (ap && alen > 0) {
        // ifap->ifa_name should be en0 (for Ethernet) or en1 (for Airport)
        if (strncmp(ifap->ifa_name, "en", 2))
          continue;
        // printf("%s: ", ifap->ifa_name); for (int i=0;i<6;i++) printf("%02X", 0xff&(ap[i])); printf("\n");
        for (int i = 0; i < 6; i++, ap++)
          mAddress[i] = 0xff & (*ap);
        if (strncmp(ifap->ifa_name, "en0", 3) == 0)
          break;  // stop as soon as we find the ethernet adapter
      }
    }
  }
  freeifaddrs(ifaphead);
}

#endif
