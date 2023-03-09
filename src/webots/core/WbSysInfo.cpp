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

#include "WbSysInfo.hpp"

#include "WbMacAddress.hpp"

#include <QtCore/QRegularExpression>
#include <QtCore/QStringList>
#include <QtGui/QOpenGLFunctions>

#include <cassert>

#ifdef __linux__
#include <math.h>
#include <unistd.h>
#include <QtCore/QFile>
#endif

#ifdef _WIN32
#include "WbWindowsRegistry.hpp"

#include <cpuid.h>
#include <d3d9.h>
typedef void(WINAPI *PGNSI)(LPSYSTEM_INFO);
#else
#include <sys/utsname.h>
#endif

#ifdef __APPLE__
#include <IOKit/IOKitLib.h>
#include <mach/mach.h>
#include <sys/sysctl.h>
#endif

#ifdef __WIN32
static quint32 gDeviceId = 0;
static quint32 gVendorId = 0;

static void updateGpuIds(QOpenGLFunctions *gl) {
  static bool firstCall = true;
  if (!firstCall)
    return;
  firstCall = false;
  D3DADAPTER_IDENTIFIER9 adapterinfo;
  LPDIRECT3D9 d3d_Object;

  d3d_Object = Direct3DCreate9(D3D_SDK_VERSION);
  d3d_Object->GetAdapterIdentifier(D3DADAPTER_DEFAULT, 0, &adapterinfo);
  gDeviceId = adapterinfo.DeviceId;
  gVendorId = adapterinfo.VendorId;
  d3d_Object->Release();
}
#endif

const void WbSysInfo::initializeOpenGlInfo() {
  openGLRenderer();
  openGLVendor();
  openGLVersion();
}

const QString &WbSysInfo::openGLRenderer() {
  static QString openGLRender;
  if (openGLRender.isEmpty())
    openGLRender = reinterpret_cast<const char *>(glGetString(GL_RENDERER));
  return openGLRender;
}

const QString &WbSysInfo::openGLVendor() {
  static QString openGLVendor;
  if (openGLVendor.isEmpty())
    openGLVendor = reinterpret_cast<const char *>(glGetString(GL_VENDOR));
  return openGLVendor;
}

const QString &WbSysInfo::openGLVersion() {
  static QString openGLVersion;
  if (openGLVersion.isEmpty())
    openGLVersion = reinterpret_cast<const char *>(glGetString(GL_VERSION));
  return openGLVersion;
}

void WbSysInfo::openGlLineWidthRange(double &min, double &max) {
  GLfloat range[2];
  glGetFloatv(GL_ALIASED_LINE_WIDTH_RANGE, range);
  min = range[0];
  max = range[1];
}

const QString &WbSysInfo::sysInfo() {
  static QString sysInfo;
  // cppcheck-suppress knownConditionTrueFalse
  if (!sysInfo.isEmpty())
    return sysInfo;

#ifdef _WIN32
  sysInfo.append(QSysInfo::prettyProductName());
  sysInfo.append(" ");

  SYSTEM_INFO winSysInfo;
  PGNSI pGetNativeSystemInfo =
    reinterpret_cast<PGNSI>(GetProcAddress(GetModuleHandle(TEXT("kernel32.dll")), "GetNativeSystemInfo"));
  if (NULL != pGetNativeSystemInfo)
    pGetNativeSystemInfo(&winSysInfo);
  else
    GetSystemInfo(&winSysInfo);

  if (winSysInfo.wProcessorArchitecture == PROCESSOR_ARCHITECTURE_INTEL)
    sysInfo.append("32-bit");
  else if (winSysInfo.wProcessorArchitecture == PROCESSOR_ARCHITECTURE_AMD64)
    sysInfo.append("64-bit");
  else if (winSysInfo.wProcessorArchitecture == PROCESSOR_ARCHITECTURE_IA64)
    sysInfo.append("Intel Itanium-based");
  else
    sysInfo.append("unknown architecture");
#else
  struct utsname buf;
  uname(&buf);
  sysInfo.append(buf.sysname);
  sysInfo.append(" ");
  sysInfo.append(buf.release);
  sysInfo.append(" ");
  sysInfo.append(buf.machine);
#endif

  return sysInfo;
}

WbSysInfo::WbPlatform WbSysInfo::platform() {
#ifdef __linux__
  return LINUX_PLATFORM;
#elif defined(__APPLE__)
  return MACOS_PLATFORM;
#elif defined(_WIN32)
  return WIN32_PLATFORM;
#else
#error unsupported platform
#endif
}

const QString &WbSysInfo::platformShortName() {
#ifdef __linux__
  static QString platformShortName;
  if (platformShortName.isEmpty()) {
    // cppcheck-suppress knownConditionTrueFalse
    if (WbSysInfo::isPointerSize64bits())
      platformShortName = "linux64";
    else
      platformShortName = "linux32";
  }
  return platformShortName;
#elif defined(__APPLE__)
  static const QString platformShortName = "mac";
  return platformShortName;
#elif defined(_WIN32)
  static const QString platformShortName = "windows";
  return platformShortName;
#else
#error unsupported platform
#endif
}

const QString &WbSysInfo::processor() {
  static QString processor;
  // cppcheck-suppress knownConditionTrueFalse
  if (!processor.isEmpty())
    return processor;
#ifdef _WIN32
  WbWindowsRegistry cpu("\\HKEY_LOCAL_MACHINE\\HARDWARE\\DESCRIPTION\\System\\CentralProcessor\\0");
  processor = cpu.stringValue("ProcessorNameString");
#elif defined(__APPLE__)
  size_t buflen = 100;
  char buf[buflen];
  sysctlbyname("machdep.cpu.brand_string", &buf, &buflen, NULL, 0);
  processor = buf;
#elif defined(__linux__)
  processor = linuxCpuModelName();
#endif
  return processor;
}

QString WbSysInfo::environmentVariable(const QString &name) {
#ifdef _WIN32  // on Windows, we cannot use the qgetenv function directly as it doesn't support UTF-8 characters
  wchar_t *wname = new wchar_t[name.length() + 1];
  name.toWCharArray(wname);
  wname[name.length()] = 0;
  int size = GetEnvironmentVariableW(wname, NULL, 0);
  if (size == 0)
    return QString();
  wchar_t *wvalue = new wchar_t[size];
  GetEnvironmentVariableW(wname, wvalue, size);
  delete[] wname;
  QString value = QString::fromWCharArray(wvalue);
  delete[] wvalue;
  return value;
#else
  return QString::fromUtf8(qgetenv(name.toUtf8()));
#endif
}

void WbSysInfo::setEnvironmentVariable(const QString &name, const QString &value) {
#ifdef _WIN32  // on Windows, we cannot use the qputenv function directly as it doesn't support UTF-8 characters
  wchar_t *wname = new wchar_t[name.length() + 1];
  name.toWCharArray(wname);
  wname[name.length()] = 0;
  wchar_t *wvalue = new wchar_t[value.length() + 1];
  value.toWCharArray(wvalue);
  wvalue[value.length()] = 0;
  SetEnvironmentVariableW(wname, wvalue);
#else
  qputenv(name.toUtf8(), value.toUtf8());
#endif
}

QString WbSysInfo::shortPath(const QString &path) {
#ifdef _WIN32
  wchar_t *input = new wchar_t[path.length() + 1];
  path.toWCharArray(input);
  input[path.length()] = 0;  // terminate string
  long length = GetShortPathNameW(input, NULL, 0);
  wchar_t *output = new wchar_t[length];
  GetShortPathNameW(input, output, length);
  QString ret = QString::fromWCharArray(output, length - 1);
  delete[] input;
  delete[] output;
  return ret;
#else
  return path;
#endif
}

// This function returns the number of CPU cores which may be different from
// the number of "logical cores" on some processors with hyperthreading
// (e.g., some Intel i7 have 4 CPU cores, but 8 logical cores thanks to
// hyperthreading). The value returned by QThread::idealThreadCount() is
// actually the number of logical cores. However, the optimal value for the
// number of threads in ODE MT is indeed the number of CPU cores. Thus, we
// need this function.
int WbSysInfo::coreCount() {
  static int coreCount = 0;
  if (coreCount > 0)
    return coreCount;

#ifdef _WIN32
  PSYSTEM_LOGICAL_PROCESSOR_INFORMATION buffer = NULL;
  DWORD returnLength = 0;
  GetLogicalProcessorInformation(buffer, &returnLength);
  buffer = (PSYSTEM_LOGICAL_PROCESSOR_INFORMATION)malloc(returnLength);
  GetLogicalProcessorInformation(buffer, &returnLength);
  int max = returnLength / sizeof(SYSTEM_LOGICAL_PROCESSOR_INFORMATION);
  coreCount = 0;
  for (int i = 0; i < max; i++)
    if (buffer[i].Relationship == RelationProcessorCore)
      coreCount++;
  free(buffer);

#elif defined(__linux__)
  coreCount = sysconf(_SC_NPROCESSORS_ONLN);

#elif defined(__APPLE__)
  kern_return_t kr;
  struct host_basic_info hostinfo;
  unsigned int count;

  count = HOST_BASIC_INFO_COUNT;
  kr = host_info(mach_host_self(), HOST_BASIC_INFO, (host_info_t)&hostinfo, &count);
  if (kr == KERN_SUCCESS)
    coreCount = hostinfo.avail_cpus;
#endif

  if (coreCount < 1)
    coreCount = 1;

  return coreCount;
}

#ifdef __linux__
const QString &WbSysInfo::linuxCpuModelName() {
  static QString cpuinfo;
  QFile cpuinfoFile("/proc/cpuinfo");
  if (cpuinfoFile.open(QIODevice::ReadOnly)) {
    const QStringList lines = QString(cpuinfoFile.readAll()).split('\n');
    foreach (const QString line, lines) {
      if (line.startsWith("model name")) {
        // 12 corresponds to the strlen("model name: ")
        cpuinfo = line.mid(12).trimmed();  // remove leading and trailing whitespace
        break;
      }
    }
  }
  return cpuinfo;
}

bool WbSysInfo::isRootUser() {
  return geteuid() == 0;
}
#endif

bool WbSysInfo::isPointerSize32bits() {
  // cppcheck-suppress knownConditionTrueFalse
  return !WbSysInfo::isPointerSize64bits();
}

bool WbSysInfo::isPointerSize64bits() {
  return (sizeof(void *) == 8);
}

bool WbSysInfo::isVirtualMachine() {
  static char virtualMachine = -1;
  if (virtualMachine == 1)
    return true;
  if (virtualMachine == 0)
    return false;
  // list taken from https://www.techrepublic.com/blog/data-center/mac-address-scorecard-for-common-virtual-machine-platforms/
  // these are MAC addresses generated by default by virtual machines
  // however virtual machine also allow users to define a custom MAC address
  const QStringList virtualMachineMacIdentifiers = {
    "005056", "000C29", "000569",  // VMware ESX3, Server, Workstation, Player
    "0003FF",                      // Microsoft Hyper-V, Virtual Server, Virtual PC
    "001C42",                      // Parallells Desktop, Workstation, Server, Virtuozzo
    "000F4B",                      // Virtual Iron 4
    "00163E",                      // Red Hat Xen, Oracle VM, XenSource, Novell Xen
    "080027"                       // Sun xVM VirtualBox
  };
  const QString macAddress = WbMacAddress::instance()->address();
  foreach (const QString &id, virtualMachineMacIdentifiers) {
    if (macAddress.startsWith(id)) {
      virtualMachine = 1;
      return true;
    }
  }
// this is a more reliable way to determine if we are running on a virtual machine
#ifdef _WIN32
  unsigned int eax = 0, ebx = 0, ecx = 0, edx = 0;
  __get_cpuid(0x1, &eax, &ebx, &ecx, &edx);
  if (!(ecx & ((unsigned int)1 << 31))) {
    virtualMachine = 0;
    return false;
  }
  const auto queryVendorIdMagic = 0x40000000;
  __get_cpuid(queryVendorIdMagic, &eax, &ebx, &ecx, &edx);
  const int vendorIdLength = 13;
  using VendorIdStr = char[vendorIdLength];
  VendorIdStr hyperVendorId = {};
  memcpy(hyperVendorId + 0, &ebx, 4);
  memcpy(hyperVendorId + 4, &ecx, 4);
  memcpy(hyperVendorId + 8, &edx, 4);
  hyperVendorId[12] = '\0';
  static const VendorIdStr vendors[]{
    "KVMKVMKVM\0\0\0",  // KVM
    "Microsoft Hv",     // Microsoft Hyper-V or Windows Virtual PC */
    "VMwareVMware",     // VMware
    "XenVMMXenVMM",     // Xen
    "prl hyperv  ",     // Parallels
    "VBoxVBoxVBox"      // VirtualBox
  };
  for (const auto &vendor : vendors) {
    if (!memcmp(vendor, hyperVendorId, vendorIdLength)) {
      virtualMachine = 1;
      return true;
    }
  }
  virtualMachine = 0;
  return false;
#else
#ifdef __linux__
  QFile cpuinfoFile("/proc/cpuinfo");
  if (!cpuinfoFile.open(QIODevice::ReadOnly)) {
    virtualMachine = 1;
    return true;  // unable to determine, assuming true
  }
  const QStringList lines = QString(cpuinfoFile.readAll()).split('\n');
  foreach (const QString &line, lines) {
    if (!line.startsWith("flags"))
      continue;
    const QStringList tokens = line.mid(line.indexOf(":") + 1).trimmed().split(" ");
    foreach (const QString &token, tokens)
      if (token == "hypervisor") {
        virtualMachine = 1;
        return true;
      }
    break;
  }
#endif
#ifdef __APPLE__
  if (system("ioreg -l | grep -e Manufacturer -e 'Vendor Name' | grep -E 'VMware|VirtualBox|Oracle|Parallels' > /dev/null") ==
      0) {
    virtualMachine = 1;
    return true;
  }
#endif
  virtualMachine = 0;
  return false;
#endif  // _WIN32
}

#ifdef _WIN32
quint32 WbSysInfo::gpuDeviceId(QOpenGLFunctions *gl) {
  updateGpuIds(gl);
  return gDeviceId;
}

quint32 WbSysInfo::gpuVendorId(QOpenGLFunctions *gl) {
  updateGpuIds(gl);
  return gVendorId;
}

int WbSysInfo::intelGPUGeneration(QOpenGLFunctions *gl) {
  // http://en.wikipedia.org/wiki/Comparison_of_Intel_graphics_processing_units or
  // https://cgit.freedesktop.org/mesa/mesa/tree/include/pci_ids/i965_pci_ids.h
  // TODO: to maintain

  quint32 id = gpuDeviceId(gl);

  // ninth generation
  if (
    // Skylake
    id == 0x1902 || id == 0x1906 || id == 0x190A || id == 0x190B || id == 0x190E || id == 0x1912 || id == 0x1913 ||
    id == 0x1915 || id == 0x1916 || id == 0x1917 || id == 0x191A || id == 0x191B || id == 0x191D || id == 0x191E ||
    id == 0x1921 || id == 0x1923 || id == 0x1926 || id == 0x1927 || id == 0x192A || id == 0x192B || id == 0x192D ||
    id == 0x1932 || id == 0x193A || id == 0x193B || id == 0x193D ||
    // Broxton / Apollo Lake
    id == 0x0A84 || id == 0x1A84 || id == 0x1A85 || id == 0x5A84 || id == 0x5A85 ||
    // Kabylake
    id == 0x5902 || id == 0x5906 || id == 0x5908 || id == 0x590A || id == 0x590B || id == 0x590E || id == 0x5912 ||
    id == 0x5913 || id == 0x5915 || id == 0x5916 || id == 0x5917 || id == 0x591A || id == 0x591B || id == 0x591D ||
    id == 0x591E || id == 0x5921 || id == 0x5923 || id == 0x5926 || id == 0x5927 || id == 0x593B)
    return 9;

  // eighth generation
  if (
    // Broadwell
    id == 0x1602 || id == 0x1606 || id == 0x160A || id == 0x160B || id == 0x160D || id == 0x160E || id == 0x1612 ||
    id == 0x1616 || id == 0x161A || id == 0x161B || id == 0x161D || id == 0x161E ||
    // Cherryview
    id == 0x1622 || id == 0x1626 || id == 0x162A || id == 0x162B || id == 0x162D || id == 0x162E || id == 0x1632 ||
    id == 0x1636 || id == 0x163A || id == 0x163B || id == 0x163D || id == 0x163E || id == 0x22B0 || id == 0x22B1 ||
    id == 0x22B2 || id == 0x22B3)
    return 8;

  // seventh generation
  if (
    // HD Graphics, HD Graphics 2500, HD Graphics 4000, HD Graphics P4000
    id == 0x0162 || id == 0x0166 || id == 0x016A || id == 0x0152 || id == 0x0156 || id == 0x015A ||
    // HD Graphics, HD Graphics 4200, HD Graphics 4400, HD Graphics 4600, HD Graphics 5000, HD Graphics 5100, HD Graphics 5200
    id == 0x0402 || id == 0x0412 || id == 0x0422 || id == 0x0406 || id == 0x0416 || id == 0x0426 || id == 0x040A ||
    id == 0x041A || id == 0x042A || id == 0x040B || id == 0x041B || id == 0x042B || id == 0x040E || id == 0x041E ||
    id == 0x042E || id == 0x0A02 || id == 0x0A12 || id == 0x0A22 || id == 0x0A06 || id == 0x0A16 || id == 0x0A26 ||
    id == 0x0A0A || id == 0x0A1A || id == 0x0A2A || id == 0x0A0B || id == 0x0A1B || id == 0x0A2B || id == 0x0A0E ||
    id == 0x0A1E || id == 0x0A2E || id == 0x0C02 || id == 0x0C12 || id == 0x0C22 || id == 0x0C06 || id == 0x0C16 ||
    id == 0x0C26 || id == 0x0C0C || id == 0x0C1C || id == 0x0C2C || id == 0x0C0B || id == 0x0C1B || id == 0x0C2B ||
    id == 0x0C0E || id == 0x0C1E || id == 0x0C2E || id == 0x0D02 || id == 0x0D12 || id == 0x0D22 || id == 0x0D06 ||
    id == 0x0D16 || id == 0x0D26 || id == 0x0D0A || id == 0x0D1A || id == 0x0D2A || id == 0x0D0B || id == 0x0D1B ||
    id == 0x0D2B || id == 0x0D0E || id == 0x0D1E || id == 0x0D2E ||
    // HD Graphics
    id == 0x0F30 || id == 0x0F31 || id == 0x0F32 || id == 0x0F33 || id == 0x0155 || id == 0x0157)
    return 7;

  // sixth generation
  if (id == 0x0102 || id == 0x0106 || id == 0x0112 || id == 0x0116 || id == 0x0122 || id == 0x0126 || id == 0x010A)
    return 6;

  // fifth generation
  if (id == 0x0042 || id == 0x0046)
    return 5;

  return 0;
}

bool WbSysInfo::isAmdLowEndGpu(QOpenGLFunctions *gl) {
  // https://pci-ids.ucw.cz/read/PC/1002
  quint32 id = gpuDeviceId(gl);
  if (
    // R3
    id == 0x9830 || id == 0x9836 || id == 0x9838 || id == 0x9850 || id == 0x9854 || id == 0x98e4 ||
    // R4
    id == 0x130b || id == 0x131b || id == 0x9851 ||
    // R5
    id == 0x130e || id == 0x1315 || id == 0x1316 || id == 0x1318 || id == 0x6607 || id == 0x6660 || id == 0x6664 ||
    id == 0x6665 || id == 0x6667 || id == 0x666f || id == 0x6771 || id == 0x6778 || id == 0x6779 || id == 0x68fa ||
    id == 0x6901 || id == 0x6907 || id == 0x9874 || id == 0xaa98 ||
    // R6
    id == 0x1309 || id == 0x130a || id == 0x130d || id == 0x131d || id == 0x9855 ||
    // R7
    id == 0x130c || id == 0x130f || id == 0x1313 || id == 0x131c || id == 0x6604 || id == 0x6605 || id == 0x6610 ||
    id == 0x6611 || id == 0x6613 || id == 0x6658 || id == 0x665c || id == 0x665d || id == 0x665f || id == 0x6810 ||
    id == 0x6819 || id == 0x682b || id == 0x683d || id == 0x683f || id == 0x6900 ||
    // R9
    id == 0x6646 || id == 0x6647 || id == 0x6798 || id == 0x679a || id == 0x67b0 || id == 0x67b1 || id == 0x67b9 ||
    id == 0x6820 || id == 0x6821 || id == 0x6823 || id == 0x6835 || id == 0x6921 || id == 0x6938 || id == 0x6939 ||
    id == 0x7300 || id == 0xaac8 || id == 0xaad8 || id == 0xaae8)
    return true;
  return false;
}

#else

bool WbSysInfo::isLowEndGpu() {
  static char lowEndGpu = -1;  // not yet determined
  if (lowEndGpu == -1) {       // based on the telemetry data from https://cyberbotics.com/telemetry
    lowEndGpu = 0;
    const QString &renderer = openGLRenderer();
    if (renderer.contains("Intel") && renderer.contains(" HD Graphics ")) {
      // we support only recent Intel GPUs from about 2015
      if (renderer.contains("Ivybridge") || renderer.contains("Sandybridge") || renderer.contains("Haswell") ||
          renderer.contains("Ironlake"))
        lowEndGpu = 1;
      else {
        const QRegularExpression re(" HD Graphics P{0,1}([\\d]{3,4})");
        const QRegularExpressionMatch match = re.match(renderer);
        const int number = match.hasMatch() ? match.captured(1).toInt() : 0;

        if ((number >= 2000 && number <= 6000) || (number >= 100 && number < 500))
          lowEndGpu = 1;
      }
    } else if (renderer.contains("Radeon HD") || renderer.contains("Radeon(TM) HD"))
      lowEndGpu = 1;  // We don't support old AMD Radeon HD cards
  }
  return (bool)lowEndGpu;
}

#endif
