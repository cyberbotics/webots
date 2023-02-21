#!/usr/bin/env python

# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Generate Windows Webots package."""

from generic_distro import WebotsPackage, print_error_message_and_exit
import ctypes
import datetime
import os
import subprocess


def convert_to_windows_path_separator(path):
    return path.replace('/', '\\')


def list_dependencies(package):
    return subprocess.check_output(['pactree', '-u', package]).decode().strip().split('\n')


class WindowsWebotsPackage(WebotsPackage):
    def __init__(self, package_name):
        super().__init__(package_name)
        self.application_file_path = self.application_name_lowercase_and_dashes + '.iss'
        self.msys64_files = []

    def create_webots_bundle(self, include_commit_file):
        super().create_webots_bundle(include_commit_file)

        self.add_folder_recursively(os.path.join(self.webots_home, 'msys64'))

        print('creating ISS descriptor')

        self.iss_script = open(self.application_file_path, 'w')
        self.iss_script.write(
          "[Setup]\n"
          "SourceDir=..\\..\n"
          "AppId=Webots\n"
          f"AppName={self.application_name}\n"
          f"AppVersion={self.full_version}\n"
          f"AppVerName={self.application_name} {self.full_version}\n"
          f"AppCopyright=Copyright (c) {datetime.date.today().year} Cyberbotics, Ltd.\n"
          "AppPublisher=Cyberbotics, Ltd.\n"
          "AppPublisherURL=https://www.cyberbotics.com\n"
          # tells Windows Explorer to reload environment variables (e.g., WEBOTS_HOME)
          "ChangesEnvironment=yes\n"
          "Compression=lzma2/fast\n"
          "DefaultDirName={autopf}\\" + self.application_name + "\n"
          "DefaultGroupName=Cyberbotics\n"
          "UninstallDisplayIcon={app}\\msys64\\mingw64\\bin\\webots-bin.exe\n"
          "PrivilegesRequired=admin\n"
          "UsePreviousPrivileges=no\n"
          "PrivilegesRequiredOverridesAllowed=dialog commandline\n"
          f"OutputBaseFileName={self.application_name_lowercase_and_dashes}-{self.package_version}_setup\n"
          f"OutputDir={convert_to_windows_path_separator(self.distribution_path)}\n"
          "ChangesAssociations=yes\n"
          "DisableStartupPrompt=yes\n"
          "ArchitecturesInstallIn64BitMode=x64\n"
          "ArchitecturesAllowed=x64\n"
          "UsePreviousAppDir=yes\n"
          "\n[Dirs]\n"
        )

        # add directories
        print('  adding folders')
        for dir in self.package_folders:
            self.make_dir(dir)
        self.copy_msys64_dependencies()

        # add files
        print('  adding files')
        self.iss_script.write('\n[Files]\n')
        for file in self.package_files:
            self.copy_file(file)
        self.copy_msys64_files()

        self.iss_script.write(
            "\n[Icons]\n"
            "Name: \"{app}\\" + self.application_name + "\"; Filename: \"{app}\\msys64\\mingw64\\bin\\webotsw.exe\"; "
            "WorkingDir: \"{app}\"; Comment: \"Robot simulator\"\n"
            "Name: \"{group}\\" + self.application_name + "\"; Filename: \"{app}\\msys64\\mingw64\\bin\\webotsw.exe\"; "
            "WorkingDir: \"{app}\"; Comment: \"Robot simulator\"\n"
            "Name: \"{userdesktop}\\" + self.application_name + "\"; Filename: \"{app}\\msys64\\mingw64\\bin\\webotsw.exe\"; "
            "WorkingDir: \"{app}""\"; Comment: \"Robot simulator\"\n"
            "Name: \"{group}\\Uninstall " + self.application_name + "\"; Filename: \"{uninstallexe}\"; WorkingDir: \"{app}\"; "
            "Comment: \"Uninstall " + self.application_name + "\"\n"
            "\n[Registry]\n"
            "Root: HKA; SubKey: \"Software\\Classes\\.wbt\"; ValueType: string; ValueData: \"webotsfile\"; "
            "Flags: uninsdeletekey\n"
            "Root: HKA; SubKey: \"Software\\Classes\\.wbt\"; ValueType: string; ValueName: \"Content Type\"; ValueData: "
            "\"application/webotsfile\"; Flags: uninsdeletekey\n"
            "Root: HKA; SubKey: \"Software\\Classes\\webotsfile\\DefaultIcon\"; ValueType: string; ValueData: "
            "\"{app}\\resources\\icons\\core\\webots_doc.ico\"; Flags: uninsdeletekey\n"
            "Root: HKA; SubKey: \"Software\\Classes\\webotsfile\\shell\\open\"; ValueType: string; ValueName: "
            "\"FriendlyAppName\"; ValueData: \"Webots\"; Flags: uninsdeletekey\n"
            "Root: HKA; SubKey: \"Software\\Classes\\webotsfile\\shell\\open\\command\"; ValueType: string; ValueData: "
            "\"\"\"{app}\\msys64\\mingw64\\bin\\webotsw.exe\"\" \"\"%1\"\"\"; Flags: uninsdeletekey\n"
            "Root: HKA; SubKey: \"Software\\Classes\\Applications\\webotsw.exe\"; ValueType: string; "
            "ValueName: \"SupportedTypes\"; ValueData: \".wbt\"; Flags: uninsdeletekey\n"
            "Root: HKA; SubKey: \"Software\\Classes\\Applications\\webotsw.exe\"; ValueType: string; "
            "ValueName: \"FriendlyAppName\"; ValueData: \"Webots\"; Flags: uninsdeletekey\n"
            "Root: HKCU; SubKey: \"Software\\Cyberbotics\"; Flags: uninsdeletekeyifempty dontcreatekey\n"
            f"Root: HKCU; SubKey: \"Software\\Cyberbotics\\{self.application_name} {self.full_version}\"; "
            "Flags: uninsdeletekey dontcreatekey\n"
            "Root: HKA; SubKey: \"SYSTEM\\CurrentControlSet\\Control\\Session Manager\\Environment\"; ValueType: string; "
            "ValueName: \"WEBOTS_HOME\"; ValueData: \"{app}\"; Flags: preservestringtype\n"
            "Root: HKA; SubKey: \"Software\\Classes\\webotsfile\"; "
            "Flags: uninsdeletekey dontcreatekey\n"
            # On some systems (as already reported by two Chinese users), some unknown third party software badly installs a
            # zlib1.dll and libeay32.dll in the C:\Windows\System32 folder.
            # Similarly, libjpeg-8.dll may be found there.
            # This is a very bad practise as such DLLs conflicts with the same DLLs provided in the msys64 folder of Webots.
            # So, we will delete any of these libraries from the C:\Windows\System32 folder before installing Webots.
            "\n[InstallDelete]\n"
            "Type: files; Name: \"{sys}\\zlib1.dll\"\n"
            "Type: files; Name: \"{sys}\\libeay32.dll\"\n"
            "Type: files; Name: \"{sys}\\libjpeg-8.dll\"\n"
            "\n[Code]\n"
            "function InitializeSetup(): Boolean;\n"
            "var\n"
            "  ResultCode: Integer;\n"
            "  Uninstall: String;\n"
            "begin\n"
            "  if isAdmin and RegQueryStringValue(HKLM, 'Software\\Microsoft\\Windows\\CurrentVersion\\Uninstall\\"
            "Webots_is1', 'UninstallString', Uninstall) then begin\n"
            "    if MsgBox('A version of Webots is already installed for all users on this computer. "
            "It will be removed and replaced by the version you are installing.', mbInformation, MB_OKCANCEL) = IDOK "
            "then begin\n"
            "      Exec(RemoveQuotes(Uninstall), ' /SILENT', '', SW_SHOWNORMAL, ewWaitUntilTerminated, ResultCode);\n"
            "      Result := TRUE;\n"
            "    end else begin\n"
            "      Result := FALSE;\n"
            "    end;\n"
            "  end else begin\n"
            "    Result := TRUE;\n"
            "  end;\n"
            "  if RegQueryStringValue(HKCU, 'Software\\Microsoft\\Windows\\CurrentVersion\\Uninstall\\"
            "Webots_is1', 'UninstallString', Uninstall) then begin\n"
            "    if MsgBox('A version of Webots is already installed for the current user on this computer. It "
            "will be removed and replaced by the version you are installing.', mbInformation, MB_OKCANCEL) = IDOK "
            "then begin\n"
            "      Exec(RemoveQuotes(Uninstall), ' /SILENT', '', SW_SHOWNORMAL, ewWaitUntilTerminated, ResultCode);\n"
            "      Result := TRUE;\n"
            "    end else begin\n"
            "      Result := FALSE;\n"
            "    end;\n"
            "  end else begin\n"
            "    Result := TRUE;\n"
            "  end;\n"
            "  if RegQueryStringValue(HKLM32, 'Software\\Microsoft\\Windows\\CurrentVersion\\Uninstall\\Webots_is1', "
            "'UninstallString', Uninstall) then begin\n"
            "    if MsgBox('A version of Webots (32 bit) is already installed on this computer. It will be removed "
            "and replaced by the version (64 bit) you are installing.', mbInformation, MB_OKCANCEL) = IDOK then begin\n"
            "      Exec(RemoveQuotes(Uninstall), ' /SILENT', '', SW_SHOWNORMAL, ewWaitUntilTerminated, ResultCode);\n"
            "      Result := TRUE;\n"
            "    end else begin\n"
            "      Result := FALSE;\n"
            "    end;\n"
            "  end else begin\n"
            "    Result := TRUE;\n"
            "  end;\n"
            "end;\n\n"
            "procedure CurUninstallStepChanged(CurUninstallStep: TUninstallStep);\n"
            "var\n"
            "  ResultCode: Integer;\n"
            "begin\n"
            "  if (CurUninstallStep = usPostUninstall) and DirExists(ExpandConstant('{app}')) then begin\n"
            "    if MsgBox(ExpandConstant('{app}') + ' was modified!'#13#10#13#10 +\n"
            "        'It seems you created or modified some files in this folder.'#13#10#13#10 +\n"
            "        'This is your last chance to do a backup of these files.'#13#10#13#10 +\n"
            "        'Do you want to delete the whole '+ ExpandConstant('{app}') +' folder now?'#13#10, mbConfirmation, "
            "MB_YESNO) = IDYES\n"
            "    then begin  // User clicked YES\n"
            "      // fix read-only status of all files and folders to be able to delete them\n"
            "      Exec('cmd.exe', '/c \"attrib -R ' + ExpandConstant('{app}') + '\\*.* /s /d\"', '', SW_HIDE, "
            "ewWaitUntilTerminated, ResultCode);\n"
            "      DelTree(ExpandConstant('{app}'), True, True, True);\n"
            "    end else begin  // User clicked NO\n"
            "      Abort;\n"
            "    end;\n"
            "  end;\n"
            "end;\n\n"
            "\n[Run]\n"
            "Filename: {app}\\msys64\\mingw64\\bin\\webotsw.exe; Description: \"Launch Webots\"; Flags: nowait postinstall "
            "skipifsilent\n"
            )

        self.iss_script.close()

        if 'INNO_SETUP_HOME' in os.environ:
            INNO_SETUP_HOME = os.getenv('INNO_SETUP_HOME')
        else:
            INNO_SETUP_HOME = "/C/Program Files (x86)/Inno Setup 6"
        print('creating webots_setup.exe (takes long)\n')
        subprocess.run([INNO_SETUP_HOME + '/iscc', '-Q', 'webots.iss'])

        print('Done.')

    def test_file(self, filename):
        if os.path.isabs(filename) or filename.startswith('$'):
            return   # ignore absolute file names
        if filename.find('*') != -1:
            return  # ignore wildcard filenames
        local_file_path = os.path.join('..', '..', filename)
        if not os.access(local_file_path, os.F_OK):
            print_error_message_and_exit(f"Missing file: {filename}")

    def make_dir(self, directory):
        win_directory = convert_to_windows_path_separator(directory)
        win_directory = win_directory.replace('\\\\', '\\')
        self.iss_script.write("Name: \"{app}\\" + win_directory + "\"\n")

    def copy_file(self, path):
        super().copy_file(path)

        dir_path = os.path.dirname(path)
        file_name = os.path.basename(path)
        file_details = os.path.splitext(file_name)
        file_extension = file_details[1]

        self.iss_script.write("Source: \"" + convert_to_windows_path_separator(path) + "\"; "
                              "DestDir: \"{app}\\" + convert_to_windows_path_separator(dir_path) + "\"")
        if file_name.startswith('.'):
            self.iss_script.write('; Attribs: hidden')
        if file_extension in ['.png', '.jpg']:
            self.iss_script.write('; Flags: nocompression')
        self.iss_script.write("\n")

    def compute_name_with_prefix_and_extension(self, basename, options):
        platform_independent = 'linux' not in options and 'windows' not in options and 'mac' not in options
        if platform_independent or 'windows' in options:
            if 'exe' in options:
                return basename + '.exe'
            if 'dll' in options:
                return basename + '.dll'
            return basename
        return ""

    def set_file_attribute(self, file, attribute):
        if attribute.lower() == 'hidden':
            flag = 0x02
        else:
            print_error_message_and_exit(f"Unknown file attribute: {attribute}")

        ret = ctypes.windll.kernel32.SetFileAttributesW(file, flag)
        if not ret:
            raise ctypes.WinError()

    def copy_msys64_dependencies(self):
        # list all the pacman dependencies needed by Webots, including sub-dependencies
        dependencies = list(set(  # use a set to make sure to avoid duplication
            list_dependencies('make') +
            list_dependencies('coreutils') +
            list_dependencies('mingw-w64-x86_64-gcc')
        ))

        # add specific folder dependencies needed by Webots
        folders = ['/tmp', '/mingw64', '/mingw64/bin', '/mingw64/bin/cpp',
                   '/mingw64/include',
                   '/mingw64/include/libssh',
                   '/mingw64/lib', '/mingw64/share',
                   '/mingw64/share/qt6', '/mingw64/share/qt6/plugins', '/mingw64/share/qt6/translations',
                   '/mingw64/share/qt6/plugins/imageformats', '/mingw64/share/qt6/plugins/platforms',
                   '/mingw64/share/qt6/plugins/tls', '/mingw64/share/qt6/plugins/styles']
        skip_paths = ['/usr/share/', '/mingw64/bin/zlib1.dll', '/mingw64/bin/libjpeg-8.dll']

        # add all the files and folders corresponding to the pacman dependencies
        for dependency in dependencies:
            print("  processing " + dependency, flush=True)
            for file in subprocess.check_output(['pacman', '-Qql', dependency]).decode().strip().split('\n'):
                skip = False
                for skip_path in skip_paths:
                    if file.startswith(skip_path):
                        skip = True
                        break
                if skip:
                    continue
                if not file.endswith('/'):
                    self.msys64_files.append(file)
                else:
                    folder = file.rstrip('/')
                    if folder not in folders:
                        folders.append(folder)

        for folder in folders:
            # remove initial '/' from folder otherwise absolute path cannot be joined
            self.make_dir(os.path.join('msys64', folder.lstrip('/')))

    def copy_msys64_files(self):
        # add the dependencies provided in the files_msys64.txt file
        root = subprocess.check_output(['cygpath', '-w', '/']).decode().strip().rstrip('\\')
        with open('files_msys64.txt', 'r') as file:
            for line in file:
                line = line.strip()
                if not line.startswith('#') and line:
                    if line in self.msys64_files:
                        print('  \033[1;31m' + line + ' is already included\033[0m')
                    else:
                        self.msys64_files.append(line)

        # automatically compute the dependencies of ffmpeg
        print("  processing ffmpeg dependencies (DLLs)", flush=True)
        for ffmpeg_dll in subprocess.check_output(['bash', 'ffmpeg_dependencies.sh'], shell=True).decode('utf-8').split():
            self.msys64_files.append('/mingw64/bin/' + ffmpeg_dll)

        # write every dependency file in the ISS file for files
        for file in self.msys64_files:
            file = file.replace('/', '\\')
            if file in ['\\mingw64\\bin\\libstdc++-6.dll',
                        '\\mingw64\\bin\\libgcc_s_seh-1.dll',
                        '\\mingw64\\bin\\libwinpthread-1.dll']:
                self.iss_script.write('Source: "' + root + file + '"; '
                                      'DestDir: "{app}\\msys64' + os.path.dirname(file) + '\\cpp"\n')
            else:
                self.iss_script.write('Source: "' + root + file + '"; DestDir: "{app}\\msys64' + os.path.dirname(file) + '"\n')
