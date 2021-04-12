// Copyright 1996-2021 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

class SystemInfo {
  static isMacOS() {
    // https://stackoverflow.com/questions/10527983/best-way-to-detect-mac-os-x-or-windows-computers-with-javascript-or-jquery
    return navigator.platform.toUpperCase().indexOf('MAC') >= 0;
  }

  static isIOS() {
    // https://stackoverflow.com/questions/9038625/detect-if-device-is-ios
    return !!navigator.platform && /iPad|iPhone|iPod/.test(navigator.platform);
  }

  static isMobileDevice() {
    // https://stackoverflow.com/questions/11381673/detecting-a-mobile-browser
    return /Android|webOS|iPhone|iPad|iPod|BlackBerry|IEMobile|Opera Mini/i.test(navigator.userAgent);
  }
}

export {SystemInfo};
