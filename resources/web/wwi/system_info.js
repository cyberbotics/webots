/* exported SystemInfo */

class SystemInfo {
  static isMacOS() {
    return navigator.platform.toUpperCase().indexOf('MAC') >= 0;
  }
}
