export default class SystemInfo {
  static isMacOS() {
    let platform = navigator?.userAgentData?.platform || navigator?.platform || 'unknown';
    return platform.toUpperCase().indexOf('MAC') >= 0;
  }

  static isIOS() {
    let platform = navigator?.userAgentData?.platform || navigator?.platform || 'unknown';
    return !!navigator.platform && /iPad|iPhone|iPod/.test(platform);
  }

  static isMobileDevice() {
    // https://stackoverflow.com/questions/11381673/detecting-a-mobile-browser
    return /Android|webOS|iPhone|iPad|iPod|BlackBerry|IEMobile|Opera Mini/i.test(navigator.userAgent);
  }

  static isSafari() {
    // https://stackoverflow.com/questions/9847580/how-to-detect-safari-chrome-ie-firefox-and-opera-browser
    return /constructor/i.test(window.HTMLElement) ||
      (function(p) { return p.toString() === '[object SafariRemoteNotification]'; })(!window['safari'] ||
        (typeof safari !== 'undefined' && window['safari'].pushNotification));
  }
}
