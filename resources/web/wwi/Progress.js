export default class Progress {
  constructor(parentNode, wwiImgUrl, message) {
    this.progress = document.createElement('div');
    this.progress.id = 'webots-progress';
    parentNode.appendChild(this.progress);

    // Progress background image
    this.progressImage = document.createElement('img');
    this.progressImage.id = 'webots-progress-image';
    this.progressImage.src = 'https://cyberbotics.com/wwi/testingR2022b/images/loading/progress2022b.png'; // wwiImgUrl + 'loading/progress2022b.png';
    this.progress.appendChild(this.progressImage);

    // Webots version panel
    let webotsPanel = document.createElement('div');
    webotsPanel.className = 'webots-progress-panel';
    webotsPanel.style.background = 'linear-gradient(#222, #444)';
    webotsPanel.style.position = 'absolute';
    webotsPanel.style.width = '50%';
    webotsPanel.style.height = '100%';
    webotsPanel.style.top = '0';
    webotsPanel.style.left = '0';
    this.progress.appendChild(webotsPanel);

    // Progress Bar
    let progressContainer = document.createElement('div');
    progressContainer.id = 'webots-progress-container';
    this.progress.appendChild(progressContainer);

    let progressLoadingGif = document.createElement('img');
    progressLoadingGif.src = 'https://cyberbotics.com/wwi/testingR2022b/images/loading/load_animation.gif'; // wwiImgUrl + 'loading/load_animation.gif';
    progressContainer.appendChild(progressLoadingGif);

    this.progressMessage = document.createElement('div');
    this.progressMessage.id = "webots-progress-message";
    this.progressMessage.innerHTML = message;
    progressContainer.appendChild(this.progressMessage);

    let progressBar = document.createElement('div');
    progressBar.id = "webots-progress-bar";
    progressBar.style.visibility = 'hidden';
    progressContainer.appendChild(progressBar);

    let progressBarBackground = document.createElement('div');
    progressBarBackground.id = "webots-progress-bar-background";
    progressBar.appendChild(progressBarBackground);

    this.progressBarPercent = document.createElement('div');
    this.progressBarPercent.id = "webots-progress-bar-percent";
    this.progressBarPercent.style.width = 0;
    progressBar.appendChild(this.progressBarPercent);

    this.progressInfo = document.createElement('div');
    this.progressInfo.id = "webots-progress-info";
    progressContainer.appendChild(this.progressInfo);
  }

  setProgress(display, message, percent, info) {
    // Display style
    this.progress.style.display = display;
    if (display !== 'none') {
      // Message style and text
      if (typeof message !== 'undefined' && message !== 'same') {
        this.progressMessage.style.visibility = 'visible';
        this.progressMessage.innerHTML = message;
      } else if (message !== 'same')
        this.progressMessage.style.visibility = 'hidden';

      // Percentage bar value
      if (typeof percent !== 'undefined' && percent !== 'same' && percent !== 'hidden') {
        if (parseInt(this.progressBarPercent.style.width.slice(0, -1)) > percent)
          this.progressBarPercent.style.transition = 'none';
        else
          this.progressBarPercent.style.transition = '0.2s all ease-in-out';
        this.progressBar.style.visibility = 'visible';
        if (percent >= 100) {
          this.progressBarPercent.style.width = '100%';
          this.progressBarPercent.style.borderTopRightRadius = '3px';
          this.progressBarPercent.style.borderBottomRightRadius = '3px';
        } else {
          this.progressBarPercent.style.width = percent.toString() + '%';
          this.progressBarPercent.style.borderTopRightRadius = '0';
          this.progressBarPercent.style.borderBottomRightRadius = '0';
        }
      } else if (message !== 'same') {
        this.progressBarPercent.style.width = '0';
        this.progressBarPercent.style.transition = 'none';
        this.progressBar.style.visibility = 'hidden';
      }

      // Information style and text
      if (typeof info !== 'undefined' && info !== 'same') {
        if (info.length > 40)
          info = info.substring(0, 37) + '...';
        this.progressInfo.style.visibility = 'visible';
        this.progressInfo.innerHTML = info;
      } else if (message !== 'same')
        this.progressInfo.style.visibility = 'hidden';
    }
  }

  setImage(imgUrl) {
    this.progressImage.src = imgUrl;
  }
}
