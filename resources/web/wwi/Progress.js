export default class Progress {
  constructor(parentNode, message, image) {
    this._progress = document.createElement('div');
    this._progress.id = 'progress';
    parentNode.appendChild(this._progress);

    // Progress image
    this._progressImage = document.createElement('img');
    this._progressImage.id = 'progress-image';
    this._progressImage.src = (image || image !== 'undefined') ? image : this._setDefaultImage;
    this._progress.appendChild(this._progressImage);
    this._progressImage.addEventListener('error', this._setDefaultImage.bind(this));

    // Webots version panel
    let progressPanel = document.createElement('div');
    progressPanel.className = 'progress-panel';
    this._progress.appendChild(progressPanel);

    let progressPanelTitle = document.createElement('div');
    progressPanelTitle.className = 'progress-panel-title';
    progressPanelTitle.innerHTML = '<img src="https://cyberbotics.com/assets/images/webots.png"></img><p>Webots</p>';
    progressPanelTitle.style.display = 'flex';
    progressPanelTitle.style.justifyContent = 'center';
    progressPanel.appendChild(progressPanelTitle);

    let progressPanelSubitle = document.createElement('div');
    progressPanelSubitle.className = 'progress-panel-subtitle';
    progressPanelSubitle.innerHTML = 'Model. Program. Simulate. Transfer.';
    progressPanel.appendChild(progressPanelSubitle);

    let progressPanelVersion = document.createElement('div');
    progressPanelVersion.className = 'progress-panel-version';
    progressPanelVersion.innerHTML = 'R2022b';
    progressPanel.appendChild(progressPanelVersion);

    let progressPanelCopyright = document.createElement('div');
    progressPanelCopyright.className = 'progress-panel-copyright';
    progressPanelCopyright.innerHTML = 'Copyright &copy 1998 - 2022 Cyberbotcs Ltd.';
    progressPanel.appendChild(progressPanelCopyright);

    // Progress Bar
    let progressBar = document.createElement('div');
    progressBar.id = 'progress-bar';
    progressPanel.appendChild(progressBar);

    this._progressBarMessage = document.createElement('div');
    this._progressBarMessage.id = 'progress-bar-message';
    this._progressBarMessage.innerHTML = message;
    progressBar.appendChild(this._progressBarMessage);

    this._progressBarPercent = document.createElement('div');
    this._progressBarPercent.id = 'progress-bar-percent';
    this._progressBarPercent.style.visibility = 'hidden';
    progressBar.appendChild(this._progressBarPercent);

    let progressBarPercentBackground = document.createElement('div');
    progressBarPercentBackground.id = 'progress-bar-percent-background';
    this._progressBarPercent.appendChild(progressBarPercentBackground);

    this._progressBarPercentValue = document.createElement('div');
    this._progressBarPercentValue.id = 'progress-bar-percent-value';
    this._progressBarPercentValue.style.width = 0;
    this._progressBarPercent.appendChild(this._progressBarPercentValue);

    this._progressBarInfo = document.createElement('div');
    this._progressBarInfo.id = 'progress-bar-info';
    progressBar.appendChild(this._progressBarInfo);
  }

  setProgressBar(display, message, percent, info) {
    if (display !== 'none') {
      // Message style and text
      if (typeof message !== 'undefined' && message !== 'same') {
        this._progressBarMessage.style.visibility = 'visible';
        this._progressBarMessage.innerHTML = message;
      } else if (message !== 'same')
        this._progressBarMessage.style.visibility = 'hidden';

      // Percentage bar value
      if (typeof percent !== 'undefined' && percent !== 'same' && percent !== 'hidden') {
        this._progressBarMessage.style.left = 0;
        this._progressBarMessage.style.right = 'auto';
        this._progressBarPercent.style.visibility = 'visible';
        this._progressBarPercentValue.style.transition = '0.2s all ease-in-out';
        if (parseInt(this._progressBarPercentValue.style.width.slice(0, -1)) < percent) {
          if (percent >= 100) {
            this._progressBarPercentValue.style.width = '100%';
            this._progressBarPercentValue.style.borderTopRightRadius = '4px';
            this._progressBarPercentValue.style.borderBottomRightRadius = '4px';
          } else {
            this._progressBarPercentValue.style.width = percent.toString() + '%';
            this._progressBarPercentValue.style.borderTopRightRadius = '0';
            this._progressBarPercentValue.style.borderBottomRightRadius = '0';
          }
        }
      } else if (message !== 'same') {
        this._progressBarMessage.style.left = 'auto';
        this._progressBarMessage.style.right = 0;
        this._progressBarPercentValue.style.transition = 'none';
        this._progressBarPercentValue.style.width = '0';
        this._progressBarPercent.style.visibility = 'hidden';
      }

      // Information style and text
      if (typeof info !== 'undefined' && info !== 'same') {
        this._progressBarInfo.style.color = info.toLowerCase().includes('error') ? 'red' : 'gray';
        this._progressBarInfo.style.visibility = 'visible';
        this._progressBarInfo.innerHTML = info;
      } else if (message !== 'same')
        this._progressBarInfo.style.visibility = 'hidden';
    } else {
      // Display none
      this._progressBarMessage.style.left = 'auto';
      this._progressBarMessage.style.right = 0;
      this._progressBarPercentValue.style.transition = 'none';
      this._progressBarPercentValue.style.width = '0';
      this._progressBarPercent.style.visibility = 'hidden';
    }

    // Display style
    this._progress.style.display = display;
  }

  _setDefaultImage() {
    this._progressImage.src = 'https://cyberbotics.com/wwi/testingR2022b/images/loading/default_thumbnail.png';
  }
}
