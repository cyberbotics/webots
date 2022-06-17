export default class Progress {
  constructor(parentNode, message, image) {
    this._progress = document.createElement('div');
    this._progress.id = 'progress';
    this.parentNode = parentNode;
    this.parentNode.appendChild(this._progress);

    // Progress image
    this._progressImage = document.createElement('img');
    this._progressImage.id = 'progress-image';
    this._progressImage.src = (image || image !== 'undefined') ? image : this._setDefaultImage;
    this._progress.appendChild(this._progressImage);
    this._progressImage.addEventListener('error', this._setDefaultImage.bind(this));

    // Webots version panel
    this._progressPanel = document.createElement('div');
    this._progressPanel.className = 'progress-panel';
    this._progress.appendChild(this._progressPanel);

    this._progressPanelTitle = document.createElement('div');
    this._progressPanelTitle.className = 'progress-panel-title';
    this._progressPanelTitle.innerHTML = '<img src="https://cyberbotics.com/assets/images/webots.png"></img><p>Webots</p>';
    this._progressPanelTitle.style.display = 'flex';
    this._progressPanelTitle.style.justifyContent = 'center';
    this._progressPanel.appendChild(this._progressPanelTitle);

    this._progressPanelSubtitle = document.createElement('div');
    this._progressPanelSubtitle.className = 'progress-panel-subtitle';
    this._progressPanelSubtitle.innerHTML = 'Model. Program. Simulate. Transfer.';
    this._progressPanel.appendChild(this._progressPanelSubtitle);

    this._progressPanelVersion = document.createElement('div');
    this._progressPanelVersion.className = 'progress-panel-version';
    this._progressPanelVersion.innerHTML = 'R2022b';
    this._progressPanel.appendChild(this._progressPanelVersion);

    this._progressPanelCopyright = document.createElement('div');
    this._progressPanelCopyright.className = 'progress-panel-copyright';
    this._progressPanelCopyright.innerHTML = 'Copyright &copy 1998 - 2022 Cyberbotcs Ltd.';
    this._progressPanel.appendChild(this._progressPanelCopyright);

    // Progress Bar
    this._progressBar = document.createElement('div');
    this._progressBar.id = 'progress-bar';
    this._progressPanel.appendChild(this._progressBar);

    this._progressBarMessage = document.createElement('div');
    this._progressBarMessage.id = 'progress-bar-message';
    this._progressBarMessage.innerHTML = message;
    this._progressBar.appendChild(this._progressBarMessage);

    this._progressBarPercent = document.createElement('div');
    this._progressBarPercent.id = 'progress-bar-percent';
    this._progressBarPercent.style.visibility = 'hidden';
    this._progressBar.appendChild(this._progressBarPercent);

    let progressBarPercentBackground = document.createElement('div');
    progressBarPercentBackground.id = 'progress-bar-percent-background';
    this._progressBarPercent.appendChild(progressBarPercentBackground);

    this._progressBarPercentValue = document.createElement('div');
    this._progressBarPercentValue.id = 'progress-bar-percent-value';
    this._progressBarPercentValue.style.width = 0;
    this._progressBarPercent.appendChild(this._progressBarPercentValue);

    this._progressBarInfo = document.createElement('div');
    this._progressBarInfo.id = 'progress-bar-info';
    this._progressBar.appendChild(this._progressBarInfo);

    this._checkAndUpdateSize();
  }

  setProgressBar(display, message, percent, info) {
    if (display !== 'none') {
      // Check and update size

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

  _checkAndUpdateSize() {
    const resizeObserver = new ResizeObserver(() => {
      if (parseInt(this._progressPanel.offsetHeight) < 180) {
        this._progressPanel.style.height = '50%';
        this._progressPanelTitle.style.display = 'none';
        this._progressPanelSubtitle.style.display = 'none';
        this._progressPanelVersion.style.display = 'none';
        this._progressPanelCopyright.style.display = 'none';
        this._progressBar.style.top = 'calc(50% - 35px)';
      } else if (parseInt(this._progressPanel.offsetHeight) < 210) {
        this._progressPanel.style.height = '80%';
        this._progressPanelTitle.style.display = 'flex';
        this._progressPanelSubtitle.style.display = 'block';
        this._progressPanelVersion.style.display = 'none';
        this._progressPanelCopyright.style.display = 'none';
        this._progressBar.style.top = 'calc(85% - 70px)';
      } else if (parseInt(this._progressPanel.offsetHeight) < 300) {
        this._progressPanel.style.height = '80%';
        this._progressPanelTitle.style.display = 'flex';
        this._progressPanelSubtitle.style.display = 'block';
        this._progressPanelVersion.style.display = 'none';
        this._progressPanelCopyright.style.display = 'block';
        this._progressBar.style.top = '40%';
      } else {
        this._progressPanel.style.height = '80%';
        this._progressPanelTitle.style.display = 'flex';
        this._progressPanelSubtitle.style.display = 'block';
        this._progressPanelVersion.style.display = 'block';
        this._progressPanelCopyright.style.display = 'block';
        this._progressBar.style.top = '115px';
      }
    });
    resizeObserver.observe(document.getElementById('view3d'));
  }

  _setDefaultImage() {
    this._progressImage.src = 'https://cyberbotics.com/wwi/testingR2022b/images/loading/default_thumbnail.png';
  }
}
