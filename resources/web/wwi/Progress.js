export default class Progress {
  #progress;
  #progressImage;
  #progressBar;
  #progressBarInfo;
  #progressBarMessage;
  #progressBarPercent;
  #progressBarPercentValue;
  #progressPanel;
  #progressPanelCopyright;
  #progressPanelSubtitle;
  #progressPanelTitle;
  #progressPanelVersion;
  constructor(parentNode, message, image) {
    this.#progress = document.createElement('div');
    this.#progress.id = 'progress';
    this.parentNode = parentNode;
    this.parentNode.appendChild(this.#progress);

    // Progress image
    this.#progressImage = document.createElement('img');
    this.#progressImage.id = 'progress-image';
    this.#progressImage.src = (image && image !== 'undefined') ? image : this.#setDefaultImage;
    this.#progress.appendChild(this.#progressImage);
    this.#progressImage.addEventListener('error', this.#setDefaultImage.bind(this));

    // Webots version panel
    this.#progressPanel = document.createElement('div');
    this.#progressPanel.className = 'progress-panel';
    this.#progress.appendChild(this.#progressPanel);

    this.#progressPanelTitle = document.createElement('div');
    this.#progressPanelTitle.className = 'progress-panel-title';
    this.#progressPanelTitle.innerHTML = '<img src="https://cyberbotics.com/assets/images/webots.png"></img><p>Webots</p>';
    this.#progressPanelTitle.style.display = 'flex';
    this.#progressPanelTitle.style.justifyContent = 'center';
    this.#progressPanel.appendChild(this.#progressPanelTitle);

    this.#progressPanelSubtitle = document.createElement('div');
    this.#progressPanelSubtitle.className = 'progress-panel-subtitle';
    this.#progressPanelSubtitle.innerHTML = 'Model. Program. Simulate. Transfer.';
    this.#progressPanel.appendChild(this.#progressPanelSubtitle);

    this.#progressPanelVersion = document.createElement('div');
    this.#progressPanelVersion.className = 'progress-panel-version';
    this.#progressPanelVersion.innerHTML = 'R2023b';
    this.#progressPanel.appendChild(this.#progressPanelVersion);

    this.#progressPanelCopyright = document.createElement('div');
    this.#progressPanelCopyright.className = 'progress-panel-copyright';
    this.#progressPanelCopyright.innerHTML = 'Copyright &copy 1998 - 2023 Cyberbotcs Ltd.';
    this.#progressPanel.appendChild(this.#progressPanelCopyright);

    // Progress Bar
    this.#progressBar = document.createElement('div');
    this.#progressBar.id = 'progress-bar';
    this.#progressPanel.appendChild(this.#progressBar);

    this.#progressBarMessage = document.createElement('div');
    this.#progressBarMessage.id = 'progress-bar-message';
    this.#progressBarMessage.innerHTML = message;
    this.#progressBar.appendChild(this.#progressBarMessage);

    this.#progressBarPercent = document.createElement('div');
    this.#progressBarPercent.id = 'progress-bar-percent';
    this.#progressBarPercent.style.visibility = 'hidden';
    this.#progressBar.appendChild(this.#progressBarPercent);

    let progressBarPercentBackground = document.createElement('div');
    progressBarPercentBackground.id = 'progress-bar-percent-background';
    this.#progressBarPercent.appendChild(progressBarPercentBackground);

    this.#progressBarPercentValue = document.createElement('div');
    this.#progressBarPercentValue.id = 'progress-bar-percent-value';
    this.#progressBarPercentValue.style.width = 0;
    this.#progressBarPercent.appendChild(this.#progressBarPercentValue);

    this.#progressBarInfo = document.createElement('div');
    this.#progressBarInfo.id = 'progress-bar-info';
    this.#progressBar.appendChild(this.#progressBarInfo);

    this.#checkAndUpdateSize();
  }

  setProgressBar(display, message, percent, info) {
    if (display !== 'none') {
      // Message style and text
      if (typeof message !== 'undefined' && message !== 'same') {
        this.#progressBarMessage.style.visibility = 'visible';
        this.#progressBarMessage.innerHTML = message;
      } else if (message !== 'same')
        this.#progressBarMessage.style.visibility = 'hidden';

      // Percentage bar value
      if (typeof percent !== 'undefined' && percent !== 'same' && percent !== 'hidden') {
        this.#progressBarMessage.style.left = 0;
        this.#progressBarMessage.style.right = 'auto';
        this.#progressBarPercent.style.visibility = 'visible';
        this.#progressBarPercentValue.style.transition = '0.2s all ease-in-out';
        if (parseInt(this.#progressBarPercentValue.style.width.slice(0, -1)) < percent) {
          if (percent >= 100) {
            this.#progressBarPercentValue.style.width = '100%';
            this.#progressBarPercentValue.style.borderTopRightRadius = '4px';
            this.#progressBarPercentValue.style.borderBottomRightRadius = '4px';
          } else {
            this.#progressBarPercentValue.style.width = percent.toString() + '%';
            this.#progressBarPercentValue.style.borderTopRightRadius = '0';
            this.#progressBarPercentValue.style.borderBottomRightRadius = '0';
          }
        }
      } else if (message !== 'same') {
        this.#progressBarMessage.style.left = 'auto';
        this.#progressBarMessage.style.right = 0;
        this.#progressBarPercentValue.style.transition = 'none';
        this.#progressBarPercentValue.style.width = '0';
        this.#progressBarPercent.style.visibility = 'hidden';
      }

      // Information style and text
      if (typeof info !== 'undefined' && info !== 'same') {
        this.#progressBarInfo.style.color = info.toLowerCase().includes('warning') ? 'orange'
          : info.toLowerCase().includes('error') ? 'red' : 'gray';
        this.#progressBarInfo.style.visibility = 'visible';
        this.#progressBarInfo.innerHTML = info;
      } else if (message !== 'same')
        this.#progressBarInfo.style.visibility = 'hidden';
    } else {
      // Display none
      this.#progressBarMessage.style.left = 'auto';
      this.#progressBarMessage.style.right = 0;
      this.#progressBarPercentValue.style.transition = 'none';
      this.#progressBarPercentValue.style.width = '0';
      this.#progressBarPercent.style.visibility = 'hidden';
    }

    // Display style
    this.#progress.style.display = display;
  }

  #checkAndUpdateSize() {
    const view3d = document.getElementById('view3d');
    if (!view3d)
      return;
    const resizeObserver = new ResizeObserver(() => {
      if (parseInt(this.#progressPanel.offsetHeight) < 180) {
        this.#progressPanel.style.height = '50%';
        this.#progressPanelTitle.style.display = 'none';
        this.#progressPanelSubtitle.style.display = 'none';
        this.#progressPanelVersion.style.display = 'none';
        this.#progressPanelCopyright.style.display = 'none';
        this.#progressBar.style.top = 'calc(50% - 35px)';
      } else if (parseInt(this.#progressPanel.offsetHeight) < 210) {
        this.#progressPanel.style.height = '80%';
        this.#progressPanelTitle.style.display = 'flex';
        this.#progressPanelSubtitle.style.display = 'block';
        this.#progressPanelVersion.style.display = 'none';
        this.#progressPanelCopyright.style.display = 'none';
        this.#progressBar.style.top = 'calc(85% - 70px)';
      } else if (parseInt(this.#progressPanel.offsetHeight) < 300) {
        this.#progressPanel.style.height = '80%';
        this.#progressPanelTitle.style.display = 'flex';
        this.#progressPanelSubtitle.style.display = 'block';
        this.#progressPanelVersion.style.display = 'none';
        this.#progressPanelCopyright.style.display = 'block';
        this.#progressBar.style.top = '40%';
      } else {
        this.#progressPanel.style.height = '80%';
        this.#progressPanelTitle.style.display = 'flex';
        this.#progressPanelSubtitle.style.display = 'block';
        this.#progressPanelVersion.style.display = 'block';
        this.#progressPanelCopyright.style.display = 'block';
        this.#progressBar.style.top = '115px';
      }
    });
    resizeObserver.observe(view3d);
  }

  #setDefaultImage() {
    this.#progressImage.src = 'https://cyberbotics.com/wwi/R2023b/images/loading/default_thumbnail.png';
  }
}
