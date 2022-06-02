import DefaultUrl from './DefaultUrl.js';

export default class Progress {
  constructor(parentNode, message, image) {
    this._progress = document.createElement('div');
    this._progress.id = 'progress';
    parentNode.appendChild(this._progress);

    // Progress image
    this._image = image ? image : 'https://cyberbotics.com/wwi/testingR2022b/images/loading.png';
    this._progressImage = document.createElement('img');
    this._progressImage.id = 'progress-image';
    this._progressImage.src = this._image;
    this._progress.appendChild(this._progressImage);

    // Webots version panel
    let progressPanel = document.createElement('div');
    progressPanel.className = 'progress-panel';
    this._progress.appendChild(progressPanel);

    let progressPanelContainer = document.createElement('div');
    progressPanelContainer.className = 'progress-panel-container';
    progressPanel.appendChild(progressPanelContainer);

    let progressPanelTitle = document.createElement('div');
    progressPanelTitle.className = "progress-panel-title";
    progressPanelTitle.innerHTML = '<img src="https://cyberbotics.com/assets/images/webots.png"></img><p>Webots</p>';
    progressPanelTitle.style.display = 'flex';
    progressPanelTitle.style.justifyContent = 'center';
    progressPanelContainer.appendChild(progressPanelTitle);

    let progressPanelSubitle = document.createElement('p');
    progressPanelSubitle.className = "progress-panel-subtitle";
    progressPanelSubitle.innerHTML = "Model. Program. Simulate. Transfer.";
    progressPanelContainer.appendChild(progressPanelSubitle);

    let progressPanelVersion = document.createElement('p');
    progressPanelVersion.className = "progress-panel-version";
    progressPanelVersion.innerHTML = "R2022b";
    progressPanelContainer.appendChild(progressPanelVersion);

    let progressPanelCopyright = document.createElement('p');
    progressPanelCopyright.className = "progress-panel-copyright";
    progressPanelCopyright.innerHTML = "Copyright &copy 1998 - 2022 Cyberbotcs Ltd.<br>Loading world...";
    progressPanelContainer.appendChild(progressPanelCopyright);

    // Progress Bar
    let progressBar = document.createElement('div');
    progressBar.id = 'progress-bar';
    this._progress.appendChild(progressBar);

    let progressLoadingGif = document.createElement('img');
    progressLoadingGif.src = DefaultUrl.wwiImagesUrl() + 'load_animation.gif';
    progressBar.appendChild(progressLoadingGif);

    this._progressBarMessage = document.createElement('div');
    this._progressBarMessage.id = "progress-bar-message";
    this._progressBarMessage.innerHTML = message;
    progressBar.appendChild(this._progressBarMessage);

    this._progressBarPercent = document.createElement('div');
    this._progressBarPercent.id = "progress-bar-percent";
    this._progressBarPercent.style.visibility = 'hidden';
    progressBar.appendChild(this._progressBarPercent);

    let progressBarPercentBackground = document.createElement('div');
    progressBarPercentBackground.id = "progress-bar-percent-background";
    this._progressBarPercent.appendChild(progressBarPercentBackground);

    this._progressBarPercentValue = document.createElement('div');
    this._progressBarPercentValue.id = "progress-bar-percent-value";
    this._progressBarPercentValue.style.width = 0;
    this._progressBarPercent.appendChild(this._progressBarPercentValue);

    this._progressBarInfo = document.createElement('div');
    this._progressBarInfo.id = "progress-bar-info";
    progressBar.appendChild(this._progressBarInfo);
  }

  setProgressBar(display, message, percent, info) {
    // Display style
    this._progress.style.display = display;
    if (display !== 'none') {
      // Message style and text
      if (typeof message !== 'undefined' && message !== 'same') {
        this._progressBarMessage.style.visibility = 'visible';
        this._progressBarMessage.innerHTML = message;
      } else if (message !== 'same')
        this._progressBarMessage.style.visibility = 'hidden';

      // Percentage bar value
      if (typeof percent !== 'undefined' && percent !== 'same' && percent !== 'hidden') {
        if (parseInt(this._progressBarPercentValue.style.width.slice(0, -1)) > percent)
          this._progressBarPercentValue.style.transition = 'none';
        else
          this._progressBarPercentValue.style.transition = '0.2s all ease-in-out';
        this._progressBarPercent.style.visibility = 'visible';
        if (percent >= 100) {
          this._progressBarPercentValue.style.width = '100%';
          this._progressBarPercentValue.style.borderTopRightRadius = '3px';
          this._progressBarPercentValue.style.borderBottomRightRadius = '3px';
        } else {
          this._progressBarPercentValue.style.width = percent.toString() + '%';
          this._progressBarPercentValue.style.borderTopRightRadius = '0';
          this._progressBarPercentValue.style.borderBottomRightRadius = '0';
        }
      } else if (message !== 'same') {
        this._progressBarPercentValue.style.width = '0';
        this._progressBarPercentValue.style.transition = 'none';
        this._progressBarPercent.style.visibility = 'hidden';
      }

      // Information style and text
      if (typeof info !== 'undefined' && info !== 'same') {
        if (info.length > 40)
          info = info.substring(0, 37) + '...';
        this._progressBarInfo.style.color = info.toLowerCase().includes("error") ? 'red' : '#007acc';
        this._progressBarPercentValue.style.backgroundColor = info.toLowerCase().includes("error") ? 'red' : '#405b75';
        this._progressBarInfo.style.visibility = 'visible';
        this._progressBarInfo.innerHTML = info;
      } else if (message !== 'same')
        this._progressBarInfo.style.visibility = 'hidden';
    }
  }

  setImage(imgUrl) {
    this._image = imgUrl;
    this._progressImage.src = imgUrl;
  }
}
