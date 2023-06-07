import WbWorld from './nodes/WbWorld.js';

export default class InformationPanel {
  constructor(parentNode) {
    this.informationPanel = document.createElement('div');
    this.informationPanel.className = 'information-panel';

    const infoTabsBar = document.createElement('div');
    infoTabsBar.className = 'info-tabs-bar';
    this.informationPanel.appendChild(infoTabsBar);

    this.tab0 = document.createElement('button');
    this.tab0.className = 'info-tab tab0';
    this.tab0.innerHTML = 'Simulation';
    this.tab0.onclick = () => this.switchTab(0);
    infoTabsBar.appendChild(this.tab0);

    this.tab1 = document.createElement('button');
    this.tab1.className = 'info-tab tab1';
    this.tab1.innerHTML = 'Webots';
    this.tab1.onclick = () => this.switchTab(1);
    infoTabsBar.appendChild(this.tab1);

    this.webotsPresentation = document.createElement('div');
    this.webotsPresentation.style.display = 'none';
    const version = typeof WbWorld.instance !== 'undefined' ? WbWorld.instance.version : '';
    this.webotsPresentation.innerHTML = `
      <h2>Webots<span class=cloud>.cloud ` + version + `</span></h2>
      <img src=https://raw.githubusercontent.com/cyberbotics/webots/master/resources/icons/core/webots.png></img></br>
      <p>Open Source Robot Simulator</p>
      <a href="https://cyberbotics.com" target="_blank">Cyberbotics Ltd.</a>`;
    this.informationPanel.appendChild(this.webotsPresentation);

    this.simulationDescription = document.createElement('div');
    this.informationPanel.appendChild(this.simulationDescription);

    this.simulationTitle = document.createElement('h2');
    this.simulationTitle.innerHTML = 'No title found';
    this.simulationDescription.appendChild(this.simulationTitle);

    this.simulationText = document.createElement('div');
    this.simulationText.className = 'simulation-text';
    this.simulationText.innerHTML = 'No description found';
    this.simulationDescription.appendChild(this.simulationText);

    parentNode.appendChild(this.informationPanel);
  }

  switchTab(number) {
    if (number === 0) {
      this.tab0.style.backgroundColor = '#222';
      this.tab1.style.backgroundColor = '#333';
      this.webotsPresentation.style.display = 'none';
      this.simulationDescription.style.display = 'block';
    } else if (number === 1) {
      this.tab0.style.backgroundColor = '#333';
      this.tab1.style.backgroundColor = '#222';
      this.webotsPresentation.style.display = 'block';
      this.simulationDescription.style.display = 'none';
    }
  }

  setTitle(title) {
    this.simulationTitle.innerHTML = title;
  }

  setDescription(description) {
    let array = description.split('"').filter(String);
    let arrayLength = array.length;
    if (arrayLength > 0)
      this.simulationText.innerHTML = '';

    for (let i = 0; i < arrayLength; i++) {
      let regExUrl = /(http|https)\:\/\/[a-zA-Z0-9\-\.]+\.[a-zA-Z]*(\/\S*)?/g;
      array[i] = array[i].replaceAll(regExUrl, link => '<a href=' + link + ' target=_blank>' + link + '</a>');
      this.simulationText.innerHTML += '<p>' + array[i] + '</p>';
    }
  }
}
