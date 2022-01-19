export default class informationPanel {
  constructor(parentNode) {
    this.informationPanel = document.createElement('div');
    this.informationPanel.className = 'information-panel';

    let infoTabsBar = document.createElement('div');
    infoTabsBar.className = 'info-tabs-bar';
    this.informationPanel.appendChild(infoTabsBar);

    this.tab0 = document.createElement('button');
    this.tab0.className = 'info-tab tab0';
    this.tab0.innerHTML = 'Simulation';
    this.tab0.onclick = () => this._switchTab(0);
    infoTabsBar.appendChild(this.tab0);

    this.tab1 = document.createElement('button');
    this.tab1.className = 'info-tab tab1';
    this.tab1.innerHTML = 'Webots';
    this.tab1.onclick = () => this._switchTab(1);
    infoTabsBar.appendChild(this.tab1);

    this.webotsPresentation = document.createElement('div');
    this.webotsPresentation.innerHTML = `
      <h2>Webots<span class=cloud>.cloud</span></h2>
      <img src=https://raw.githubusercontent.com/cyberbotics/webots/master/resources/icons/core/webots.png></img></br>
      <p>Open Source Robot Simulator</p>
      <a href="https://cyberbotics.com" target="_blank">Cyberbotics Ltd.</a>`;

    this.informationPanel.appendChild(this.webotsPresentation);
    parentNode.appendChild(this.informationPanel);
  }

  _switchTab(nbr) {
    console.log(nbr)
    if (nbr === 0) {
      this.tab0.style.backgroundColor = '#333';
      this.tab1.style.backgroundColor = '#555';
    } else if (nbr === 1) {
      this.tab0.style.backgroundColor = '#555';
      this.tab1.style.backgroundColor = '#333';
    }
  }
}
