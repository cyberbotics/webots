'use strict';

export default class HeaderView {
  constructor(element, designer) {
    this.headerElement = element;
    this.designer = designer;
    this.createTopBar();
  };

  createTopBar() {
    const menuItems = ['Export', 'Settings', 'Help'];

    const div = document.createElement('div');
    div.classList.add('menu');
    for (let i = 0; i < menuItems.length; ++i) {
      const button = document.createElement('button');
      button.classList.add('menu-button');
      button.innerHTML = menuItems[i];
      button.setAttribute('id', menuItems[i]);
      button.addEventListener('click', this.menuHandler.bind(this));
      div.appendChild(button);
    }
    this.headerElement.appendChild(div);
  };

  menuHandler(e) {
    console.log('Menu item pressed: ', e.target.id);
    switch (e.target.id) {
      case 'Export':
        this.export();
        break;
      case 'Settings':
        this.settings();
        break;
      case 'Help':
        this.help();
        break;
      default:
        throw new Error('Unknown menu button: ', e.target.id);
    }
  };

  export() {
    const data = this.designer.exportProto();
    const filename = 'CustomProto.proto';
    const mimeType = 'text/txt';
    let blob = new Blob([data], {type: mimeType});
    let e = document.createEvent('MouseEvents');
    let a = document.createElement('a');
    a.download = filename;
    a.href = window.URL.createObjectURL(blob);
    a.dataset.downloadurl = [mimeType, a.download, a.href].join(':');
    e.initMouseEvent('click', true, false, window, 0, 0, 0, 0, 0, false, false, false, false, 0, null);
    a.dispatchEvent(e);
  }
};
