'use strict';

export default class HeaderView {
  constructor(element, designer) {
    this.headerElement = element;
    this.designer = designer;
    this.createMenu();
  };

  createMenu() {
    //this.element.innerHTML = '<p><i>' + text + '</i></p>';

    const menuItems = ['File', 'Help'];

    const div = document.createElement('div');
    div.className = 'dropdown';

    const button = document.createElement('button');
    button.innerHTML = 'File'
    button.className = 'dropdown-button';
    button.addEventListener('click', this.menuHandler.bind(this));

    const dropdownDiv = document.createElement('div');
    dropdownDiv.setAttribute('id', 'file-menu');
    dropdownDiv.className = 'dropdown-content';

    const a = document.createElement('a');
    const aText = document.createTextNode('Export');
    a.setAttribute('href', '#export');
    a.appendChild(aText);

    const b = document.createElement('a');
    const bText = document.createTextNode('Quit');
    b.setAttribute('href', '#quit');
    b.appendChild(bText);

    dropdownDiv.appendChild(a);
    dropdownDiv.appendChild(b);
    div.appendChild(button);
    div.appendChild(dropdownDiv);

    this.headerElement.appendChild(div);
    console.log(this.headerElement.innerHTML)
  };

  menuHandler(e) {
    console.log('menu pressed');
    document.getElementById('file-menu').classList.toggle('show');
    this.export();
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
