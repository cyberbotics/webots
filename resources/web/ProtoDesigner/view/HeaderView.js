'use strict';

export default class HeaderView {
  constructor(element) {
    this.element = element;

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

    this.element.appendChild(div);
    console.log(this.element.innerHTML)
  };

  menuHandler(e) {
    console.log('menu pressed');
    document.getElementById('file-menu').classList.toggle('show');
  };
};

/*
<div class="dropdown">
  <button onclick="myFunction()" class="dropbtn">Dropdown</button>
  <div id="myDropdown" class="dropdown-content">
    <a href="#home">Home</a>
    <a href="#about">About</a>
    <a href="#contact">Contact</a>
  </div>
</div>
*/
