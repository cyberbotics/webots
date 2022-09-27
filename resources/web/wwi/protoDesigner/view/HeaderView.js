'use strict';

export default class HeaderView {
  constructor(element, designer) {
    this.headerElement = element;
    this.designer = designer;
    this.createTopBar();

    this.setupNewProjectModalWindow();
    this.setupSettingsModalWindow();
    this.setupHelpModalWindow();
  };

  createTopBar() {
    const menuItems = ['New Project', 'Export', 'Help'];

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
      case 'New Project':
        this.newProject();
        break;
      case 'Export':
        this.export();
        break;
      case 'Settings':
        this.settings(e);
        break;
      case 'Help':
        this.help(e);
        break;
      default:
        throw new Error('Unknown menu button: ', e.target.id);
    }
  };

  setupNewProjectModalWindow() {
    let platforms = ['Tinkerbots', 'Thymio'];
    let assets = ['./view/assets/tinkerbots.png', './view/assets/thymio.png'];

    // create modal window
    let modal = document.getElementById('newProjectModal');
    modal.style.display = 'block';
    modal.classList.add('new-project-modal');

    for (let i = 0; i < platforms.length; ++i) {
      let figure = document.createElement('figure');
      figure.classList.add('new-project-platform');

      let img = document.createElement('img');
      img.classList.add('new-project-icon');
      img.setAttribute('draggable', false);
      img.setAttribute('id', platforms[i]);
      img.setAttribute('src', assets[i]);
      img.addEventListener('click', this.baseSelector.bind(this));

      let figcaption = document.createElement('figcaption');
      figcaption.classList.add('caption');
      figcaption.innerText = platforms[i];

      figure.appendChild(img);
      figure.appendChild(figcaption);
      modal.appendChild(figure);
    }

    this.headerElement.appendChild(modal);
  }

  setupHelpModalWindow() {
    let modal = document.getElementById('helpModal');
    modal.style.display = 'none';
  }

  setupSettingsModalWindow() {
    let modal = document.getElementById('settingsModal');
    modal.style.display = 'none';
  }

  newProject() {
    // clean up
    this.designer.cleanup();
    // show possible Robot bases
    let modal = document.getElementById('newProjectModal');
    modal.style.display = 'block';
  }

  settings(e) {
    let modal = document.getElementById('settingsModal');

    if (typeof modal === 'undefined')
      throw new Error('Error, settingsModal component is missing.');

    if (modal.style.display === 'block')
      modal.style.display = 'none';
    else {
      const position = e.target.getBoundingClientRect();

      let backgrounds = ['blue', 'mountains'];
      let assets = ['./view/assets/background-blue.png', './view/assets/background-blue.png'];

      // get the modal
      modal.style.top = '25px';
      modal.style.left = position.x + 'px';

      for (let i = 0; i < backgrounds.length; ++i) {
        let figure = document.createElement('figure');
        figure.classList.add('settings-backgrounds');

        let img = document.createElement('img');
        img.classList.add('settings-icon');
        img.setAttribute('draggable', false);
        img.setAttribute('id', backgrounds[i]);
        img.setAttribute('src', assets[i]);
        img.addEventListener('click', this.baseSelector.bind(this));

        figure.appendChild(img);
        modal.appendChild(figure);
      }

      modal.style.display = 'block';
    }
  }

  help(e) {
    let modal = document.getElementById('helpModal');

    if (typeof modal === 'undefined')
      throw new Error('Error, helpModal component is missing.');

    if (modal.style.display === 'block')
      modal.style.display = 'none';
    else {
      const position = e.target.getBoundingClientRect();
      // get the modal
      modal.style.top = '25px';
      modal.style.left = position.x + 'px';

      modal.style.display = 'block';
    }
  }

  baseSelector(e) {
    if (typeof this.designer.scene === 'undefined')
      return; // minimal scene not ready yet

    console.log('Selected ' + e.target.id + ' as base proto');
    if (e.target.id === 'Thymio') {
      alert('Platform not yet supported.');
      return;
    }

    let modal = document.getElementById('newProjectModal');
    modal.style.display = 'none';

    // const url = '../wwi/Protos/ProtoTestParameters.proto';
    // const url = '../wwi/Protos/ProtoBox.proto';
    // const url = '../wwi/Protos/ProtoTemplate.proto';
    // const url = '../wwi/Protos/ProtoTransform.proto';
    // const url = '../wwi/Protos/ProtoTestAll.proto';
    // const url = '../wwi/Protos/ProtoDefUse.proto';

    // base geometries
    // const url = '../wwi/Protos/ProtoTestBox.proto';
    // const url = '../wwi/Protos/ProtoTestCylinder.proto';
    // const url = '../wwi/Protos/ProtoTestSphere.proto';
    // const url = '../wwi/Protos/ProtoTestCapsule.proto';
    // const url = '../wwi/Protos/ProtoTestCone.proto';

    // const url = '../wwi/Protos/ProtoTestSFNode.proto';
    // const url = './examples/protos/ProtoSolid.proto';
    // const url = './examples/protos/ProtoIndexedFaceSet.proto';
    // const url = './examples/protos/ProtoSlot.proto';
    //const url = './examples/protos/ProtoSlotContainer.proto';
    // const url = './library/Tinkerbots/ProtoWithNested.proto';

    //const url = './library/Tinkerbots/TinkerbotsUniversal.proto';
    // Tinkerbots
// const url = './library/Tinkerbots/TinkerbotsAxle.proto';
     const url = './library/Tinkerbots/TinkerbotsBase.proto';
    // const url = './library/Tinkerbots/TinkerbotsBrickAdapter.proto';
    // const url = './library/Tinkerbots/TinkerbotsCube.proto';
    // const url = './library/Tinkerbots/TinkerbotsCubieBoxWithCrossSlots.proto';
// const url = './library/Tinkerbots/TinkerbotsCubieBoxWithRoundSlots.proto';
    // const url = './library/Tinkerbots/TinkerbotsCubieFemaleCube.proto';
    // const url = './library/Tinkerbots/TinkerbotsCubieMaleCube.proto';
    // const url = './library/Tinkerbots/TinkerbotsCubiePyramid.proto';
    // const url = './library/Tinkerbots/TinkerbotsCubieTriangle.proto';
// const url = './library/Tinkerbots/TinkerbotsDistanceSensor.proto';
    // const url = './library/Tinkerbots/TinkerbotsFinger.proto';
    // const url = './library/Tinkerbots/TinkerbotsGrabber.proto';
    // const url = './library/Tinkerbots/TinkerbotsLightSensor.proto';
    // const url = './library/Tinkerbots/TinkerbotsMotor.proto';
    // const url = './library/Tinkerbots/TinkerbotsPivot.proto';
    // const url = './library/Tinkerbots/TinkerbotsTwister.proto';
    // const url = './library/Tinkerbots/TinkerbotsWheel.proto';

    this.designer.loadProto(url);
  }

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
