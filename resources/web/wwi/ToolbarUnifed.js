import Animation from './Animation.js';
import AnimationSlider from './AnimationSlider.js';
import InformationPanel from './InformationPanel.js';
import WbWorld from './nodes/WbWorld.js';

export default class ToolbarUnifed {
  constructor(view, type, parentNode) {
    this._view = view;
    this.type = type;
    this._createToolbar(parentNode);
    if (type === 'animation')
      this._createSlider();
  }

  setType(type) {
    if (this.type !== type) {
      this.type = type;
      // TODO update toolbar accordingly
      return 0; // filler, remove when TODO is done;
    }
  }

  _createToolbar(parentNode) {
    console.log("create toolbar")
    this.toolbar = document.createElement('div');
    this.toolbar.id = 'toolbar';
    this.toolbar.className = 'toolbar';
    parentNode.appendChild(this.toolbar);

    // this.toolbar.addEventListener('mouseover', () => this._showPlayBar());
    // this.toolbar.addEventListener('mouseleave', _ => this._onMouseLeave(_));

    this.toolBarLeft = document.createElement('div');
    this.toolBarLeft.className = 'toolbar-left';
    this.toolBarLeft.id = 'toolbar-left';

    this.toolBarRight = document.createElement('div');
    this.toolBarRight.className = 'toolbar-right';
    this.toolBarRight.id = 'toolbar-right';

    this.toolbar.appendChild(this.toolBarLeft);
    this.toolbar.appendChild(this.toolBarRight);
    // this._view.mouseEvents.hidePlayBar = () => this._hidePlayBar();
    // this._view.mouseEvents.showPlayBar = () => this._showPlayBar();
  }

  _createInformation() {
    let div = document.createElement('div');
    let informationPanel = new InformationPanel(div);
    div.style.width = '100%';
    div.style.height = '100%';
    div.style.position = 'absolute';
    div.style.top = '0px';
    div.style.left = '0px';
    div.style.pointerEvents = 'none';
    let webotsView = document.getElementsByTagName('webots-view')[0];
    if (webotsView)
      webotsView.appendChild(div);

    if (typeof WbWorld.instance !== 'undefined' && WbWorld.instance.readyForUpdates) {
      informationPanel.setTitle(WbWorld.instance.title);
      informationPanel.setDescription(WbWorld.instance.description);
    }
  }

  _createSlider() {
    if (!Animation.sliderDefined) {
      window.customElements.define('animation-slider', AnimationSlider);
      Animation.sliderDefined = true;
    }
    this._timeSlider = document.createElement('animation-slider');
    this._timeSlider.id = 'time-slider';
    document.addEventListener('sliderchange', this.sliderchangeRef = _ => this._updateSlider(_));
    this.toolbar.appendChild(this._timeSlider);
    // this._timeSlider.shadowRoot.getElementById('range').addEventListener('mousemove', this.updateFloatingTimeRef = _ => this._updateFloatingTimePosition(_));
    // this._timeSlider.shadowRoot.getElementById('range').addEventListener('mouseleave', this.hideFloatingTimeRef = _ => this._hideFloatingTimePosition(_));
  }
}
