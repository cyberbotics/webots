const template = document.createElement('template');

template.innerHTML = `
<style>
.range {
  z-index: 1;
  position: absolute;
  bottom: 41px;
  left: 1%;
  width: 98%;
  height:5px;
  background:rgba(140,140,140,0.5);
  cursor: pointer;
}

.slider {
  z-index: 3;
  background:red;
  height: 5px;
  width:50%;
  position: absolute;
  opacity:1;
}

.thumb {
  z-index: 2;
  background: red;
  border-radius: 50%;
  width: 14px;
  height: 14px;
  position: absolute;
  bottom: -5px;
  right: -7px;
  opacity:1;
}

</style>
<div class="range" id="range">
  <div class="slider" id="slider">
    <div class="thumb" id="thumb">
  </div>
</div>
`;

class Slider extends HTMLElement {
  constructor() {
    super();
    this._shadowRoot = this.attachShadow({ mode: 'open' });
    this._shadowRoot.appendChild(template.content.cloneNode(true));

    this.shadowRoot.getElementById('range').addEventListener('mousedown', this._mouseDown);
    document.addEventListener('mousemove', this._mouseMove);
    document.addEventListener('mouseup', this._mouseUp);
    this.shadowRoot.getElementById('range').addEventListener('click', this._onClick);
  }

  _onClick(e) {
    let bounds = document.querySelector('my-slider').shadowRoot.getElementById('range').getBoundingClientRect()
    let x = (e.clientX - bounds.left) / (bounds.right - bounds.left) * 100;
    document.querySelector('my-slider').shadowRoot.getElementById('slider').style.width = x + '%';
    //var event = document.createEvent('Event');
    //event.initEvent('slider_input', true, true);

    var event = new Event('slider_input', {
      bubbles: true,
      cancelable: true,
      detail: 5
    });

    document.dispatchEvent(event);
  }

  _mouseDown() {
    Slider.isSelected = true;
  }

  _mouseUp() {
    Slider.isSelected = false;
  }

  _mouseMove(e) {
    if (Slider.isSelected) {
      let bounds = document.querySelector('my-slider').shadowRoot.getElementById('range').getBoundingClientRect()
      let x = (e.clientX - bounds.left) / (bounds.right - bounds.left) * 100;
      if (x > 100)
        x = 100
      document.querySelector('my-slider').shadowRoot.getElementById('slider').style.width = x + '%';
    }
  }

  value() {
    return document.querySelector('my-slider').shadowRoot.getElementById('slider').style.width;
  }

  setValue(percentage) {
    document.querySelector('my-slider').shadowRoot.getElementById('slider').style.width = percentage + '%';
  }
}

window.customElements.define('my-slider', Slider);
