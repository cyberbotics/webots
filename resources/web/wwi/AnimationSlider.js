const template = document.createElement('template');

template.innerHTML = `
<link rel="stylesheet" href="https://cyberbotics.com/wwi/Wrenjs/css/animation_slider.css">

<div class="range" id="range">
  <div class="slider" id="slider">
    <div class="thumb" id="thumb"></div>
</div>
<span class="floating-time" id="floating-time">00:<small>00</small></span>
`;

// The name of this web component have to be "animation-slider"
export default class AnimationSlider extends HTMLElement {
  constructor() {
    super();
    this._shadowRoot = this.attachShadow({ mode: 'open' });
    this._shadowRoot.appendChild(template.content.cloneNode(true));

    this.shadowRoot.getElementById('range').addEventListener('mousedown', _ => this._mouseDown(_));
    document.addEventListener('mousemove', _ => this._mouseMove(_));
    document.addEventListener('mouseup', () => this._mouseUp());

    this.offset = 0; // use to center the floating time correctly
    this.isSelected = false;
  }

  _mouseDown(e) {
    let bounds = document.querySelector('animation-slider').shadowRoot.getElementById('range').getBoundingClientRect();
    let x = (e.clientX - bounds.left) / (bounds.right - bounds.left) * 100;
    document.querySelector('animation-slider').shadowRoot.getElementById('slider').style.width = x + '%';

    let event = new Event('slider_input', {
      bubbles: true,
      cancelable: true
    });

    event.detail = x;
    document.dispatchEvent(event);

    this.isSelected = true;
    document.querySelector('animation-slider').shadowRoot.getElementById('thumb').style.visibility = 'visible';
    document.querySelector('animation-slider').shadowRoot.getElementById('slider').style.height = '5px';
    document.querySelector('animation-slider').shadowRoot.getElementById('range').style.height = '5px';
  }

  _mouseUp() {
    if (this.isSelected) {
      let event = new Event('slider_input', {
        bubbles: true,
        cancelable: true
      });

      event.mouseup = true;
      document.dispatchEvent(event);

      this.isSelected = false;
      document.querySelector('animation-slider').shadowRoot.getElementById('thumb').style.visibility = '';
      document.querySelector('animation-slider').shadowRoot.getElementById('slider').style.height = '';
      document.querySelector('animation-slider').shadowRoot.getElementById('range').style.height = '';
      document.querySelector('animation-slider').shadowRoot.getElementById('floating-time').style.visibility = '';
    }
  }

  _mouseMove(e) {
    if (this.isSelected) {
      let bounds = document.querySelector('animation-slider').shadowRoot.getElementById('range').getBoundingClientRect();
      let x = (e.clientX - bounds.left) / (bounds.right - bounds.left) * 100;
      if (x > 100)
        x = 100;
      else if (x < 0)
        x = 0;

      document.querySelector('animation-slider').shadowRoot.getElementById('slider').style.width = x + '%';

      this.setFloatingTimePosition(e.clientX);

      let event = new Event('slider_input', {
        bubbles: true,
        cancelable: true
      });

      event.detail = x;
      document.dispatchEvent(event);
    }
  }

  value() {
    return document.querySelector('animation-slider').shadowRoot.getElementById('slider').style.width;
  }

  setValue(percentage) {
    document.querySelector('animation-slider').shadowRoot.getElementById('slider').style.width = percentage + '%';
  }

  setTime(time) {
    document.querySelector('animation-slider').shadowRoot.getElementById('floating-time').innerHTML = time;
    document.querySelector('animation-slider').shadowRoot.getElementById('floating-time').style.visibility = 'visible';
  }

  setFloatingTimePosition(position) {
    let bounds = document.querySelector('animation-slider').shadowRoot.getElementById('range').getBoundingClientRect();
    let x = (position - bounds.left);
    if (x - this.offset < bounds.left)
      x = bounds.left + this.offset;
    else if (x + this.offset + 20 > bounds.right)
      x = bounds.right - this.offset - 20;
    document.querySelector('animation-slider').shadowRoot.getElementById('floating-time').style.left = x - this.offset + 'px';
  }

  setOffset(offset) {
    this.offset = offset;
  }

  selected() {
    return this.isSelected;
  }
}
