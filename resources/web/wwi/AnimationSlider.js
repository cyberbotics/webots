const template = document.createElement('template');

template.innerHTML = `
<style>
.range {
  z-index: 1;
  position: absolute;
  bottom: 41px;
  left: 1%;
  width: 98%;
  height:3px;
  background:rgba(140,140,140,0.5);
  cursor: pointer;
  padding: 3px 0px;
  background-clip: content-box;
  user-select: none; /*fix selection bug in chrome*/
  -webkit-user-select: none;
}

.range:hover{
  height: 5px;
}

.range:hover .slider{
  height: 5px;
}

.range:hover .thumb{
  visibility: visible
}

.slider {
  z-index: 3;
  background:red;
  height: 3px;
  width:50%;
  position: absolute;
  opacity:1;
}

.thumb {
  visibility: hidden;
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

.floating-time {
  visibility: hidden;
  position: absolute;
  bottom: 15px;
  color:rgb(240, 240, 240);
  text-shadow: 1px 1px 1px rgb(104,104,104);
}

</style>
<div class="range" id="range">
  <div class="slider" id="slider">
    <div class="thumb" id="thumb"></div>
</div>
<span class="floating-time" id="floating-time">00:<small>00</small></span>
`;

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
    let bounds = document.querySelector('my-slider').shadowRoot.getElementById('range').getBoundingClientRect();
    let x = (e.clientX - bounds.left) / (bounds.right - bounds.left) * 100;
    document.querySelector('my-slider').shadowRoot.getElementById('slider').style.width = x + '%';

    let event = new Event('slider_input', {
      bubbles: true,
      cancelable: true
    });

    event.detail = x;
    document.dispatchEvent(event);

    this.isSelected = true;
    document.querySelector('my-slider').shadowRoot.getElementById('thumb').style.visibility = 'visible';
    document.querySelector('my-slider').shadowRoot.getElementById('slider').style.height = '5px';
    document.querySelector('my-slider').shadowRoot.getElementById('range').style.height = '5px';
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
      document.querySelector('my-slider').shadowRoot.getElementById('thumb').style.visibility = '';
      document.querySelector('my-slider').shadowRoot.getElementById('slider').style.height = '';
      document.querySelector('my-slider').shadowRoot.getElementById('range').style.height = '';
      document.querySelector('my-slider').shadowRoot.getElementById('floating-time').style.visibility = '';
    }
  }

  _mouseMove(e) {
    if (this.isSelected) {
      let bounds = document.querySelector('my-slider').shadowRoot.getElementById('range').getBoundingClientRect();
      let x = (e.clientX - bounds.left) / (bounds.right - bounds.left) * 100;
      if (x > 100)
        x = 100;
      else if (x < 0)
        x = 0;

      document.querySelector('my-slider').shadowRoot.getElementById('slider').style.width = x + '%';

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
    return document.querySelector('my-slider').shadowRoot.getElementById('slider').style.width;
  }

  setValue(percentage) {
    document.querySelector('my-slider').shadowRoot.getElementById('slider').style.width = percentage + '%';
  }

  setTime(time) {
    document.querySelector('my-slider').shadowRoot.getElementById('floating-time').innerHTML = time;
    document.querySelector('my-slider').shadowRoot.getElementById('floating-time').style.visibility = 'visible';
  }

  setFloatingTimePosition(position) {
    let bounds = document.querySelector('my-slider').shadowRoot.getElementById('range').getBoundingClientRect();
    let x = (position - bounds.left);
    if (x - this.offset < bounds.left)
      x = bounds.left + this.offset;
    else if (x + this.offset + 20 > bounds.right)
      x = bounds.right - this.offset - 20;
    document.querySelector('my-slider').shadowRoot.getElementById('floating-time').style.left = x - this.offset + 'px';
  }

  setOffset(offset) {
    this.offset = offset;
  }

  selected() {
    return this.isSelected;
  }
}
