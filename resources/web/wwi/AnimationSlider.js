const template = document.createElement('template');

template.innerHTML = `
<link rel="stylesheet" href="https://cyberbotics.com/wwi/R2023b/css/animation_slider.css">

<div class="range" id="range">
  <div class="slider" id="slider">
    <div class="thumb" id="thumb"></div>
</div>
<span class="floating-time" id="floating-time">00:<small>00</small></span>
`;

// The name of this web component have to be "animation-slider"
export default class AnimationSlider extends HTMLElement {
  #isSelected;
  #offset;
  #shadowRoot;
  constructor() {
    super();
    this.#shadowRoot = this.attachShadow({ mode: 'open' });
    this.#shadowRoot.appendChild(template.content.cloneNode(true));

    this.#shadowRoot.getElementById('range').addEventListener('mousedown', _ => this.#mouseDown(_));
    this.#shadowRoot.getElementById('range').addEventListener('touchstart', _ => this.#mouseDown(_));
    document.addEventListener('mousemove', this.mousemoveRef = _ => this.#mouseMove(_));
    document.addEventListener('touchmove', this.mousemoveRef = _ => this.#mouseMove(_));
    document.addEventListener('mouseup', this.mouseupRef = () => this.#mouseUp());
    document.addEventListener('touchend', this.mouseupRef = () => this.#mouseUp());
    document.addEventListener('touchcancel', this.mouseupRef = () => this.#mouseUp());

    this.#offset = 0; // use to center the floating time correctly
    this.#isSelected = false;
    this.#shadowRoot.getElementById('slider').style.width = '0%';
  }

  #mouseDown(e) {
    e = e.touches ? e.touches[0] : e;
    const bounds = document.querySelector('animation-slider').shadowRoot.getElementById('range').getBoundingClientRect();
    const x = (e.clientX - bounds.left) / (bounds.right - bounds.left) * 100;
    document.querySelector('animation-slider').shadowRoot.getElementById('slider').style.width = x + '%';

    const event = new Event('sliderchange', {
      bubbles: true,
      cancelable: true
    });

    event.detail = x;
    document.dispatchEvent(event);

    this.#isSelected = true;
    document.querySelector('animation-slider').shadowRoot.getElementById('thumb').style.visibility = 'visible';
    document.querySelector('animation-slider').shadowRoot.getElementById('slider').style.height = '5px';
    document.querySelector('animation-slider').shadowRoot.getElementById('range').style.height = '5px';
  }

  #mouseUp() {
    if (this.#isSelected) {
      const event = new Event('sliderchange', {
        bubbles: true,
        cancelable: true
      });

      event.mouseup = true;
      document.dispatchEvent(event);

      this.#isSelected = false;
      document.querySelector('animation-slider').shadowRoot.getElementById('thumb').style.visibility = '';
      document.querySelector('animation-slider').shadowRoot.getElementById('slider').style.height = '';
      document.querySelector('animation-slider').shadowRoot.getElementById('range').style.height = '';
      document.querySelector('animation-slider').shadowRoot.getElementById('floating-time').style.visibility = '';
    }
  }

  #mouseMove(e) {
    e = e.touches ? e.touches[0] : e;
    if (this.#isSelected) {
      const bounds = document.querySelector('animation-slider').shadowRoot.getElementById('range').getBoundingClientRect();
      let x = (e.clientX - bounds.left) / (bounds.right - bounds.left) * 100;
      if (x > 100)
        x = 100;
      else if (x < 0)
        x = 0;

      document.querySelector('animation-slider').shadowRoot.getElementById('slider').style.width = x + '%';

      this.setFloatingTimePosition(e.clientX);

      const event = new Event('sliderchange', {
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
    const bounds = document.querySelector('animation-slider').shadowRoot.getElementById('range').getBoundingClientRect();
    let x = position - bounds.left;
    if (x - this.#offset < 0)
      x = this.#offset;
    else if (position + this.#offset + 20 > bounds.right)
      x = bounds.right - bounds.left - this.#offset - 20;

    document.querySelector('animation-slider').shadowRoot.getElementById('floating-time').style.left = x - this.#offset + 'px';
  }

  setOffset(offset) {
    this.#offset = offset;
  }

  selected() {
    return this.#isSelected;
  }

  removeEventListeners() {
    document.removeEventListener('mousemove', this.mousemoveRef);
    this.mousemoveRef = undefined;
    document.removeEventListener('mouseup', this.mouseupRef);
    this.mouseupRef = undefined;
  }
}
