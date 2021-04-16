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
  background:black;
  cursor: pointer;
}

.slider {
  z-index: 3;
  background:red;
  height: 5px;
  width:50%;
  position: absolute;
}

.thumb {
  z-index: 2;
  background: blue;
  border-radius: 50%;
  width: 14px;
  height: 14px;
  position: absolute;
  bottom: -5px;
  right: -7px
}

</style>
<div class="range" id="range">
  <div class="slider" id="slider">  <div class="thumb" id="thumb"></div>
</div>
</div>
`;

class Slider extends HTMLElement {
  constructor() {
    super();
    this._shadowRoot = this.attachShadow({ mode: 'open' });
    this._shadowRoot.appendChild(template.content.cloneNode(true));

    /*let percent = 0;
    let timer = setInterval(() => {
      if (percent > 100)
        percent = 0

      document.querySelector('my-slider').shadowRoot.getElementById('slider').style.width = percent + '%';
      percent += 5;
    }, 1000);*/
    this.isSelected = false;
    document.querySelector('my-slider').shadowRoot.getElementById('thumb').addEventListener('mousedown', this._mouseDown);
    document.querySelector('my-slider').shadowRoot.getElementById('range').addEventListener('mousemove', this._mouseMove);
    document.querySelector('my-slider').shadowRoot.getElementById('thumb').addEventListener('mouseup', this._mouseUp);
  }

  _mouseDown() {
    console.log(this.isSelected)
    this.isSelected = true;
  }

  _mouseUp() {
    console.log("over")
    this.isSelected = false;
  }

  _mouseMove(e) {
    if (this.isSelected) {
      let pos = e.pageX / window.innerWidth * 100 + '%';
      document.querySelector('my-slider').shadowRoot.getElementById('slider').style.width = pos;
      console.log(e.pageX / window.innerWidth * 100 + '%');
    }
  }
}

window.customElements.define('my-slider', Slider);
