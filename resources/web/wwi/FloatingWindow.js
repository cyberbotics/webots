export default class FloatingWindow {
  constructor(parentNode, name, url) {
    this.name = name;
    this.url = url;

    this.floatingWindow = document.createElement('div');
    this.floatingWindow.className = 'floating-window';
    this.floatingWindow.id = name;
    this.floatingWindow.style.visibility = 'hidden';
    parentNode.appendChild(this.floatingWindow);
    this.floatingWindowHeader = document.createElement('div');
    this.floatingWindowHeader.className = 'floating-window-header';
    this.floatingWindow.appendChild(this.floatingWindowHeader);

    this.headerText = document.createElement('p');
    this.headerText.className = 'floating-window-text';
    this.headerText.innerHTML = name;
    this.floatingWindowHeader.appendChild(this.headerText);

    this.headerQuit = document.createElement('button');
    this.headerQuit.className = 'floating-window-close';
    this.headerQuit.id = 'close-' + name;
    this.headerQuit.innerHTML = ('&times;');
    this.floatingWindowHeader.appendChild(this.headerQuit);

    const resizeContainer = document.createElement('div');
    resizeContainer.className = 'resize-container';
    this.floatingWindow.appendChild(resizeContainer);

    const directions = ['n', 'e', 's', 'w', 'ne', 'nw', 'se', 'sw'];
    for (const d of directions) {
      let resizeElement = document.createElement('div');
      resizeElement.className = 'resize-element';
      resizeElement.id = 'resize-' + d;
      resizeElement.style.cursor = d + '-resize';
      resizeContainer.appendChild(resizeElement);
    }

    this.floatingWindowContent = document.createElement('div');
    this.floatingWindowContent.className = 'floating-window-content';
    this.floatingWindow.appendChild(this.floatingWindowContent);

    this.frame = document.createElement('iframe');
    this.frame.id = this.name + '-window';
    this.floatingWindowContent.appendChild(this.frame);

    this._interactElement(this.floatingWindow);
    //this._maxSize(name);
  }

  getId() {
    return this.floatingWindow.id;
  }

  getSize() {
    return [this.floatingWindow.offsetWidth, this.floatingWindow.offsetHeight];
  }

  setSize(w, h) {
    this.floatingWindow.style.width = w.toString() + 'px';
    this.floatingWindow.style.height = h.toString() + 'px';
  }

  getPosition() {
    return [this.floatingWindow.offsetLeft, this.floatingWindow.offsetTop];
  }

  setPosition(xPos, yPos) {
    this.floatingWindow.style.left = xPos.toString() + 'px';
    this.floatingWindow.style.top = yPos.toString() + 'px';
  }

  changeVisibility() {
    if (this.floatingWindow.style.visibility === 'hidden')
      this.floatingWindow.style.visibility = 'visible';
    else
      this.floatingWindow.style.visibility = 'hidden';
  }

  setVisibility(visibility) {
    this.floatingWindow.style.visibility = visibility;
  }

  getVisibility() {
    return this.floatingWindow.style.visibility;
  }

  _interactElement(fw) {
    let posX, posY, dX, dY, id, top, bottom, left, right, width, height, interactionType;
    let containerHeight = fw.parentNode.offsetHeight;
    let containerWidth = fw.parentNode.offsetWidth;
    let topOffset = 0;
    let leftOffset = 0;
    let minWidth = parseInt(window.getComputedStyle(fw).getPropertyValue('min-width'));
    let minHeight = parseInt(window.getComputedStyle(fw).getPropertyValue('min-height'));
    const containerTop = fw.parentNode.parentNode.offsetTop;

    fw.firstChild.onmousedown = interactMouseDown;
    for (let n of fw.childNodes[1].childNodes)
      n.onmousedown = interactMouseDown;

    function interactMouseDown(event) {
      fw.lastElementChild.style.pointerEvents = 'none';

      containerHeight = fw.parentNode.offsetHeight;
      containerWidth = fw.parentNode.offsetWidth;

      event.preventDefault();

      posX = event.clientX;
      posY = event.clientY;
      topOffset = posY - fw.offsetTop;
      leftOffset = posX - fw.offsetLeft;
      id = event.target.id.substring(7);
      interactionType = id.length === 0 ? 'drag' : 'resize';

      document.onmouseup = closeInteractElement;
      document.onmousemove = floatingWindowInteract;
    }

    function floatingWindowInteract(event) {
      event.preventDefault();

      dX = event.clientX - posX;
      dY = event.clientY - posY;
      posX = event.clientX;
      posY = event.clientY;

      if (interactionType === 'resize') {
        document.body.style.cursor = id + '-resize';

        // Resize element
        if (id.length == 2 || id == 'e' || id == 'w') {
          if (id.includes('w')) {
            if (!(fw.offsetWidth === minWidth && dX > 0) && posX < right - minWidth) {
              width = fw.offsetWidth - dX;
              left = fw.offsetLeft + dX;
            }
          } else if (posX > left + minWidth)
            width = fw.offsetWidth + dX;
        }
        if (id.length === 2 || id === 'n' || id === 's') {
          if (id.includes('n')) {
            if (!(fw.offsetHeight === minHeight && dY > 0) && posY < bottom - containerTop + minHeight) {
              height = fw.offsetHeight - dY;
              top = fw.offsetTop + dY;
            }
          } else if (posY > top + containerTop + minHeight)
            height = fw.offsetHeight + dY;
        }
        bottom = top + fw.offsetHeight;
        right = left + fw.offsetWidth;

        // Check boundary conditions
        if (top < 0) {
          top = 0;
          height = fw.offsetHeight;
        } else if (bottom > containerHeight) {
          console.log("In here!");
          top = containerHeight - fw.offsetHeight;
          height = fw.offsetHeight;
        }
        if (left < 0) {
          left = 0;
          width = fw.offsetWidth;
        } else if (right > containerWidth || posX > containerWidth) {
          left = containerWidth - fw.offsetWidth;
          width = fw.offsetWidth;
        }
      } else if (interactionType === 'drag') {
        // Move element
        top = fw.offsetTop + dY;
        left = fw.offsetLeft + dX;
        bottom = top + fw.offsetHeight;
        right = left + fw.offsetWidth;
        width = fw.offsetWidth;
        height = fw.offsetHeight;

        // Check boundary conditions
        if (top < 0 || posY < topOffset)
          top = 0;
        else if (bottom > containerHeight || posY > containerHeight - fw.offsetHeight + topOffset)
          top = containerHeight - fw.offsetHeight;
  
        if (left < 0 || posX < leftOffset)
          left = 0;
        else if (right > containerWidth || posX > containerWidth - fw.offsetWidth + leftOffset)
          left = containerWidth - fw.offsetWidth;
      }

      fw.style.top = top + 'px';
      fw.style.left = left + 'px';
      fw.style.width = width + 'px';
      fw.style.height = height + 'px';
    }

    function closeInteractElement() {
      document.onmouseup = null;
      document.onmousemove = null;
      fw.lastElementChild.style.pointerEvents = 'auto';
      document.body.style.cursor = 'default';
    }
  }

  _maxSize(name) {
    function setMaxSize(name) {
      const fw = document.getElementById(name);
      const maxHeight = fw.parentNode.offsetHeight - fw.offsetTop;
      const maxWidth = fw.parentNode.offsetWidth - fw.offsetLeft;
      fw.style.maxHeight = maxHeight.toString() + 'px';
      fw.style.maxWidth = maxWidth.toString() + 'px';
      fw.lastElementChild.style.pointerEvents = 'none';
    }

    function restorePointerEvents(name) {
      const fw = document.getElementById(name);
      fw.lastElementChild.style.pointerEvents = 'auto';
    }

    document.getElementById(name).onmousedown = () => { setMaxSize(name); };
    document.getElementById(name).onmouseup = () => { restorePointerEvents(name); };
  }
}
