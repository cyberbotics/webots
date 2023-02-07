export default class FloatingWindow {
  constructor(parentNode, name, url) {
    this.parentNode = parentNode;
    this.name = name;
    this.url = url;

    this.floatingWindow = document.createElement('div');
    this.floatingWindow.className = 'floating-window';
    this.floatingWindow.id = name;
    this.floatingWindow.style.visibility = 'hidden';
    this.floatingWindow.style.zIndex = '3';
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

    this.#interactElement(this.floatingWindow);

    this.floatingWindow.addEventListener('mousedown', this.bringToFront.bind(this));
  }

  bringToFront() {
    document.querySelectorAll('.floating-window').forEach((window) => {
      window.style.zIndex = '3';
    });
    this.floatingWindow.style.zIndex = '4';
  }

  getId() {
    return this.floatingWindow.id;
  }

  setSize(w, h) {
    this.floatingWindow.style.width = w.toString() + 'px';
    this.floatingWindow.style.height = h.toString() + 'px';
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

  #interactElement(fw) {
    let posX, dX, top, height, maxTop, maxHeight, containerHeight, topOffset, bottomOffset;
    let posY, dY, left, width, maxLeft, maxWidth, containerWidth, leftOffset, rightOffset;
    let interactionType, direction, parentOffsetX, parentOffsetY;
    const minWidth = parseInt(window.getComputedStyle(fw).getPropertyValue('min-width'));
    const minHeight = parseInt(window.getComputedStyle(fw).getPropertyValue('min-height'));

    fw.firstChild.onmousedown = interactMouseDown;
    fw.firstChild.ontouchstart = interactMouseDown;
    for (let n of fw.childNodes[1].childNodes) {
      n.onmousedown = interactMouseDown;
      n.ontouchstart = interactMouseDown;
    }

    function interactMouseDown(event) {
      fw.style.userSelect = 'none';

      let e = event.touches ? event.touches[0] : event;
      containerHeight = fw.parentNode.offsetHeight;
      containerWidth = fw.parentNode.offsetWidth;
      maxHeight = fw.offsetTop + fw.offsetHeight;
      maxWidth = fw.offsetLeft + fw.offsetWidth;
      maxTop = maxHeight - minHeight;
      maxLeft = maxWidth - minWidth;
      const boundingRectangle = document.getElementsByClassName('webots-view')[0].getBoundingClientRect();
      parentOffsetX = boundingRectangle.left;
      parentOffsetY = boundingRectangle.top;
      posX = e.clientX - parentOffsetX;
      posY = e.clientY - parentOffsetY;

      topOffset = posY - fw.offsetTop;
      bottomOffset = posY + containerHeight - fw.offsetTop - fw.offsetHeight;
      leftOffset = posX - fw.offsetLeft;
      rightOffset = posX + containerWidth - fw.offsetLeft - fw.offsetWidth;
      direction = event.target.id.substring(7);
      interactionType = direction.length === 0 ? 'drag' : 'resize';

      document.onmouseup = closeInteractElement;
      document.onmousemove = floatingWindowInteract;
      document.ontouchend = closeInteractElement;
      document.ontouchcancel = closeInteractElement;
      document.ontouchmove = floatingWindowInteract;
    }

    function floatingWindowInteract(event) {
      document.querySelectorAll('.floating-window').forEach((floatingWindow) => {
        floatingWindow.style.pointerEvents = 'none';
      });

      top = fw.offsetTop;
      left = fw.offsetLeft;
      width = fw.offsetWidth;
      height = fw.offsetHeight;

      let e = event.touches ? event.touches[0] : event;
      dX = e.clientX - parentOffsetX - posX;
      dY = e.clientY - parentOffsetY - posY;
      posX = e.clientX - parentOffsetX;
      posY = e.clientY - parentOffsetY;

      if (interactionType === 'resize') {
        // Resize element
        document.body.style.cursor = direction + '-resize';
        if (direction.includes('n')) {
          if (top + dY < 0 || posY < topOffset) { // out of bounds
            top = 0;
            height = maxHeight;
          } else if (top + dY > maxTop || posY - topOffset > maxTop) { // min height
            height = minHeight;
            top = maxTop;
          } else if (height > minHeight || dY < 0) { // resize
            height -= dY;
            top += dY;
          }
        }
        if (direction.includes('w')) {
          if (left + dX < 0 || posX < leftOffset) { // out of bounds
            left = 0;
            width = maxWidth;
          } else if (left + dX > maxLeft || posX > maxLeft) { // min width
            width = minWidth;
            left = maxLeft;
          } else if (width > minWidth || dX < 0) { // resize
            width -= dX;
            left += dX;
          }
        }
        if (direction.includes('s')) {
          if (top + fw.offsetHeight + dY > containerHeight || posY > bottomOffset) // out of bounds
            height = containerHeight - fw.offsetTop;
          else if (posY - bottomOffset + containerHeight < top + minHeight + dY) // min height
            height = minHeight;
          else // resize
            height += dY;
        }
        if (direction.includes('e')) {
          if (left + fw.offsetWidth + dX > containerWidth || posX > rightOffset) // out of bounds
            width = containerWidth - fw.offsetLeft;
          else if (posX - rightOffset + containerWidth < left + minWidth + dX) // min width
            width = minWidth;
          else // resize
            width += dX;
        }
      } else if (interactionType === 'drag') {
        // Drag element
        document.body.style.cursor = 'move';
        top = fw.offsetTop + dY;
        left = fw.offsetLeft + dX;
        if (top < 0 || posY < topOffset) // top boundary
          top = 0;
        else if (top + fw.offsetHeight > containerHeight || posY > containerHeight - fw.offsetHeight + topOffset)
          // bottom boundary
          top = containerHeight - fw.offsetHeight;
        if (left < 0 || posX < leftOffset) // left boundary
          left = 0;
        else if (left + fw.offsetWidth > containerWidth || posX > containerWidth - fw.offsetWidth + leftOffset)
          // right boundary
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
      document.ontouchstart = null;
      document.ontouchmove = null;
      document.querySelectorAll('.floating-window').forEach((floatingWindow) => {
        floatingWindow.style.pointerEvents = 'auto';
      });
      fw.style.userSelect = 'auto';
      document.body.style.cursor = 'default';
    }
  }
}
