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
    this.headerQuit.innerHTML = ('&times');
    this.floatingWindowHeader.appendChild(this.headerQuit);

    this.floatingWindowContent = document.createElement('div');
    this.floatingWindowContent.className = 'floating-window-content';
    this.floatingWindow.appendChild(this.floatingWindowContent);

    this.frame = document.createElement('iframe');
    this.frame.id = this.name + '-window';
    this.floatingWindowContent.appendChild(this.frame);

    this._dragElement(this.floatingWindow);
    this._maxSize(name);
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

  _dragElement(fw) {
    let pos1 = 0, pos2 = 0, pos3 = 0, pos4 = 0;
    let containerHeight = fw.parentNode.offsetHeight;
    let containerWidth = fw.parentNode.offsetWidth;
    let topOffset = 0, leftOffset = 0;

    fw.firstChild.onmousedown = dragMouseDown;

    function dragMouseDown(event) {
      fw.lastElementChild.style.pointerEvents = 'none';

      containerHeight = fw.parentNode.offsetHeight;
      containerWidth = fw.parentNode.offsetWidth;

      event.preventDefault();

      pos1 = event.clientX;
      pos2 = event.clientY;
      topOffset = pos2 - fw.offsetTop
      leftOffset = pos1 - fw.offsetLeft;

      document.onmouseup = closeDragElement;
      document.onmousemove = robotWindowDrag;
    }

    function robotWindowDrag(event) {
      event.preventDefault();

      pos3 = pos1 - event.clientX;
      pos4 = pos2 - event.clientY;
      pos1 = event.clientX;
      pos2 = event.clientY;

      let top = fw.offsetTop - pos4;
      let left = fw.offsetLeft - pos3
      let bottom = top + fw.offsetHeight;
      let right = left + fw.offsetWidth;

      if (top < 0 || event.clientY < topOffset)
        top = 0;
      else if (bottom>containerHeight || event.clientY>containerHeight-fw.offsetHeight + topOffset)
        top = containerHeight-fw.offsetHeight;
      
      if (left < 0 || event.clientX < leftOffset)
        left = 0;
      else if (right>containerWidth || event.clientX>containerWidth-fw.offsetWidth + leftOffset)
        left = containerWidth-fw.offsetWidth;

      fw.style.top = top + "px";
      fw.style.left = left + "px";
    }

    function closeDragElement() {
      document.onmouseup = null;
      document.onmousemove = null;
      fw.lastElementChild.style.pointerEvents = 'auto';
    }
  }

  _maxSize(name) {
    function setMaxHeight(name) {
      const fw = document.getElementById(name)
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

    document.getElementById(name).onmousedown = () => {setMaxHeight(name)};
    document.getElementById(name).onmouseup = () => {restorePointerEvents(name)};
  }
}
