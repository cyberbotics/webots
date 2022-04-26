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

    this._interact(this.floatingWindow);
  }

  _interact(fw) {
    interact('.floating-window')
      .draggable({
        listeners: {
          move(event) {
            let target = event.target;
            let transformMatrix = new DOMMatrixReadOnly(window.getComputedStyle(event.target).transform)
            let x = (transformMatrix.m41 || 0);
            let y = (transformMatrix.m42 || 0);

            target.lastElementChild.style.pointerEvents = 'none';
            target.style.userSelect = 'none';

            x += event.dx;
            y += event.dy;

            target.style.transform = 'translate(' + x + 'px,' + y + 'px)';
          },
          end(event) {
            event.target.lastElementChild.style.pointerEvents = 'auto';
            event.target.style.userSelect = 'auto';
          }
        },
        modifiers: [
          interact.modifiers.restrictRect({
            restriction: 'parent'
          })
        ]
      })
      .resizable( {
        edges: { left: true, right: true, bottom: true, top: true },
        listeners: {
          move(event) {
            let target = event.target;
            let transformMatrix = new DOMMatrixReadOnly(window.getComputedStyle(event.target).transform)
            let x = (transformMatrix.m41 || 0);
            let y = (transformMatrix.m42 || 0);

            target.lastElementChild.style.pointerEvents = 'none';
            target.style.userSelect = 'none';
            target.style.width = event.rect.width + 'px';
            target.style.height = event.rect.height + 'px';

            x += event.deltaRect.left;
            y += event.deltaRect.top;

            target.style.transform = 'translate(' + x + 'px,' + y + 'px)';
          },
          end(event) {
            event.target.lastElementChild.style.pointerEvents = 'auto';
            event.target.style.userSelect = 'auto';
          }
        },
        modifiers: [
          interact.modifiers.restrictEdges( {
            outer: 'parent'
          }),
    
          interact.modifiers.restrictSize( {
            min: { 
              width: parseInt(window.getComputedStyle(fw).minWidth),
              height: parseInt(window.getComputedStyle(fw).minHeight)
            }
          })
        ]
      })
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
}
