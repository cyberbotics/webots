export default class FloatingRobotWindow {
    constructor(parentNode, name) {
        console.log("New Branch")
        this.floatingRobotWindow = document.createElement('div');
        this.floatingRobotWindow.className = 'floating-robotwindow';
        this.floatingRobotWindow.id = name;
        this.floatingRobotWindow.style.visibility = 'hidden';
        parentNode.appendChild(this.floatingRobotWindow);
        
        this.floatingRobotWindowHeader = document.createElement('div');
        this.floatingRobotWindowHeader.className = 'floating-robotwindow-header';
        this.floatingRobotWindowHeader.id = 'robotwindow-id';
        this.floatingRobotWindow.appendChild(this.floatingRobotWindowHeader);

        this.headerText = document.createElement('p');
        this.headerText.className = ('text-robotwindow');
        this.headerText.innerHTML = 'Robot Window: ' + name;
        this.floatingRobotWindowHeader.appendChild(this.headerText);

        let headerQuit = document.createElement('button');
        headerQuit.className = ('close-robotwindow');
        headerQuit.innerHTML = ('&times');
        headerQuit.addEventListener('mouseup', _ => this.changeVisibility());
        this.floatingRobotWindowHeader.appendChild(headerQuit);

        let floatingRobotWindowContent = document.createElement('div');
        floatingRobotWindowContent.className = 'floating-robotwindow-content';
        this.floatingRobotWindow.appendChild(floatingRobotWindowContent);

        let contentText = document.createElement('p');
        contentText.className = 'temp-class';
        contentText.innerHTML = 'Robot window goes in here <br> Lots of information <br> wow <br> Look at this robot window';
        floatingRobotWindowContent.appendChild(contentText);

        this.dragElement(document.getElementById(name));
    }

    getWidth() {
        return this.floatingRobotWindow.clientWidth;
    }

    setPosition(xPos, yPos) {
        this.floatingRobotWindow.style.top = yPos.toString()+'px';
        this.floatingRobotWindow.style.left = xPos.toString()+'px';
    }

    changeVisibility() {
        if (this.floatingRobotWindow.style.visibility == 'hidden') 
            this.floatingRobotWindow.style.visibility = 'visible';
        else
            this.floatingRobotWindow.style.visibility = 'hidden';
    }

    setVisibility(visibility) {
        this.floatingRobotWindow.style.visibility = visibility;
    }

    getVisibility() {
        return this.floatingRobotWindow.style.visibility;
    }

    dragElement(elmnt) {
        var pos1 = 0, pos2 = 0, pos3 = 0, pos4 = 0;

        elmnt.firstChild.onmousedown = dragMouseDown;

        function dragMouseDown(e) {
            e = e || window.event;
            e.preventDefault();
            pos3 = e.clientX;
            pos4 = e.clientY;
            document.onmouseup = closeDragElement;
            document.onmousemove = elementDrag;
        }

        function elementDrag(e) {
            e = e || window.event;
            e.preventDefault();
            pos1 = pos3 - e.clientX;
            pos2 = pos4 - e.clientY;
            pos3 = e.clientX;
            pos4 = e.clientY;

            elmnt.style.top = (elmnt.offsetTop - pos2) + "px";
            elmnt.style.left = (elmnt.offsetLeft - pos1) + "px";
        }

        function closeDragElement() {
            document.onmouseup = null;
            document.onmousemove = null;
        }
    }
}
  