export default class FloatingRobotWindow {
    constructor(parentNode, name, window, url) {
        this.floatingRobotWindow = document.createElement('div');
        this.floatingRobotWindow.className = 'floating-robotwindow';
        this.floatingRobotWindow.id = name;
        this.floatingRobotWindow.style.visibility = 'hidden';
        parentNode.appendChild(this.floatingRobotWindow);
        
        this.floatingRobotWindowHeader = document.createElement('div');
        this.floatingRobotWindowHeader.className = 'floating-robotwindow-header';
        this.floatingRobotWindow.appendChild(this.floatingRobotWindowHeader);

        this.headerText = document.createElement('p');
        this.headerText.className = 'floating-robotwindow-text';
        this.headerText.innerHTML = 'Robot Window: ' + name;
        this.floatingRobotWindowHeader.appendChild(this.headerText);

        this.headerQuit = document.createElement('button');
        this.headerQuit.className = 'floating-robotwindow-close';
        this.headerQuit.innerHTML = ('&times');
        this.floatingRobotWindowHeader.appendChild(this.headerQuit);

        let floatingRobotWindowContent = document.createElement('div');
        floatingRobotWindowContent.className = 'floating-robotwindow-content';
        this.floatingRobotWindow.appendChild(floatingRobotWindowContent);

        let robotWindow = document.createElement('iframe');
        robotWindow.className = name+"-robotwindow";
        robotWindow.src = url+"/robot_windows/"+window+"/"+window+".html?name="+name;
        floatingRobotWindowContent.appendChild(robotWindow);

        this.dragElement(document.getElementById(name));
    }

    getID() {
        return this.floatingRobotWindow.id;
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
  