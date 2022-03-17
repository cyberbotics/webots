export default class FloatingRobotWindow {
    constructor(parentNode, name, window, url) {
        this.name = name;
        this.window = window;
        this.url = url;

        this.floatingRobotWindow = document.createElement('div');
        this.floatingRobotWindow.className = 'floating-robotWindow';
        this.floatingRobotWindow.id = name;
        this.floatingRobotWindow.style.visibility = 'hidden';
        parentNode.appendChild(this.floatingRobotWindow);
        
        this.floatingRobotWindowHeader = document.createElement('div');
        this.floatingRobotWindowHeader.className = 'floating-robotWindow-header';
        this.floatingRobotWindow.appendChild(this.floatingRobotWindowHeader);

        this.headerText = document.createElement('p');
        this.headerText.className = 'floating-robotWindow-text';
        this.headerText.innerHTML = 'Robot Window: ' + name;
        this.floatingRobotWindowHeader.appendChild(this.headerText);

        this.headerQuit = document.createElement('button');
        this.headerQuit.className = 'floating-robotWindow-close';
        this.headerQuit.id = 'close-'+name;
        this.headerQuit.innerHTML = ('&times');
        this.floatingRobotWindowHeader.appendChild(this.headerQuit);

        this.floatingRobotWindowContent = document.createElement('div');
        this.floatingRobotWindowContent.className = 'floating-robotWindow-content';
        this.floatingRobotWindow.appendChild(this.floatingRobotWindowContent);

        let robotWindow = document.createElement('iframe');
        robotWindow.id = this.name+"-robotWindow";
        robotWindow.src = this.url+"/robot_windows/"+this.window+"/"+this.window+".html?name="+this.name;
        this.floatingRobotWindowContent.appendChild(robotWindow);

        this.dragElement(document.getElementById(name));
    }

    getID() {
        return this.floatingRobotWindow.id;
    }

    setSize(w, h) {
        this.floatingRobotWindow.style.width = w.toString()+'px';
        this.floatingRobotWindow.style.height = h.toString()+'px';
    }

    setPosition(xPos, yPos) {
        this.floatingRobotWindow.style.left = xPos.toString()+'px';
        this.floatingRobotWindow.style.top = yPos.toString()+'px';
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

    dragElement(rw) {
        var pos1 = 0, pos2 = 0, pos3 = 0, pos4 = 0;
        var pageHeight = rw.parentNode.parentNode.parentNode.offsetHeight;
        var containerHeight = rw.parentNode.parentNode.offsetHeight;
        var containerWidth = rw.parentNode.parentNode.offsetWidth;
        var topOffset = 0, leftOffset = 0;

        rw.firstChild.onmousedown = dragMouseDown;
        rw.addEventListener('resize', function(event) {console.log("Risizing "+event.srcId);});

        function dragMouseDown(event) {
            rw.lastElementChild.style.pointerEvents = 'none';

            pageHeight = rw.parentNode.parentNode.parentNode.offsetHeight;
            containerHeight = rw.parentNode.parentNode.offsetHeight;
            containerWidth = rw.parentNode.parentNode.offsetWidth;

            event.preventDefault();

            pos1 = event.clientX;
            pos2 = event.clientY;
            topOffset = pos2 - rw.offsetTop
            leftOffset = pos1 - rw.offsetLeft;

            document.onmouseup = closeDragElement;
            document.onmousemove = robotWindowDrag;
        }

        function robotWindowDrag(event) {
            event.preventDefault();

            pos3 = pos1 - event.clientX;
            pos4 = pos2 - event.clientY;
            pos1 = event.clientX;
            pos2 = event.clientY;

            let top = rw.offsetTop - pos4;
            let left = rw.offsetLeft - pos3
            let bottom = top + rw.offsetHeight;
            let right = left + rw.offsetWidth;

            if (top<0 || event.clientY<topOffset)
                top = 0;
            else if (bottom>containerHeight || event.clientY>containerHeight-rw.offsetHeight+topOffset)
                top = containerHeight-rw.offsetHeight;
            
            if (left<0 || event.clientX<leftOffset)
                left = 0;
            else if (right>containerWidth || event.clientX>containerWidth-rw.offsetWidth+leftOffset)
                left = containerWidth-rw.offsetWidth;

            rw.style.top = top + "px";
            rw.style.left = left + "px";
        }

        function closeDragElement() {
            document.onmouseup = null;
            document.onmousemove = null;
            rw.lastElementChild.style.pointerEvents = 'auto';
        }
    }
}
  