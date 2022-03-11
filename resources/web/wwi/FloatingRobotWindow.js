export default class FloatingRobotWindow {
    constructor(parentNode, name, window, url) {
        this.name = name;
        this.window = window;
        this.url = url;

        this.floatingRobotWindow = document.createElement('div');
        this.floatingRobotWindow.className = 'floating-robotWindow';
        this.floatingRobotWindow.id = name;
        this.floatingRobotWindow.style.visibility = 'visible';
        this.floatingRobotWindow.style.height = '300px';
        this.floatingRobotWindow.style.width = '600px'
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

        //let robotWindow = document.createElement('iframe');
        //robotWindow.id = this.name+"-robotWindow";
        //robotWindow.src = this.url+"/robot_windows/"+this.window+"/"+this.window+".html?name="+this.name;
        //this.floatingRobotWindowContent.appendChild(robotWindow);

        let robotWindow = 

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

    dragElement(rw) {
        var pos1 = 0, pos2 = 0, pos3 = 0, pos4 = 0;

        rw.firstChild.onmousedown = dragMouseDown;

        function dragMouseDown(event) {
            event.preventDefault();
            pos1 = event.clientX;
            pos2 = event.clientY;
            document.onmouseup = closeDragElement;
            document.onmousemove = robotWindowDrag;
        }

        function robotWindowDrag(event) {
            event.preventDefault();
            pos3 = pos1 - event.clientX;
            pos4 = pos2 - event.clientY;
            pos1 = event.clientX;
            pos2 = event.clientY;
            rw.style.top = (rw.offsetTop - pos4) + "px";
            rw.style.left = (rw.offsetLeft - pos3) + "px";
        }

        function closeDragElement() {
            document.onmouseup = null;
            document.onmousemove = null;
        }
    }
}
  