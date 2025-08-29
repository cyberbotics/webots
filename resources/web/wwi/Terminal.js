import FloatingWindow from './FloatingWindow.js';

export default class Terminal extends FloatingWindow {
  constructor(parentNode) {
    super(parentNode, 'terminal');
    this.floatingWindow.style.zIndex = '3';
    this.headerText.innerHTML = 'Terminal';
    this.floatingWindowContent.removeChild(this.frame);
    this.options = document.createElement('div');
    this.options.className = 'terminal-options';
    this.floatingWindowContent.appendChild(this.options);
    this.frame = document.createElement('div');
    this.frame.id = this.name + '-terminal';
    this.floatingWindowContent.appendChild(this.frame);
    this.ansiUp = new AnsiUp();
    this.textIDs = [];
    this.messagesStd = [];
    this.messagesTag = [];
    this.currentID = 0;

    parentNode.setWebotsMessageCallback(_ => this.createMessage(_));
    parentNode.setWebotsErrorMessageCallback(_ => this.createErrorMessage(_));

    this.#createOptions();
  }

  createMessage(message) {
    const newElement = document.createElement('p');
    newElement.style.color = 'white';
    this.messagesStd.push('stdout');
    if (!this.stdout)
      newElement.style.display = 'none';
    this.#addMessage(message, newElement);
  }

  createErrorMessage(message) {
    const newElement = document.createElement('p');
    newElement.style.color = 'red';
    this.messagesStd.push('stderr');
    if (!this.stderr)
      newElement.style.display = 'none';
    this.#addMessage(message, newElement);
  }

  clear() {
    this.textIDs = [];
    this.messagesStd = [];
    this.messagesTag = [];
    this.floatingWindowContent.removeChild(this.frame);
    this.frame = document.createElement('div');
    this.frame.id = this.name + '-terminal';
    this.floatingWindowContent.appendChild(this.frame);
  }

  #addMessage(message, element) {
    const html = this.ansiUp.ansi_to_html(message);
    if (html.startsWith('INFO:')) {
      element.style.color = 'cyan';
      this.messagesTag.push('info');
      if (!this.info)
        element.style.display = 'none';
    } else if (html.startsWith('WARNING:')) {
      this.messagesTag.push('warning');
      if (!this.warning)
        element.style.display = 'none';
    } else if (html.startsWith('ERROR:')) {
      this.messagesTag.push('error');
      if (!this.error)
        element.style.display = 'none';
    } else {
      this.messagesTag.push('other');
      if (!this.other)
        element.style.display = 'none';
    }

    element.className = 'terminal-message';
    element.id = 't' + this.currentID;
    element.innerHTML = html;

    this.textIDs.push('t' + this.currentID);
    this.currentID++;

    const terminal = this.frame;
    if (terminal) {
      terminal.appendChild(element);
      if (this.textIDs.length > 1000) {
        const id = this.textIDs.shift();
        this.messagesStd.shift();
        const nodeToRemove = document.getElementById(id);
        if (nodeToRemove)
          nodeToRemove.parentNode.removeChild(nodeToRemove);
      }
      terminal.scrollTop = terminal.scrollHeight;
    }
  }

  #createOptions() {
    const button = document.createElement('button');
    button.onclick = () => this.clear();
    button.className = 'icon-clear terminal-button';
    this.options.appendChild(button);

    let separator = document.createElement('div');
    separator.className = 'terminal-separator';
    separator.id = 'separator-1';
    this.options.appendChild(separator);

    this.stdoutButton = document.createElement('button');
    this.stdoutButton.onclick = () => this.#changeStdVisibility('stdout');
    this.stdoutButton.className = 'terminal-std terminal-button terminal-stdout';
    this.stdoutButton.innerHTML = 'stdout';
    this.stdout = true;
    this.options.appendChild(this.stdoutButton);

    this.stderrButton = document.createElement('button');
    this.stderrButton.onclick = () => this.#changeStdVisibility('stderr');
    this.stderrButton.className = 'terminal-std terminal-button terminal-stderr';
    this.stderrButton.innerHTML = 'stderr';
    this.stderr = true;
    this.options.appendChild(this.stderrButton);

    separator = document.createElement('div');
    separator.className = 'terminal-separator';
    separator.id = 'separator-2';
    this.options.appendChild(separator);

    this.infoButton = document.createElement('button');
    this.infoButton.onclick = () => this.#changeStdVisibility('info');
    this.infoButton.className = 'terminal-std terminal-button terminal-info';
    this.infoButton.innerHTML = 'INFO:';
    this.info = true;
    this.options.appendChild(this.infoButton);

    this.warningButton = document.createElement('button');
    this.warningButton.onclick = () => this.#changeStdVisibility('warning');
    this.warningButton.className = 'terminal-std terminal-button terminal-warning';
    this.warningButton.innerHTML = 'WARNING:';
    this.warning = true;
    this.options.appendChild(this.warningButton);

    this.errorButton = document.createElement('button');
    this.errorButton.onclick = () => this.#changeStdVisibility('error');
    this.errorButton.className = 'terminal-std terminal-button terminal-error';
    this.errorButton.innerHTML = 'ERROR:';
    this.error = true;
    this.options.appendChild(this.errorButton);

    this.otherButton = document.createElement('button');
    this.otherButton.onclick = () => this.#changeStdVisibility('other');
    this.otherButton.className = 'terminal-std terminal-button terminal-other';
    this.otherButton.innerHTML = 'other';
    this.other = true;
    this.options.appendChild(this.otherButton);
  }

  #changeStdVisibility(buttonName) {
    let enable;
    let button;
    if (buttonName === 'stdout') {
      this.stdout = !this.stdout;
      enable = this.stdout;
      button = this.stdoutButton;
    } else if (buttonName === 'stderr') {
      this.stderr = !this.stderr;
      enable = this.stderr;
      button = this.stderrButton;
    } else if (buttonName === 'info') {
      this.info = !this.info;
      enable = this.info;
      button = this.infoButton;
    } else if (buttonName === 'warning') {
      this.warning = !this.warning;
      enable = this.warning;
      button = this.warningButton;
    } else if (buttonName === 'error') {
      this.error = !this.error;
      enable = this.error;
      button = this.errorButton;
    } else if (buttonName === 'other') {
      this.other = !this.other;
      enable = this.other;
      button = this.otherButton;
    }

    if (enable) {
      button.style.border = '';
      button.style.backgroundColor = '';
    } else {
      button.style.border = '0px';
      button.style.backgroundColor = 'transparent';
    }

    this.#updateMessagesVisibility();
  }

  #updateMessagesVisibility() {
    for (let i = 0; i < this.textIDs.length; i++) {
      const stdText = this.messagesStd[i];
      const tagText = this.messagesTag[i];

      let stdOn;
      if (stdText === 'stdout')
        stdOn = this.stdout;
      else
        stdOn = this.stderr;

      let tagOn;
      if (tagText === 'info')
        tagOn = this.info;
      else if (tagText === 'warning')
        tagOn = this.warning;
      else if (tagText === 'error')
        tagOn = this.error;
      else
        tagOn = this.other;

      if (stdOn && tagOn)
        document.getElementById(this.textIDs[i]).style.display = 'block';
      else
        document.getElementById(this.textIDs[i]).style.display = 'none';
    }
  }
}
