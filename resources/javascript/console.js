class Console {
  constructor() {
    if (!console.instance)
      console.instance = this;
    return console.instance;
  }

  log(arg) {
    stdout += 'ASD';
  }
}

const instance = new Console();
Object.freeze(instance);

export default instance;
