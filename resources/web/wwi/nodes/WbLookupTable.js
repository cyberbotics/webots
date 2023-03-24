export default class WbLookupTable {
  #inputs;
  #minOutput;
  #maxOutput;
  #size;
  constructor(lookupTable) {
    this.#size = lookupTable?.length;
    if (this.#size) {
      this.#inputs = [];

      for (let i = 0; i < this.#size; i++) {
        const v = lookupTable[i];
        this.#inputs[i] = v.x;
      }
    } else {
      this.#inputs = undefined;
    }
  }

  minMetricsRange() {
    return this.#size ? this.#inputs[0] : 0.0;
  }

  maxMetricsRange() {
    return this.#size ? this.#inputs[this.#size - 1] : 0.0;
  }
}
