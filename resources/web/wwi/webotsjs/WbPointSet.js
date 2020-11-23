import {WbGeometry} from "./WbGeometry.js"

class WbPointSet extends WbGeometry {
  constructor(id){
    super(id);
  }
  
  postFinalize() {
    super.postFinalize();
  }
}

export {WbPointSet}
