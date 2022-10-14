'use strict';

import {FieldModel} from './FieldModel.js';
import {BaseNode}from './BaseNode.js';

let gProtoModels = new Map();
let gBaseModels = new Map();

export function createNode(tokenizer, externProto) {
  const nodeName = tokenizer.nextWord();
  console.log('CREATE NODE ' + nodeName);
  let node;
  if (typeof FieldModel[nodeName] !== 'undefined') { // it's a base node
    if (!gBaseModels.has(nodeName)) {
      // create prototype is none is available
      const model = new BaseNode(nodeName, externProto);
      gBaseModels.set(nodeName, model);
    }

    node = gBaseModels.get(nodeName).clone();
  } else { // it's a PROTO node
    // note: gProtoModels is expected to already contain models for all PROTO we need this session since these have been
    // downloaded when retrieving the EXTERNPROTO and a prototype for each should have been computed already
    if (!externProto.has(nodeName))
      throw new Error('Node name ' + nodeName + ' is not recognized. Was it declared as EXTERNPROTO?');

    const url = externProto.get(nodeName);
    if (!gProtoModels.has(url))
      throw new Error('Model of PROTO ' + nodeName + ' not available. Was it declared as EXTERNPROTO?');

    node = gProtoModels.get(url).clone();
  }

  node.configureNodeFromTokenizer(tokenizer);

  return node;
}

export async function createPrototype(protoText, protoUrl) {
  if (!gProtoModels.has(protoUrl)) {
    const proto = new ProtoNode(protoText, protoUrl);
    await proto.fetch();
    gProtoModels.set(protoUrl, proto)
  }
}



