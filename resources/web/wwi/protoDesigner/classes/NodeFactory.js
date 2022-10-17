'use strict';

import {FieldModel} from './FieldModel.js';
import {BaseNode}from './BaseNode.js';
import {ProtoNode}from './ProtoNode.js';


const gProtoModels = new Map();
const gBaseModels = new Map();

export default class NodeFactory {
  static instance;

  constructor() {
    if (this.instance)
      return this.instance;

    this.instance = this;
  }

  createNode(tokenizer) {
    if (tokenizer.peekWord() === 'DEF') {
      tokenizer.nextToken();
      tokenizer.nextToken();
    }

    const nodeName = tokenizer.nextWord();
    console.log('CREATE NODE ' + nodeName);
    let node;
    if (typeof FieldModel[nodeName] !== 'undefined') { // it's a base node
      if (!gBaseModels.has(nodeName)) {
        // create prototype if none is available
        const model = new BaseNode(nodeName);
        gBaseModels.set(nodeName, model);
      }

      node = gBaseModels.get(nodeName).clone();
      //console.log('ORIGINAL', gBaseModels.get(nodeName))
      //console.log('CLONE', node)
    } else { // it's a PROTO node
      // note: protoModels is expected to already contain models for all PROTO we need this session since these have been
      // downloaded when retrieving the EXTERNPROTO and a prototype for each should have been computed already
      if (!tokenizer.proto.externProto.has(nodeName))
        throw new Error('Node name ' + nodeName + ' is not recognized. Was it declared as EXTERNPROTO?');

      const url = tokenizer.proto.externProto.get(nodeName);
      console.log(this)
      if (!gProtoModels.has(url))
        throw new Error('Model of PROTO ' + nodeName + ' not available. Was it declared as EXTERNPROTO?');

      node = gProtoModels.get(url).clone();
    }

    console.log(node)
    node.configureNodeFromTokenizer(tokenizer);
    if (node instanceof ProtoNode)
      node.parseBody()

    return node;
  }

  async createPrototype(protoText, protoUrl) {
    if (!gProtoModels.has(protoUrl)) {
      const proto = new ProtoNode(protoText, protoUrl);
      await proto.fetch();
      console.log('adding proto model: ', protoUrl)
      gProtoModels.set(protoUrl, proto)
    }
  }
}

export { NodeFactory };


