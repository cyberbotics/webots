import re


class NodeExtractor(object):
    """Helper class to extract basic root nodes string."""
    def __init__(self, wbtFilename):
        with open(wbtFilename, 'r') as f:
            self.wbtContent = f.read()

    def extractRootNodes(self, nodeName):
        nodes = []
        # Iterate on "nodeName {" and then count the curly brackets until the node is closed.
        for m in re.finditer(r'[^a-zA-Z]%s\s*\{' % (nodeName), self.wbtContent):
            nodeString = nodeName + ' {'
            curlyBracketCounter = 1
            index = m.end()  # => character after the first open curly bracket.
            while curlyBracketCounter != 0:
                c = self.wbtContent[index]
                nodeString += c
                if c == '{':
                    curlyBracketCounter += 1
                elif c == '}':
                    curlyBracketCounter -= 1
                index += 1
            nodes.append(nodeString)
        return nodes


# Uncomment and execute this file to debug this class.
# if __name__ == "__main__":
#     import os
#     filename = os.path.join(os.environ['WEBOTS_HOME'], 'projects', 'vehicles', 'worlds', 'city.wbt')
#     ne = NodeExtractor(filename)
#     for nodeString in ne.extractRootNodes('StraightRoadSegment'):
#         print('\n\nNODE\n')
#         print(nodeString)
