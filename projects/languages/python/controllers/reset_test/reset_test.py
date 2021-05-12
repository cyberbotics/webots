from controller import Supervisor


robot = Supervisor()
root = robot.getRoot()
children = root.getField('children')
newNode = None

while robot.step(32) != -1:
    if robot.getTime() > 0.2:
        robot.simulationReset()
        robot.step(32)
        if  newNode:
            print('B new node', newNode.getTypeName())
        human_model = '/home/stefi/Desktop/human.wbo'
        children.importMFNode(-1, human_model)
        newNode = children.getMFNode(11)
        print('A new node', newNode.getTypeName())
        