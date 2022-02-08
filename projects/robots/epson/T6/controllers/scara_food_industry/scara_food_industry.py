"""scara_example controller."""

from controller import Supervisor
from random import randint

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

# actuators
base_arm = supervisor.getDevice("base_arm_motor")
arm = supervisor.getDevice("arm_motor")
shaft_linear = supervisor.getDevice("shaft_linear_motor")
led = supervisor.getDevice("epson_led")

# sensors
base_arm_pos = supervisor.getDevice('base_arm_position')
base_arm_pos.enable(timestep)
arm_pos = supervisor.getDevice('arm_position')
arm_pos.enable(timestep)
shaft_pos = supervisor.getDevice('shaft_linear_position')
shaft_pos.enable(timestep)

ledStatus = True
merged_tool = False
fruitType = 0


def merge_tool(fruit_id):
    global merged_tool

    if not merged_tool:
        fruit = supervisor.getFromDef("fruit" + str(fruit_id))
        gripper = supervisor.getSelf()
        if fruit:
            string = fruit.exportString()
            fruit.remove()
            field = gripper.getField("handSlot")

            newString = ""
            for line in string.splitlines():
                if line.startswith("  translation "):
                    line = " translation 0 0 -0.07\n"
                if line.startswith("  rotation "):
                    line = "  rotation 0 0 0 0\n"
                newString += line
            field.importMFNodeFromString(-1, newString)
            merged_tool = True


def unmerge_tool(fruit_id):
    global merged_tool

    if merged_tool:
        fruit = supervisor.getFromDef("fruit" + str(fruit_id))
        gripper = supervisor.getSelf()

        if fruit:
            string = fruit.exportString()
            oldPos = fruit.getPose()
            oldPosStr = " translation " + str(oldPos[3]) + " " + str(oldPos[7]) + " " + str(oldPos[11]) + "\n"
            newString = ""
            for line in string.splitlines():
                if line.startswith("  translation "):
                    line = oldPosStr
                if line.startswith("  rotation "):
                    line = "  rotation 0 0 0 0\n"
                newString += line
            fruit.remove()
            root = supervisor.getRoot()
            field = root.getField("children")
            field.importMFNodeFromString(-1, newString)
            gripper.resetPhysics()
            merged_tool = False


def ledAnimation():
    global ledStatus
    led.set(ledStatus)
    ledStatus = not(ledStatus)


def animation():
    global fruitType

    if supervisor.step(0) == -1:
        return
    arm.setPosition(0.6)
    base_arm.setPosition(0.2)
    ledAnimation()

    if supervisor.step(400) == -1:
        return
    shaft_linear.setPosition(-0.148)
    ledAnimation()

    if supervisor.step(800) == -1:
        return
    merge_tool(fruitType)
    shaft_linear.setPosition(0)
    ledAnimation()

    if supervisor.step(900) == -1:
        led()
        return
    if fruitType:
        base_arm.setPosition(0)
        arm.setPosition(-0.83)
    else:
        base_arm.setPosition(-0.73)
        arm.setPosition(-0.83)
    ledAnimation()
    if supervisor.step(1100) == -1:
        led()
        return
    unmerge_tool(fruitType)
    fruitType = randint(0, 1)
    ledAnimation()


while supervisor.step(timestep) != -1:

    animation()
