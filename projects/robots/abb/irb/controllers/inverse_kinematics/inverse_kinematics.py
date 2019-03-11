"""TODO."""

from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink

from controller import Supervisor

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())


armChain = Chain(name='arm', links=[
    OriginLink(),
    URDFLink(
        name="A motor",
        translation_vector=[0, 0, 0.159498],
        orientation=[0, 0, 0],
        rotation=[0, 0, -1],
    ),
    URDFLink(
        name="B motor",
        translation_vector=[0.178445, -0.122498, 0.334888],
        orientation=[0, 0, 0],
        rotation=[0, -1, 0],
    ),
    URDFLink(
        name="C motor",
        translation_vector=[-0.003447, -0.0267, 1.095594],
        orientation=[0, 0, 0],
        rotation=[0, -1, 0],
    ),
    URDFLink(
        name="D motor",
        translation_vector=[0.340095, 0.149198, 0.174998],
        orientation=[0, 0, 0],
        rotation=[1, 0, 0],
    ),
    URDFLink(
        name="E motor",
        translation_vector=[0.929888, 0, 0],
        orientation=[0, 0, 0],
        rotation=[0, 1, 0],
    )
])


motors = []
for motorName in ['A motor', 'B motor', 'C motor', 'D motor', 'E motor', 'F motor']:
    motor = supervisor.getMotor(motorName)
    motors.append(motor)

target = supervisor.getFromDef('TARGET')
arm = supervisor.getFromDef('ARM')

while supervisor.step(4 * timestep) != -1:
    targetPosition = target.getPosition()
    armPosition = arm.getPosition()
    x = targetPosition[0] - armPosition[0]
    y = targetPosition[1] - armPosition[1]
    z = targetPosition[2] - armPosition[2]

    ikResults = armChain.inverse_kinematics([
        [1, 0, 0, x],
        [0, 1, 0, -z],
        [0, 0, 1, y],
        [0, 0, 0, 1]
    ])

    for i in range(3):
        motors[i].setPosition(
            max(motors[i].getMinPosition(), min(motors[i].getMaxPosition(), ikResults[i + 1]))
        )
