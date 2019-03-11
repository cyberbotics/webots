"""Let the arm hand points to the "TARGET" solid using inverse kinematics (ikpy)."""

try:
    from ikpy.chain import Chain
    from ikpy.link import OriginLink, URDFLink
except ImportError:
    import sys
    sys.exit('The Python module "ikpy" is not installed. Please install it with "pip". Typically: "pip install ikpy"')

from controller import Supervisor

# Create the arm chain.
# The values below are taken from the Irb4600-40.proto file looking at the HingeJoint node fields.
# Note: it's necessary to give up to the "E motor" bone to ikpy, because it defines the hand position.
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

# Initialize the Webots Supervisor.
supervisor = Supervisor()
print('Move the green sphere to move the arm...')

# Initialize the arm motors.
motors = []
for motorName in ['A motor', 'B motor', 'C motor', 'D motor', 'E motor', 'F motor']:
    motor = supervisor.getMotor(motorName)
    motors.append(motor)

# Get the arm and target nodes.
target = supervisor.getFromDef('TARGET')
arm = supervisor.getFromDef('ARM')

# Main loop.
while supervisor.step(32) != -1:
    # Get the absolute postion of the target and the arm base.
    targetPosition = target.getPosition()
    armPosition = arm.getPosition()

    # Compute the position of the target relatively to the arm.
    # Note: x and y axis are inverted because the arm is not aligned with the Webots global axes.
    x = targetPosition[0] - armPosition[0]
    y = - (targetPosition[2] - armPosition[2])
    z = targetPosition[1] - armPosition[1]

    # Call ikpy to inverse the arm.
    ikResults = armChain.inverse_kinematics([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])

    # Actuate the 3 first arm motors with the IK results.
    for i in range(3):
        motors[i].setPosition(
            max(motors[i].getMinPosition(), min(motors[i].getMaxPosition(), ikResults[i + 1]))
        )
