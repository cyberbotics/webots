## Execution Scheme

The following diagram illustrates the sequence of execution of the plugin callback functions.
In addition, the principal interactions of Webots with the ODE functions are indicated.

%figure "Physics Plugin Execution Scheme"
%chart
graph TD
  S0["dWorldCreate()</br>dSimpleSpaceCreate(NULL)</br>contactJointGroup = dJointGroupCreate(0)"] --> S1["Webots joints creation:</br>dJointCreateHinge() for HingeJoint</br>dJointCreateSlider() for SliderJoint</br>etc.</br>dJointAttach"]
  S1 --> S2["Loading the plugin</br>webots_physics_init()</br>(can be used to initialize data, open files, etc.)"]
  S2 --> S3["Webots motor PID-controller:</br>dJointSetHinge/SliderParam(joint, dParamVel, Vc) and</br>dJointSetHinge/SliderParam(joint, dParamFMax, F)</br>are called for each joint</br></br>dBodyAddTorque/Force()</br>is called if spring/damping behavior is wanted (spring/dampingConstant != 0)</br>or if wb_motor_set_force/torque was invoked"]
  S3 --> S4["dSpaceCollide(space)</br>is invoked which eventually causes multiple callbacks to</br>webots_physics_collide()"]
  S4 --> S5["webots_physics_step()</br>is called to allow the user to setup additional forces, etc."]
  S5 --> S6["rigid body simulation:</br>dWorldStep()"]
  S6 --> S7["webots_physics_step_end()</br>is called to allow the user to read dJointFeedback structs"]
  S7 --> S8["dJointGroupEmpty(contactJointGroup)</br>is invoked to clear the contact joint group"]
    S7 -.->|simulation step| S3
  S8 --> S9["webots_physics_cleanup()</br>(should be used to cleanup memory, close files, etc.)</br>Unloading the plugin"]
  S9 --> S10["dJointGroupDestroy(contactJointGroup)</br>dSpaceDestroy()</br>dWorldDestroy()"]
    S10 -.->|simulation revert| S0
%end
%end
