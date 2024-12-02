## ODE Improvements

In order to extend ODE possibilities and correct some bugs, the version of ODE bundled with Webots was improved.
New functions were added and some existing functions were modified.

### Hinge Joint

It is possible to set and get the suspension axis thanks to the following two functions:

```c
void dJointSetHingeSuspensionAxis (dJointID, dReal x, dReal y, dReal z);
void dJointGetHingeSuspensionAxis (dJointID, dVector3 result);
```

Furthermore, the `dJointSetHingeParam` and `dJointGetHingeParam` functions support `dParamSuspensionERP` and `dParamSuspensionCFM` parameters.

### Hinge 2 Joint

By default in ODE, the suspension is along one of the axes of the joint, in the ODE version of Webots, the suspension has been improved in order to use any arbitrary axis.
It is possible to set and get this axis thanks to the following two functions:

```c
void dJointSetHinge2SuspensionAxis (dJointID, dReal x, dReal y, dReal z);
void dJointGetHinge2SuspensionAxis (dJointID, dVector3 result);
```
