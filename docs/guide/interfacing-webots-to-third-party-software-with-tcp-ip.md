## Interfacing Webots to Third Party Software with TCP/IP

### Overview

Webots offers programming APIs for the following languages: C/C++, Java, Python and *MATLAB*<sup>TM</sup>.
It is also possible to interface Webots with other programming languages of software packages, such as *Lisp*<sup>TM</sup>, *LabView*<sup>TM</sup>, etc.
Such an interface can be implemented through a TCP/IP protocol that you can define yourself.
Webots comes with an example of interfacing a simulated Khepera robot via TCP/IP to any third party program able to read from and write to a TCP/IP connection.
This example world is called "[khepera\_tcpip.wbt]({{ url.github_tree  }}/projects/robots/k-team/khepera1/worlds/khepera_tcpip.wbt)", and can be found in the "[WEBOTS\_HOME/projects/robots/k-team/khepera1/worlds]({{ url.github_tree  }}/projects/robots/k-team/khepera1/worlds)" directory of Webots.
The simulated Khepera robot is controlled by the "[tcpip]({{ url.github_tree  }}/projects/robots/k-team/khepera1/controllers/tcpip)" controller which is in the "[controllers](https://github.com/cyberbotics/webots/blob/master/projects/robots/k-team/khepera1/controllers)" directory of the same project.
This small C controller comes with full source code in "[tcpip.c]({{ url.github_tree  }}/projects/robots/k-team/khepera1/controllers/tcpip/tcpip.c)", so that you can modify it to suit your needs.
A client example is provided in "[client.c]({{ url.github_tree  }}/projects/robots/k-team/khepera1/controllers/tcpip/client.c)".
This client may be used as a model to write a similar client using the programming language of your third party software.
This has already been implemented in *Lisp*<sup>TM</sup> and MATLAB by some Webots users.

### Main Advantages

There are several advantages of using such an interface.
First, you can have several simulated robots in the same world using several instances of the same "tcpip" controller, each using a different TCP/IP port, thus allowing your third party software to control several robots through several TCP/IP connections.
To allow the "tcpip" process to open a different port depending on the controlled robot, you should give a different `name` to each robot and use the `robot_get_name` function in the "tcpip" controller to retrieve this name and decide which port to open for each robot.

The second advantage is that you can also control a real robot from your third party software by simply implementing your library based on the given remote control library.
Switching to the remote control mode will redirect the input/output to the real robot through the Inter-Process Communication (IPC).
An example of remote control is implemented for the e-puck robot in the file "[WEBOTS\_HOME/projects/robots/gctronic/e-puck/worlds/e-puck.wbt]({{ url.github_tree  }}/projects/robots/gctronic/e-puck/worlds/e-puck.wbt)" directory of Webots.

The third advantage is that you can spread your controller programs over a network of computers.
This is especially useful if the controller programs perform computationally expensive algorithms such as genetic algorithms or other learning techniques.

Finally, you should set the controlled robot to synchronous or asynchronous mode depending on whether or not you want the Webots simulator to wait for commands from your controllers.
In synchronous mode (with the `synchronization` field of the robot equal to `TRUE`), the simulator will wait for commands from your controllers.
The controller step defined by the `robot_step` parameter of the "tcpip" controller will be respected.
In asynchronous mode (with the `synchronization` field of the robot set to `FALSE`), the simulator will run as fast as possible, without waiting for commands from your controllers.
In the latter case, you may want to run the simulation in real time mode so that robots will behave like real robots controlled through an asynchronous connection.

### Limitations

The main drawback of TCP/IP interfacing is that if your robot has a camera device, the protocol must send the images to the controller via TCP/IP, which might be network intensive.
Hence it is recommended to have a high speed network, or use small resolution camera images, or compress the image data before sending it to the controller.
This overhead is negligible if you use a low resolution camera such as the Khepera K213 (see example "[WEBOTS\_HOME/projects/robots/k-team/khepera1/worlds/khepera\_k213.wbt]({{ url.github_tree  }}/projects/robots/k-team/khepera1/worlds/khepera_k213.wbt)").
