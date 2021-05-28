## Starting Webots Remotely ("ssh")

Webots can be started on a remote computer, by using `ssh` (or a similar) command.
However, Webots will work only if it can get a X11 connection to a X-server running locally (on the same computer).
It is currently not possible to redirect Webots graphical output to another computer.

### Using the "ssh" Command

Here is the usual way to start from computer A, a Webots instance that will run on computer B:

```sh
$ ssh myname@computerB.org
$ export DISPLAY=:0.0
$ webots --mode=fast --no-rendering --stdout --stderr myworld.wbt
```

The first line logs onto computer B.
The 2nd line sets the DISPLAY variable to the display 0 (and screen 0) of computer B.
This will indicate to all X11 applications (including Webots) that they need to connect to the X-server running on the local computer: computer B in this case.
This step is necessary because the DISPLAY variable is usually not set in an `ssh` session.

The last line starts Webots: the *--mode=fast* option enables the *Fast* simulation mode.
The *--mode=fast* in conjunction with *--no-rendering* option makes the simulation run as fast as possible, without graphical rendering, which is fine because the graphical output won't be visible anyway from computer A.
Options *--stdout* and *--stderr* are used to redirect Webots' output to the standard streams instead of Webots console, otherwise the output would not be visible on computer A.

At this point, Webots will start only if a X-server with proper authorizations is running on computer B.
To ensure that this is the case, the simplest solution is to have an open login session on computer B, i.e., to have logged in using the login screen of computer B, and not having logged out.
Unless configured differently, the `ssh` login and the screen login session must belong to the same user, otherwise the X-server will reject the connection.
Note that the `xhost +` command can be used to grant access to the X-server to another user.
For security reasons, the screen of the open session on computer B can be locked (e.g. with a screen-saver): this won't affect the running X-server.

### Terminating the "ssh" Session

A little problem with the above approach is that closing the `ssh` session will kill the remote jobs, including Webots.
Fortunately it is easy to overcome this problem by starting the Webots as a background job and redirecting its output to a file:

```sh
$ ssh myname@computerB.org
$ export DISPLAY=:0.0
$ webots --mode=fast --no-rendering --stdout --stderr myworld.wbt &> out.txt &
$ exit
```

The &> sign redirects into a text file the output that would otherwise appear in the `ssh` terminal.
The & sign starts Webots as a background job: so the user can safely exit the `ssh` session, while Webots keeps running.

In this case the decision to terminate the job is usually made in the [Supervisor](../reference/supervisor.md) controller code according to simulation specific criteria.
The `wb_supervisor_simulation_quit` function can be used to automatically terminate Webots when the job is over.
