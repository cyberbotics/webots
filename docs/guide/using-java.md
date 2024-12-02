## Using Java

### Introduction

The Java API has been generated from the C++ API by using SWIG.
That implies that their class hierarchy, their class names and their function names are almost identical.
The Java API is currently composed of a set of about 25 classes having about 200 public functions located in the package called *com.cyberbotics.webots.controller*.
The classes are either representations of a node of the scene tree (such as Robot, LED, etc.) or utility classes (such as Motion, ImageRef, etc.).
A complete description of these functions can be found in the reference guide while the instructions about the common way to program a Java controller can be found in this [chapter](programming-fundamentals.md).

### Java and Java Compiler Installation

In order to develop and run Java controllers for Webots it is necessary to have the 64-bit version of the Java Development Kit (JDK) version 1.8 or later.

The Java Development Kit (JDK) is free for personal and development use and it can be downloaded from the [Oracle Technology Network](http://www.oracle.com/technetwork/java/javase/downloads).
Make sure you choose the most recent 64-bit release of the Standard Edition (SE) of the JDK version 8 or later.
Then, follow the installation instructions.

The `java` command is the Java Virtual Machine (JVM); it is used to execute Java controllers in Webots.
The `javac` command is the Java compiler; it is used to compile Java controllers in Webots' text editor.

These commands should be accessible from a terminal after the installation.
If it is not the case, this can be done by modifying your *PATH* environment variable.

#### Windows

On Windows, the *PATH* variable must be set using the `Environment Variables` dialog.

This dialog can be opened like this: Choose `Start, Settings, Control Panel, System and Security, System` and open `Advanced system settings`.
Select the `Advanced` tab and click on the `Environment Variables` button.

In the dialog, in the `User variables for ...` section, look for a variable named *PATH*.
Add the "bin" path of the installed SDK to the right end of *PATH* variables.
If the *PATH* variable does not exist you should create it.
A typical value for *PATH* is:

```
C:\Program Files\Java\jdk-XXXXXXX\bin
```

Where *jdk-XXXXXX* stands for the actual name of the installed JDK package.

Then, you need to restart Webots so that the change is taken into account.

Note that the *PATH* can also be set globally for all users.
On Linux this can be achieved by adding it in the "/etc/profile" file.
On Windows this can be achieved by adding it to the *Path* variable in the `System variables` part of the `Environment Variables` dialog.

#### Linux

If after the installation ``java`` or ``javac`` commands are not available from your terminal, you should update the *PATH* by adding this line to your "~/.bashrc" or equivalent file:

```sh
$ export PATH=/usr/lib/jvm/java-XXXXXX/bin:$PATH
```

Where *java-XXXXXX* should correspond to the actual name of the installed JDK package.

##### OpenJDK

In alternative to Oracle JDK, on most popular Linux distribution it is also possible to directly install the open-source JDK from the system package manager.
Detailed information can be found on the [OpenJDK website](http://openjdk.java.net/install/index.html).

#### macOS

On macOS, the JDK installer should setup the *PATH* variable automatically, so you shouldn't have to do anything.

#### Troubleshooting the Java Installation

If a Java controller fails to execute or compile, check that the `java` and the `javac` commands are reachable and correspond to the 64-bit version of the JDK.
You can verify this easily by opening a Terminal (Linux and macOS) or a Command Prompt (Windows) and typing `java -version` or `javac -version`.
If these commands are not reachable from the Terminal (or Command Prompt) they will not be reachable by Webots.
If your default java command points to a 32-bit version of Java, Webots may display an error message that looks like this:

```
Native code library failed to load. See the chapter on Dynamic Linking
Problems in the SWIG Java documentation for help.
java.lang.UnsatisfiedLinkError: libJavaController.jnilib: no suitable
image found.
```

### Link with External JAR Files

When a Java controller is either compiled or executed, respectively the `java` and the `javac` commands are executed with the `-classpath` option.
This option is filled internally with the location of the controller library, the location of the current controller directory, and the content of the *CLASSPATH* environment variable.
In order to include third-party jar files, you should define (or modify) this environment variable before running Webots (see the previous section in order to know how to set an environment variable).
Under windows, the CLASSPATH looks like this:

```sh
$ set CLASSPATH=C:\Program Files\java\jdk\bin;relative\mylib.jar
```

While under Linux and macOS, it looks like this:

```sh
$ export CLASSPATH=/usr/lib/jvm/java/bin:relative/mylib.jar
```

### Source Code of the Java API

The source code of the Java API is available in the Webots release.
You may be interested in looking through the [directory containing the Java files]({{ url.github_tree }}/src/controller/java/SWIG_generated_files) in order to get the precise definition of every classes and functions although these files have been generated by SWIG and are difficult to read.

For users who want to use a third-party development environment, it can be useful to know that the package of the Java API ("Controller.jar") is located in the "lib" directory.

Advanced users may want to modify the Java API.
They will need to modify the [SWIG script]({{ url.github_tree }}/src/controller/java/controller.i) and the [Makefile]({{ url.github_tree }}/src/controller/java/Makefile).
