## Controller Start-up

The .wbt file contains the name of the controller that needs to be started for each robot.
The controller name is a platform and language independent field; for example when a controller name is specified as "xyz\_controller" in the .wbt file, this does not say anything about the controller's programming language or platform.
This is done deliberately to ensure that the *.wbt* file is independent from the platform and programming language.

When Webots tries to start a controller it must first determine what programming language is used by this controller.
So, Webots looks in the project's *controllers* directory for a subdirectory that matches the name of the controller.
Then, in this controller directory, it looks for a file that matches the controller's name.
For example if the controller's name is "xyz\_controller", then Webots looks for these files in the specified order, in the "PROJECT\_DIRECTORY/controllers/xyz\_controller" directory.

1. "xyz\_controller[.exe]" (a binary executable)
2. "xyz\_controller.class" (a Java bytecode class)
3. "xyz\_controller.jar" (a Java .jar file)
4. "xyz\_controller.bsg" (a Webots/BotStudio file)
5. "xyz\_controller.py" (a Python script)
6. "xyz\_controller.m" (a MATLAB function)

The first file that is found will be executed by Webots using the required language interpreter (Java, Python, MATLAB).
So the priority is defined by the file extension, e.g. it won't be possible to execute "xyz\_controller.m" if a file named "xyz\_controller.py" is also present in the same controller directory.
In the case that none of the above filenames exist or if the required language interpreter is not found, an error message will be issued and Webots will start the "<generic>" controller instead.

> **Note** [Java]: In the Java case there are two options.
The controller can be placed in a ".class" file or in a ".jar" file.
If a ".class" file is used, it must be named "xyz\_controller.class".
If a ".jar" file is used it must be named "xyz\_controller.jar" and it must contain a class named "xyz\_controller" that Webots will attempts to start.
