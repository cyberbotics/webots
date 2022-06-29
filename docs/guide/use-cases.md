## Use Cases

### 3D Object Designer

As a researcher, I want to design a 3D object model for use in Webots and share it with my colleagues or the robotics community.
My model may be a passive object (like a table), a sensor (like a specific model of a lidar), an actuator (like a gripper), or a simple robot (with no controller program and no robot window).

1. I download and install Webots, Blender, FreeCAD, Gimp and other tools on my computer.
2. I create a 3D model of an object as a Webots PROTO file, meshes and textures.
3. I create a public GitHub repository for my project
4. I push my files on this repository, including:
    - The Webots `.proto` file which refers to a specific version of Webots in the header line.
    - The icon file corresponding to the PROTO file.
    - Some texture images (`.jpeg` or `.png`) inside a `textures` folder.
    - Some mesh files (`.obj`, `.stl` or `.dae`) inside a `meshes` folder.
    - Some sound files (`.wav) inside a `sounds` folder.
    - A `README.md` file.
5. I register my GitHub repository on webots.cloud to advertise it.

It may be possible to provide several versions of the same PROTO for different version of Webots by using different git tags or branch named against the corresponding versions of Webots, e.g., `R2022a`, `R2022b`, etc.

In this case, there is no need for a [docker container](https://github.com/cyberbotics/webots-cloud/wiki/Setup-a-Webots-Project-Repository#docker-solution) as no code is executed (except the Javascript code inside the PROTO, but this doesn't represent any security threat).

### Competition Participant

As a student participating in a robot programming competition, I want to develop a robot controller for a specific scenario provided by the competition organizers. Then, I will want to submit the code of my robot controller to the competition web site.

1. I download and install Webots and the competition scenario.
2. I write and test my robot controller locally.
3. I create a GitHub private repository for my project.
4. I push my files on this repository, including:
    - My robot controller files (written in C, C++, Python or Java).
    - Any resource file: data files, images, etc.
    - If using a compiled language, my `Makefile` allowing to compile the source to get the binary or byte-code files.
    - A `Dockerfile` used to build the [docker image](https://github.com/cyberbotics/webots-cloud/wiki/Setup-a-Webots-Project-Repository#docker-solution) for my controller, including all the needed dependencies: operating system, Webots version, libraries, etc.
5. I submit my GitHub repository to the competition web site (giving organizers read access rights to my private repository).
6. I watch the result on my robot controller running online in real-time or as a recorded animation on the competition web site.
7. I see my ranking evolving in the leader board of the competition web site.
8. I improve my robot controller and loop to step 4 to try to improve my ranking.

**Note**: for some advanced competitions, it may also be possible that participants submit also some PROTO files for their robotics design. Such PROTO files and corresponding resources should be committed on the GitHub repository as well.

### Robot Model Designer

As a developer in a robotics company, I want to design a robot simulation model for Webots and share it with my colleagues and the robotics community. I follow the same procedure as in the **3D Object Designer** use case, I want to include some default controller program, some robot window code, some remote-control code, and/or some specific library. So I will also provide a `Dockerfile` specifying the dependencies of my code, like in the **Competition Participant** use case, so that my code can be executed safely in a sandbox environment. Then, I will publish the corresponding docker image so that users willing to use my robot (including its code dependencies) in their project will inherit their own docker images from mine.

### Researcher Disseminating Achievements

As a university researcher, I developed a Webots simulation from which I achieved some significant research results. I prepared a scientific paper which I plan to submit to a journal. I would like to make my Webots simulation available to the reviewers of the journal and, once the paper is published, to the whole robotics community.

1. I create a GitHub private repository for my project.
2. I push my project files on this repository, including:
    - My world file(s) and related resources (textures, meshes, sounds)
    - My `.proto` files(s) and related resources (textures, meshes, sounds)
    - Robot controller files (written in C, C++, Python or Java) and related resources (data, images, Makefile)
    - My robot windows files and related resources (HTML, Javascript, images, etc.).
    - Several `Dockerfile` used to build the docker image for each robot controller, including all the needed dependencies: operating system, Webots version, ROS version, ROS packages, python version, python modules, libraries, etc.
3. I give read access to my private repository to webots.cloud so that webots.cloud will generate a complex link to an unlisted simulation (similar to youtube unlisted videos).
4. I submit this link to the journal reviewers, so that they can easily run my simulation from webots.cloud.
5. Once my paper is published, I make my repository public and re-submit it to webots.cloud so that anyone can run my project from the web.

**Note**: the requested version of Webots is specified in the header of the main world file.

### Competition Organizer

As a university researcher, I want to organize a robot competition to be held during an international conference.

1. I download and install Webots and all the necessary dependencies needed for my competition.
2. I develop my competition scenario in Webots.
3. I publish my competition scenario on GitHub from this [template](https://github.com/cyberbotics/webots-competition-organizer-template) (similar to **Researcher Disseminating Achievements**)
4. I register my competition repository on webots.cloud.
5. I advertise my competition.
6. I run my competition and give prizes to the winners.

### Robotics Teacher

As a robotics teacher, I want to write a syllabus where my students can practice a series of simulation-based exercises to learn robotics. My exercises should include interactions with running simulations and programming the behavior of the robots. I should be able to write some course content, e.g., rich text, images, videos, code snippets, etc.

1. Follow the procedure described in **Researcher Disseminating Achievements** to create a GitHub repository including the simulations.
2. Add course content files, e.g., markdown files with images, videos, code snippets, etc. and link to specific simulations.
3. Register the GitHub repository on webots.cloud.
4. webots.cloud will automatically detect the markdown files for the course content and display them to browse the course and run the corresponding simulations.
