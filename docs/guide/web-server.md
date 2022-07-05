# Web Server

This chapter describes how to setup a simulation web service similar to [webots.cloud](https://webots.cloud) to run Webots in the cloud.
Such a system may be distributed on several machines to provide a powerful cluster of simulation servers.
One machine runs a session server that communicates with several simulation servers.
Each machine runs one instance of a simulation server that receives requests from the session server and starts for each connected client a new instance of Webots that communicates directly with the client.

Webots instances can be executed in a secure environment using [Docker](https://www.docker.com).
This is needed if the simulations are coming from the outside world and may contain some malicious code that could compromise the simulation server.
That is the case with [robotbenchmark.net](https://robotbenchmark.net) where robot controllers are python programs written by external users and may potentially harm the simulation server.
Other use cases include simulations created by external users that include binary code for a physics plug-in or a robot window.
Running them in a Docker container ensures the integrity of the simulation server.
However, if the simulations executed on a simulation server can't contain any malicious code, then it is safe to run the Webots instances without Docker.
This is the case if the simulation servers run only simulations from a limited list of allowed GitHub repositories controlled by the owner of the simulation servers.

**Note:** The Web Simulation system is still work in progress and could change in the next releases of Webots.

## Sections

- [Prerequisites and Overview](prerequisites-and-overview.md)
- [Protocol and Quick Start](protocol-and-quick-start.md)
- [Session Server](session-server.md)
- [Simulation Server](simulation-server.md)
- [Last Steps](last-steps.md)
- [Advanced configuration](advanced-configuration.md)
- [Setup a Webots Project Repository](setup-a-webots-project-repository.md)
