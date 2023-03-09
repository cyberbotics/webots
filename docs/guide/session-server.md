## Session Server

### Description

The session server is the entry point for requesting the start of a new simulation.
It manages the load of the simulation server machines and sends the URL of the available simulation server to the client.

These are the configuration parameters for the session server:
```
# server:             fully qualilified domain name of the session server
# ssl:                for https/wss URL (true by default)
# port:               local port on which the server is listening
# portRewrite:        port rewritten in the URL by apache (true by default)
# simulationServers:  lists all the available simulation servers
# administrator:      email address that will receive notifications
# mailServer:         SMTP mail server host for sending notifications
# mailServerPort:     SMTP mail server port
# mailSender:         email address used to send the notifications
# mailSenderUser:     user name to authenticate on the SMTP server
# mailSenderPassword: password of mailSenderUser
# logDir:             directory where the log file is written
# debug:              output debug information to stdout (false by default)
```

HTTP request handlers:
* The `/` request queries the availability of the simulation servers and returns 1 if some are available or 0 if no simulation server is available.

* The `/monitor` request opens a page showing an overview of the available simulation servers and their load.

WebSocket request handler:
* The `/session` request checks for available simulation servers and returns the URL of the minimally loaded server.

### Hardware Requirements

We recommend to run the session server on the same machine as a simulation server, as it is very lightweight.
See the hardware requirements of a simulation server [here](simulation-server.md#hardware-requirements).
However it may also run on an independent machine if you prefer.
In that case, there is no special hardware requirements for a session server: it doesn't require any specific graphics card, nor a powerful CPU.

It is recommended to configure the BIOS of the computer for automatic reboot after power outage.
This can be done by entering the BIOS at boot time and selecting:
- **Power Management Setup** &rarr; **Restore AC Power Loss** &rarr; **Power on**, or
- **POWER** &rarr; **APM Configuration** &rarr; **Restore on AC Power Loss: [Power On]**, or
- something similar depending on your BIOS.

### Network Requirements

It is necessary for a session server to be available from the Internet, so that it can be used by anyone.
So, the machine running the session server should be attached to a [fully qualified domain name](https://en.wikipedia.org/wiki/Fully_qualified_domain_name) such as `cyberbotics1.epfl.ch` for example.
The local router or firewall should allow inbound connections to this machine on port 443 (HTTPS).

You should also have a SMTP mail server allowing you to send mails from a script (see details below) (optional).
This mail server may run on the local computer or externally through a local provider.
The setup of a mail server is not covered by this guide.

### Software Requirements

Instructions are provided here for Ubuntu 20.04, however it may work on any other version of Linux.
It is assumed that you will be running a simulation server on the same machine.
If not, you can install the server edition of Ubuntu and skip step 2.

1. Install Ubuntu 20.04:
    - Choose the desktop version of Ubuntu 20.04. Simulation servers needs a X display and 3D OpenGL hardware acceleration which is easier to setup from a desktop version.
    - Create a user account named `cyberbotics` (or anything else).
2. Makes that this user account has the auto-login feature enabled, so that when you boot the machine it get automatically logged in.
3. Configure the [unattended upgrades](https://www.linuxbabe.com/ubuntu/automatic-security-update-unattended-upgrades-ubuntu) to reboot after security updates.
4. For your convenience, it is recommended to install ssh: `apt install ssh`.
5. Install apache2: `apt install apache2`.
6. Install a [Let's Encrypt](https://letsencrypt.org) SSL certificate and choose to redirect HTTP traffic to HTTPS:
    - `sudo apt install certbot python3-certbot-apache`
    - `sudo certbot --apache` (choose to redirect HTTP traffic to HTTPS)
7. Install Python dependencies:
    - `sudo apt-get install python3-pip python-is-python3`
    - `pip install tornado`
8. Clone the [webots-server](https://github.com/cyberbotics/webots-server) repository in `~/webots-server`


### Setup

1. Add rewrite rules to redirect traffic to simulation servers and Webots, including WebSocket:
    - `sudo a2enmod proxy proxy_http proxy_wstunnel`
    - Edit `/etc/apache2/site-available/000-default-le-ssl.conf` and add the following lines at the end of the `VirtualHost` section:

    ```
    RewriteEngine on
    # port redirection rules (for session_server.py, simulation_server.py and webots)
    # websockets (should come first)
    RewriteCond %{HTTP:Upgrade} websocket [NC]
    RewriteCond %{HTTP:Connection} upgrade [NC]
    RewriteRule ^/(\d*)/(.*)$ "ws://localhost:$1/$2" [P,L]
    # http traffic (should come after websocket)
    RewriteRule ^/load$ "http://localhost:1999/load" [P,L]
    RewriteRule ^/monitor$ "http://localhost:1999/monitor" [P,L]
    RewriteRule ^/session$ "http://localhost:1999/session" [P,L]
    RewriteRule ^/(\d*)/(.*)$ "http://localhost:$1/$2" [P,L]
    ```

2. Configure the session server:
    - Create a file named `~/webots-server/config/session/session.json` with the following contents (to be adapted to your local setup):

    ```
    {
      "port": 1999,
      "portRewrite": true,
      "server": "cyberbotics1.epfl.ch",
      "administrator": "admin@cyberbotics.com",
      "mailServer": "mail.infomaniak.com",
      "mailServerPort": 587,
      "mailSender": "support@cyberbotics.com",
      "mailSenderPassword": "********",
      "simulationServers": [
        "cyberbotics1.epfl.ch/2000"
      ]
    }
    ```

3. Setup the automatic launch of the session server on reboot.

    ```
    cd ~/.config
    mkdir -p autostart
    cd autostart
    echo "[Desktop Entry]" > session_server.desktop
    echo "Name=session_server" >> session_server.desktop
    echo "Exec=python /home/cyberbotics/webots-server/session_server.py /home/cyberbotics/webots-server/config/session/session.json" >> session_server.desktop
    echo "Type=Application" >> session_server.desktop
    echo "X-GNOME-Autostart-enabled=true" >> session_server.desktop
    ```

4. Reboot your server:
   - `sudo reboot`

### Test

To test the status of your session server, simply visit this URL: https://cyberbotics1.epfl.ch/monitor (replace `cyberbotics1.epfl.ch` with the host name of your local setup).

And check the inbox of the administrator as it may have received a new message.

Now you can setup a [simulation server](simulation-server.md) on the same machine and other machines.
