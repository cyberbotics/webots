## Last Steps

### Port Rewrite

In order to make the web simulation work properly from the outside world over SSL, you have to configure your web server, `session_server.py` and `simulation_server.py` to perform port rewrite.
Requests arriving to the web server will be redirected to the local network, e.g., `wss://webserver.com/2000/client` &rarr; `ws://webserver.com:2000/client` and `https://webserver.com/2000/monitor` &rarr; `http://webserver.com:2000/monitor`.
You just need to have port 443 open in your local firewall for incoming connections.
Such a setup is compatible with any client firewall allowing outgoing connections on port 443 to your server (which is very standard).

Port rewrite can be tested on a local host by setting up [XAMPP to use SSL](https://gist.github.com/nguyenanhtu/33aa7ffb6c36fdc110ea8624eeb51e69).
In the `session_server.py` configuration, the `simulationServers` should be listed using the outside URL: `hostname/2000` instead of `hostname:2000`.
Your web server should be configured to redirect `http` traffic to `https` and to rewrite ports in URLs for both `https` and `wss`.
With the Apache web server, this can be achieved by adding the following rules in your `httpd.conf` file:

%tab-component "generic"

%tab "webserver.com"
```
LoadModule proxy_module modules/mod_proxy.so
LoadModule proxy_http_module modules/mod_proxy_http.so
LoadModule proxy_wstunnel_module modules/mod_proxy_wstunnel.so

<VirtualHost *:80>
  ServerName webserver.com
  ServerAlias www.webserver.com

  [ ... ]

  RewriteEngine on

  # this rule redirect HTTP requests to HTTPS, removing the 'www.' prefix if any
  RewriteCond %{SERVER_NAME} =%{SERVER_NAME} [OR]
  RewriteCond %{SERVER_NAME} =www.%{SERVER_NAME}
  RewriteRule ^ https://%{SERVER_NAME}%{REQUEST_URI} [END,NE,R=permanent]
</VirtualHost>

<VirtualHost *:443>
  ServerName webserver.com
  ServerAlias www.webserver.com

  [ ... ]

  RewriteEngine on

  # this rule removes the 'www.' prefix from the hostname in the URL if any
  RewriteCond %{SERVER_NAME} =www.webserver.com
  RewriteRule ^ https://webserver.com%{REQUEST_URI} [END,NE,R=permanent]

  # port redirection rules (for session_server.py, simulation_server.py and webots)
  # websockets (should come first)
  RewriteCond %{HTTP:Upgrade} websocket [NC]
  RewriteCond %{HTTP:Connection} upgrade [NC]
  RewriteRule ^/(\d*)/(.*)$ "ws://%{SERVER_NAME}:$1/$2" [P,L]
  # http traffic (should come after websocket)
  RewriteRule ^/(\d*)/(.*)$ "http://%{SERVER_NAME}:$1/$2" [P,L]

</VirtualHost>
```
%tab-end

%tab "localhost"
```
LoadModule alias_module modules/mod_alias.so
LoadModule proxy_module modules/mod_proxy.so
LoadModule proxy_http_module modules/mod_proxy_http.so
LoadModule proxy_wstunnel_module modules/mod_proxy_wstunnel.so

<VirtualHost *:80>
  ServerName localhost

  [ ... ]

  # this rule redirect HTTP requests to HTTPS
  Redirect "/" "https://localhost/"
</VirtualHost>

<VirtualHost *:443>
  ServerName localhost

  [ ... ]

  RewriteEngine on

  # port redirection rules (for session_server.py, simulation_server.py and webots)
  # websockets (should come first)
  RewriteCond %{HTTP:Upgrade} websocket [NC]
  RewriteCond %{HTTP:Connection} upgrade [NC]
  RewriteRule ^/(\d*)/(.*)$ "ws://%{SERVER_NAME}:$1/$2" [P,L]
  # http traffic (should come after websocket)
  RewriteRule ^/(\d*)/(.*)$ "http://%{SERVER_NAME}:$1/$2" [P,L]

</VirtualHost>
```
%tab-end

%end

**Note:** Port rewrite can be disabled for testing purposes in the `session_server.py` and `simulation_server.py` by simply setting the `portRewrite` configuration option to `false`.

### Server Display

On Linux, a working Linux display (":0") should be available to run Webots remotely.
This can be achieved generally simply by executing `xhost +` on the server computer.

**Note:** On some computers, in addition to the previous comment, the display entity is linked with the fact that a monitor is plugged.
In this case, you can open automatically a user session when the computer is switched on, run `session_server.py` and `simulation_server.py` automatically when the session starts up, and let the monitor switched on at Ubuntu startup.
If you have a headless system, i.e., a system without any physical monitors attached, then with the NVIDIA graphics card you could fake a monitor in the X session.
This solution basically consists in adding a screen configuration to the X server configuration file by copying the Extended Display Identification Data (EDID) of a temporary attached monitor.

### Startup Procedure

The startup procedure is the following:

1. Start all the simulation servers (`simulation_server.py`).
2. Start the session server (`session_server.py`).

This procedure should be automated in a startup script, so that the servers are restarted after a reboot of the machine.

The folder containing `simulation_server.py` and `session_server.py` should also contain a `server.sh` utility script to automatically start and stop both `session_server.py` and `simulation_server.py` with a given configuration file.

Please make sure that the `WEBOTS_HOME` variable is set before running the simulation and session server scripts.
