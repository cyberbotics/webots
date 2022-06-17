#!/usr/bin/env python3

# Copyright 1996-2022 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Webots session server."""

import asyncio
import json
import logging
import os
import smtplib
import socket
import sys
import threading
import time
import tornado.ioloop
import tornado.httpserver
import tornado.web
import tornado.websocket
import traceback
from urllib.request import urlopen, URLError

LOAD_THRESHOLD = 99  # disable a simulation server when its load is 99% or more
LOAD_REFRESH = 5  # query load of simulation servers every 5 seconds

availability = False


def expand_path(path):
    """Expand user and environmental variables in a string."""
    return os.path.expandvars(os.path.expanduser(path))


class SessionHandler(tornado.web.RequestHandler):
    """Handle simulation session requests."""

    def set_default_headers(self):
        """Set headers needed to avoid cross-origin resources sharing (CORS) errors."""
        self.set_header('Access-Control-Allow-Origin', '*')
        self.set_header('Access-Control-Allow-Headers', 'x-requested-with')
        self.set_header('Access-Control-Allow-Methods', 'POST, GET, OPTIONS')

    def get(self):
        """Return the less loaded simulation server."""
        global LOAD_THRESHOLD
        global simulation_server_loads
        minimum = LOAD_THRESHOLD
        minimally_loaded_server = ''
        for i in range(len(config['simulationServers'])):
            if simulation_server_loads[i] >= LOAD_THRESHOLD:
                continue
            if simulation_server_loads[i] < minimum:
                minimum = simulation_server_loads[i]
                minimally_loaded_server = config['simulationServers'][i]
        if minimum < LOAD_THRESHOLD:
            protocol = 'wss:' if config['ssl'] else 'ws:'
            self.write(protocol + '//' + minimally_loaded_server)
            return
        self.write('Error: no simulation server available at the moment.')


class LoadHandler(tornado.web.RequestHandler):
    """Provide the current load of the simulation servers managed by the session server."""

    def get(self):
        average_load = 0
        average_count = 0
        for i in range(len(config['simulationServers'])):
            if simulation_server_loads[i] != 1000:
                average_load += simulation_server_loads[i]
                average_count += 1
        if average_count == 0:
            average_load = -1  # N/A
        else:
            average_load /= average_count
        self.write(str(average_load))


class MonitorHandler(tornado.web.RequestHandler):
    """Display the monitor web page."""

    def get(self):
        """Write the web page content."""
        self.write('<!DOCTYPE html>\n')
        self.write('<html>\n<head><meta charset="utf-8"/><title>Webots simulation server</title>')
        self.write('<link rel="stylesheet" type="text/css" href="https://cyberbotics.com/wwi/R2022b/css/monitor.css">')
        self.write('</head>\n<body><h1>Webots session server</h1>\n')
        nServer = len(config['simulationServers'])
        self.write(f'<p>Started on {start_time}</p>\n')
        self.write(f'<p>Managing {nServer} simulation server')
        if nServer > 1:
            self.write('s')
        self.write(':</p>')
        self.write('<table class="bordered"><thead><tr><th>#</th><th>simulation server</th><th>load</th></tr></thead>\n')
        average_load = 0
        average_count = 0
        for i in range(nServer):
            self.write(f'<tr><td>{i + 1}</td>')
            url = 'https' if config['ssl'] else 'http'
            url += f'://{config["simulationServers"][i]}/monitor'
            self.write(f'<td><a href="{url}">{config["simulationServers"][i]}</a></td><td>')
            if simulation_server_loads[i] >= LOAD_THRESHOLD:
                self.write('<font color="red">')
            if simulation_server_loads[i] == 1000:
                value = 'N/A'
            else:
                value = f'{simulation_server_loads[i]}%'
                average_load += simulation_server_loads[i]
                average_count += 1
            self.write(value)
            if simulation_server_loads[i] >= LOAD_THRESHOLD:
                self.write('</font>')
            self.write('</td></tr>\n')
        if average_count == 0:
            average_load = 1000
        else:
            average_load /= average_count
        self.write('<tr><td></td><td style="text-align:right">average:</td><td><b>')
        if average_load >= LOAD_THRESHOLD:
            self.write('<font color="red">')
        if average_load == 1000:
            self.write('N/A')
        else:
            self.write(f'{average_load}%%')
        if average_load >= LOAD_THRESHOLD:
            self.write('</font>')
        self.write('</b></td></tr>\n')
        self.write('</table>\n')
        self.write('</body></html>')


class ClientWebSocketHandler(tornado.websocket.WebSocketHandler):
    """This class handles websocket connections."""

    clients = set()

    def check_origin(self, origin):
        """Allow to run the server on the same computer as the client."""
        return True

    def open(self):
        """Open a new connection for an incoming client."""
        global availability
        self.set_nodelay(True)
        ClientWebSocketHandler.clients.add(self)
        if availability:
            message = '1'
        else:
            message = '0'
        self.write_message(message)
        logging.info(f'[{self.request.host}] New client')

    def on_message(self, message):
        """Log message received from client."""
        logging.info(f'[{self.request.host}] Ignored client message: {message}')

    def on_close(self):
        """Close connection after client leaves."""
        logging.info(f'[{self.request.host}] Client disconnected')
        ClientWebSocketHandler.clients.remove(self)


def send_email(subject, content):
    """Send notification email."""
    sender = config['mailSender']
    receivers = [config['administrator']]
    if 'mailServerPort' in config:
        port = config['mailServerPort']
    else:
        port = 0

    message = f'From: Simulation Server <{sender}>\n' + \
              f'To: Administrator <{config["administrator"]}>\n' + \
              f'Subject: {subject}\n\n{content}'
    try:
        with smtplib.SMTP(config['mailServer'], port, timeout=5) as smtp:
            if 'mailSenderPassword' in config:
                smtp.starttls()
                if 'mailSenderUser' in config:
                    user = config['mailSenderUser']
                else:
                    user = sender
                smtp.login(user, config['mailSenderPassword'])
            smtp.sendmail(sender, receivers, message)
    except smtplib.SMTPException:
        logging.error(f'Error: unable to send email to {config["administrator"]}\n')


def retrieve_load(url, i):
    """Contact the i-th simulation server and retrieve its load."""
    global simulation_server_loads
    url = f'https://{url}' if config['ssl'] else f'http://{url}'
    url += '/load'
    if 'administrator' in config:
        protocol = 'https://' if config['ssl'] else 'http://'
        separator = '/' if config['portRewrite'] else ':'
        check_string = f'Check it at {protocol}{config["server"]}{separator}{config["port"]}/monitor\n\n' + \
                       '-Simulation Server'
    try:
        response = urlopen(url, timeout=5)
    except URLError:
        if simulation_server_loads[i] != 1000:
            if 'administrator' in config:
                send_email('Simulation server not responding', f'Hello,\n\n{config["simulationServers"][i]} ' +
                           'simulation server may be down, as it is not responding to the requests of the session server' +
                           f'...\n{check_string}')
            else:
                logging.info(f'{config["simulationServers"][i]} simulation server is not responding (assuming 100% load)')
            simulation_server_loads[i] = 1000
    except socket.timeout:
        if simulation_server_loads[i] != 1000:
            logging.info(f'{config["simulationServers"][i]} simulation server is taking too long to respond '
                         '(assuming 100% load)')
            simulation_server_loads[i] = 1000
    else:
        load = float(response.read())
        if simulation_server_loads[i] == 1000:
            message = f'{config["simulationServers"][i]} simulation server is up and running again ' + \
                      f'(load = {load}%)'
            if 'administrator' in config:
                send_email('Simulation server working again', f'Hello,\n\n{message}.\n{check_string}')
            else:
                logging.info(message)
        elif simulation_server_loads[i] < LOAD_THRESHOLD and load >= LOAD_THRESHOLD:
            message = f'{config["simulationServers"][i]} simulation server has reached maximum load ({load}%)'
            if 'administrator' in config:
                send_email('Simulation server reached maximum load', f'Hello,\n\n{message}.\n{check_string}')
            else:
                logging.info(message)
        elif simulation_server_loads[i] >= LOAD_THRESHOLD and load < LOAD_THRESHOLD:
            message = f'{config["simulationServers"][i]} simulation server is available again (load = {load}%)'
            if 'administrator' in config:
                send_email('Simulation server available again', f'Hello,\n\n{message}.\n{check_string}')
            else:
                logging.info(message)
        simulation_server_loads[i] = load


def update_load():
    """Check regularly the simulation servers load and notify the client if availability changed."""
    """This function should take no more than 1 second because the timeout is set to this value and requests are threaded."""
    global LOAD_REFRESH
    global availability
    global simulation_server_loads
    threads = [threading.Thread(target=retrieve_load, args=(config['simulationServers'][i], i))
               for i in range(len(config['simulationServers']))]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    new_availability = False
    for i in range(len(config['simulationServers'])):
        if simulation_server_loads[i] < LOAD_THRESHOLD:
            new_availability = True
    if new_availability != availability:
        availability = new_availability
        if availability:
            message = '1'
        else:
            message = '0'
        for client in ClientWebSocketHandler.clients:
            client.write_message(message)

    tornado.ioloop.IOLoop.current().add_timeout(int(time.time()) + LOAD_REFRESH, update_load)


def main():
    """Start the session server."""
    # the following config variables read from the config.json file
    # are described here:
    #
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
    #
    global config
    global simulation_server_loads

    # logging system
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.DEBUG)
    if 'logDir' not in config:
        config['logDir'] = 'log'
    else:
        config['logDir'] = expand_path(config['logDir'])
    if 'ssl' not in config:
        config['ssl'] = True
    if 'portRewrite' not in config:
        config['portRewrite'] = True
    if 'debug' not in config:
        config['debug'] = False
    if not os.path.isabs(config['logDir']):
        config['logDir'] = os.path.join(os.path.dirname(os.path.abspath(__file__)), config['logDir'])
    sessionLogDir = os.path.join(config['logDir'], 'session')
    logFile = os.path.join(sessionLogDir, 'output.log')
    try:
        if not os.path.exists(sessionLogDir):
            os.makedirs(sessionLogDir)
        if config['debug']:
            file_handler = logging.StreamHandler(sys.stdout)
        else:
            file_handler = logging.handlers.RotatingFileHandler(logFile, maxBytes=500000, backupCount=10)
        formatter = logging.Formatter('%(asctime)-15s [%(levelname)-7s]  %(message)s')
        file_handler.setFormatter(formatter)
        file_handler.setLevel(logging.INFO)
        root_logger.addHandler(file_handler)
    except (OSError, IOError) as e:
        sys.exit(f'Log file {logFile} cannot be created: {e}')
    # disable tornado.access INFO logs
    tornado_access_log = logging.getLogger('tornado.access')
    tornado_access_log.setLevel(logging.WARNING)

    simulation_server_loads = [0] * len(config['simulationServers'])
    if 'administrator' in config:
        if 'mailServer' not in config:
            logging.info('No mail server defined in configuration, disabling e-mail notifications to ' +
                         f'{config["administrator"]}.')
            del config['administrator']
        elif 'mailSender' not in config:
            logging.info('No mail sender defined in configuration, disabling e-mail notifications to ' +
                         f'{config["administrator"]}.')
            del config['administrator']
    handlers = []
    handlers.append((r'/session', SessionHandler))
    handlers.append((r'/', ClientWebSocketHandler))
    handlers.append((r'/monitor', MonitorHandler))
    handlers.append((r'/load', LoadHandler))
    application = tornado.web.Application(handlers)
    if 'server' not in config:
        config['server'] = 'localhost'
    http_server = tornado.httpserver.HTTPServer(application)
    update_load()
    http_server.listen(config['port'])
    message = f'Session server running on port {config["port"]}'
    logging.info(message)

    try:
        tornado.ioloop.IOLoop.current().start()
    except Exception:
        logging.error(traceback.format_exc() + '\n')
        for client in ClientWebSocketHandler.clients:
            del client


# ensure we are in the script directory
os.chdir(os.path.dirname(os.path.realpath(__file__)))
argc = len(sys.argv)
if argc == 1:
    config_json = 'config/session/local.json'
elif argc == 2:
    config_json = sys.argv[1]
else:
    sys.exit('Too many arguments.')
if not os.path.isabs(config_json):
    config_json = os.path.join(os.path.dirname(os.path.abspath(__file__)), config_json)
with open(config_json) as config_file:
    config = json.load(config_file)
start_time = time.strftime('%A, %B %d, %Y at %H:%M:%S')
if sys.platform == 'win32' and sys.version_info >= (3, 8):
    asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
if __name__ == '__main__':
    main()
