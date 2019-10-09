#!/usr/bin/env python3

# Copyright 1996-2019 Cyberbotics Ltd.
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

import json
import logging
import os
import smtplib
import sys
import threading
import time
import tornado.ioloop
import tornado.httpserver
import tornado.web
import tornado.websocket
import traceback
try:  # for Python 3.0 and later
    from urllib.request import urlopen, URLError
except ImportError:
    from urllib2 import urlopen, URLError

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
        self.set_header("Access-Control-Allow-Origin", "*")
        self.set_header("Access-Control-Allow-Headers", "x-requested-with")
        self.set_header('Access-Control-Allow-Methods', 'POST, GET, OPTIONS')

    def get(self):
        """Return the less loaded simulation server."""
        global LOAD_THRESHOLD
        global simulation_server_loads
        minimum = LOAD_THRESHOLD
        minimally_loaded_server = ''
        for i in range(len(config[u'simulationServers'])):
            if simulation_server_loads[i] >= LOAD_THRESHOLD:
                continue
            if simulation_server_loads[i] < minimum:
                minimum = simulation_server_loads[i]
                minimally_loaded_server = config[u'simulationServers'][i]
        if minimum < LOAD_THRESHOLD:
            if config[u'ssl']:
                protocol = 'wss:'
            else:
                protocol = 'ws:'
            self.write(protocol + '//' + minimally_loaded_server)
            return
        self.write('Error: no simulation server available at the moment.')


class MonitorHandler(tornado.web.RequestHandler):
    """Display the monitor web page."""

    def get(self):
        """Write the web page content."""
        self.write("<!DOCTYPE html>\n")
        self.write("<html><head><meta charset='utf-8'/><title>Webots simulation server</title>\n")
        self.write("<link rel='stylesheet' type='text/css' href='css/monitor.css'></head>\n")
        self.write("<body><h1>Webots session server</h1>\n")
        nServer = len(config[u'simulationServers'])
        self.write("<p>Started on " + start_time + "</p>\n")
        self.write("<p>Managing %d simulation server" % nServer)
        if nServer > 1:
            self.write("s")
        self.write(":</p>")
        self.write("<table class='bordered'><thead><tr><th>#</th><th>simulation server</th><th>load</th></tr></thead>\n")
        average_load = 0
        for i in range(nServer):
            self.write("<tr><td>%d</td>" % (i + 1))
            url = "http"
            if config[u'ssl']:
                url += "s"
            url += "://" + config[u'simulationServers'][i] + "/monitor"
            self.write("<td><a href='" + url + "'>" + config[u'simulationServers'][i] + "</a></td><td>")
            if simulation_server_loads[i] >= LOAD_THRESHOLD:
                self.write("<font color='red'>")
            self.write(str(simulation_server_loads[i]) + "%")
            average_load += simulation_server_loads[i]
            if simulation_server_loads[i] >= LOAD_THRESHOLD:
                self.write("</font>")
            self.write("</td></tr>\n")
        if nServer > 1:
            average_load /= nServer
        self.write("<tr><td></td><td style='text-align:right'>average:</td><td><b>")
        if average_load >= LOAD_THRESHOLD:
            self.write("<font color='red'>")
        self.write("%g%%" % average_load)
        if average_load >= LOAD_THRESHOLD:
            self.write("</font>")
        self.write("</b></td></tr>\n")
        self.write("</table>\n")
        self.write("</body></html>")


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
        logging.info('[' + self.request.host + '] New client')

    def on_close(self):
        """Close connection after client leaves."""
        logging.info('[' + self.request.host + '] Client disconnected')
        ClientWebSocketHandler.clients.remove(self)


def send_email(subject, content):
    """Send notification email."""
    sender = config[u'mailSender']
    receivers = [config[u'administrator']]

    message = "From: Simulation Server <" + sender + ">\n" + \
              "To: Administrator <" + config[u'administrator'] + ">\n" + \
              "Subject: " + subject + "\n\n" + content
    try:
        smtp = smtplib.SMTP(config[u'mailServer'])
        smtp.sendmail(sender, receivers, message)
        smtp.quit()
    except smtplib.SMTPException:
        logging.error("Error: unable to send email to " + config[u'administrator'] + "\n")


def read_url(url, i):
    """Contact the i-th simulation server and retrieve its load."""
    global simulation_server_loads
    if u'administrator' in config:
        if config[u'ssl']:
            protocol = 'https://'
        else:
            protocol = 'http://'
        check_string = "Check it at " + protocol + config[u'server'] + ":" + str(config[u'port']) + "/monitor\n\n" + \
                       "-Simulation Server"
    try:
        response = urlopen(url, timeout=1)
    except URLError:
        if simulation_server_loads[i] != 100:
            if u'administrator' in config:
                send_email("Simulation server not responding", "Hello,\n\n" + config[u'simulationServers'][i] +
                           " simulation server may be down, as it is not responding to the requests of the session server" +
                           "...\n" + check_string)
            else:
                logging.info(config[u'simulationServers'][i] + " simulation server is not responding (assuming 100% load)")
        simulation_server_loads[i] = 100
    else:
        load = float(response.read())
        if simulation_server_loads[i] == 100:
            message = config[u'simulationServers'][i] + " simulation server is up and running again (load = " + str(load) + "%)"
            if u'administrator' in config:
                send_email("Simulation server working again", "Hello,\n\n" + message + ".\n" + check_string)
            else:
                logging.info(message)
        elif simulation_server_loads[i] < LOAD_THRESHOLD and load >= LOAD_THRESHOLD:
            message = config[u'simulationServers'][i] + " simulation server has reached maximum load (" + str(load) + "%)"
            if u'administrator' in config:
                send_email("Simulation server reached maximum load", "Hello,\n\n" + message + ".\n" + check_string)
            else:
                logging.info(message)
        elif simulation_server_loads[i] >= LOAD_THRESHOLD and load < LOAD_THRESHOLD:
            message = config[u'simulationServers'][i] + " simulation server is available again (load = " + str(load) + "%)"
            if u'administrator' in config:
                send_email("Simulation server available again", "Hello,\n\n" + message + ".\n" + check_string)
            else:
                logging.info(message)
        simulation_server_loads[i] = load


def update_load():
    """Check regularly the simulation servers load and notify the client if availability changed."""
    """This function should take no more than 1 second because the timeout is set to this value and requests are threaded."""
    global LOAD_REFRESH
    global availability
    global simulation_server_loads
    if config[u'ssl']:
        protocol = 'https://'
    else:
        protocol = 'http://'
    threads = [threading.Thread(target=read_url, args=(protocol + config[u'simulationServers'][i] + '/load', i))
               for i in range(len(config[u'simulationServers']))]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    new_availability = False
    for i in range(len(config[u'simulationServers'])):
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
    # port:              local port on which the server is listening.
    # server:            host where this session script is running.
    # sslKey:            private key for a SSL enabled server.
    # sslCertificate:    certificate for a SSL enabled server.
    # simulationServers: lists all the available simulation servers.
    # administrator:     email address of administrator that will receive the notifications.
    # mailServer:        mail server host from which the notifications are sent.
    # mailSender:        email address used to send the notifications.
    # logDir:            directory where the log file is written.
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
    sessionLogDir = os.path.join(config['logDir'], 'session')
    logFile = os.path.join(sessionLogDir, 'output.log')
    try:
        if not os.path.exists(sessionLogDir):
            os.makedirs(sessionLogDir)
        file_handler = logging.FileHandler(logFile)
        formatter = logging.Formatter('%(asctime)-15s [%(levelname)-7s]  %(message)s')
        file_handler.setFormatter(formatter)
        file_handler.setLevel(logging.INFO)
        root_logger.addHandler(file_handler)
    except (OSError, IOError) as e:
        sys.exit("Log file '" + logFile + "' cannot be created: " + str(e))

    simulation_server_loads = [0] * len(config[u'simulationServers'])
    config[u'WEBOTS_HOME'] = os.getenv('WEBOTS_HOME', '../../..').replace('\\', '/')
    if u'administrator' in config:
        if u'mailServer' not in config:
            logging.info("No mail server defined in configuration, disabling e-mail notifications to " +
                         config[u'administrator'] + ".")
            del config[u'administrator']
        elif u'mailSender' not in config:
            logging.info("No mail sender defined in configuration, disabling e-mail notifications to " +
                         config[u'administrator'] + ".")
            del config[u'administrator']
    handlers = []
    handlers.append((r'/session', SessionHandler))
    handlers.append((r'/', ClientWebSocketHandler))
    handlers.append((r'/monitor', MonitorHandler))
    handlers.append((r'/(.*)', tornado.web.StaticFileHandler,
                    {'path': config[u'WEBOTS_HOME'] + '/resources/web/server/www',
                     'default_filename': 'index.html'}))
    application = tornado.web.Application(handlers)
    if u'server' not in config:
        config[u'server'] = 'localhost'
    if u'sslCertificate' in config and u'sslKey' in config:
        config[u'ssl'] = True
        ssl_certificate = os.path.abspath(expand_path(config[u'sslCertificate']))
        ssl_key = os.path.abspath(expand_path(config[u'sslKey']))
        ssl_options = {"certfile": ssl_certificate, "keyfile": ssl_key}
        http_server = tornado.httpserver.HTTPServer(application, ssl_options=ssl_options)
    else:
        config[u'ssl'] = False
        http_server = tornado.httpserver.HTTPServer(application)
    update_load()
    http_server.listen(config[u'port'])
    message = "Session server running on port %d (" % config['port']
    if not config[u'ssl']:
        message += 'no '
    message += 'SSL)'
    logging.info(message)

    try:
        tornado.ioloop.IOLoop.current().start()
    except Exception:
        logging.error(traceback.format_exc() + "\n")
        for client in ClientWebSocketHandler.clients:
            del client


# ensure we are in the script directory
os.chdir(os.path.dirname(os.path.realpath(__file__)))
argc = len(sys.argv)
if argc == 1:
    config_json = 'config/session/default.json'
elif argc == 2:
    config_json = sys.argv[1]
else:
    sys.exit('Too many arguments.')
with open(config_json) as config_file:
    config = json.load(config_file)
start_time = time.strftime("%A, %B %d, %Y at %H:%M:%S")
if __name__ == '__main__':
    main()
