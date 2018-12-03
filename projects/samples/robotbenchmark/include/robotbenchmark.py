"""Utility functions to communicate with the robotbenchmark server."""

import os
import sys


def robotbenchmarkRecord(answer, benchmark, record):
    """Return the record info retrieved from the robotbenchmark server."""
    answer = answer[7:]  # remove the "record:" header
    if ':' in answer:
        userData = answer.split(':')
        user = 'email=' + userData[0] + '&password=' + userData[1]
    else:
        user = 'owner=' + answer
    outFile = 'record.txt'
    try:
        hostFile = open("../../host.txt", 'r')
    except IOError:
        sys.stderr.write("Error: cannot open host file.\n")
        return
    host = hostFile.read()
    hostFile.close()
    # Compute the Fully Qualified Domain Name (fqdn) from the host:
    # E.g., https://my.domain.com:8443 => my.domain.com
    if host.startswith('https://'):
        fqdn = host[8:]
    else:  # assuming 'http://'
        fqdn = host[7:]
    n = fqdn.find(':')
    if n > 0:
        fqdn = fqdn[:n]
    WEBOTS_HOME = os.environ["WEBOTS_HOME"]
    try:
        keyFile = open(WEBOTS_HOME + "/resources/web/server/key/" + fqdn, "r")
    except IOError:
        sys.stderr.write("Error: cannot open key file.\n")
        return
    key = keyFile.readline()
    key = key.rstrip(os.linesep)
    keyFile.close()

    cmd = """wget -q --output-document %s """ \
          """--post-data "%s&"""\
          """record=%f&benchmark=%s&key=%s" %s/record.php""" \
          % (outFile, user, record, benchmark, key, host)
    os.system(cmd)
    recordFile = open(outFile)
    record = recordFile.readline()
    recordFile.close()
    os.remove(outFile)
    return 'record:' + record
