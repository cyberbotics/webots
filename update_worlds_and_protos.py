import os
import re
from pathlib import Path


def find_path(path, url, proto):
    if os.path.isfile(path + url):
        return 'webots://' + path + url
    paths = ['projects/default/worlds/', 'projects/objects/traffic/protos/', 'projects/objects/road/protos/',
             'projects/robots/softbank/nao/protos/', 'projects/samples/contests/ratslife/protos/e-puck/',
             'projects/samples/contests/tower_of_hanoi/protos/', 'projects/objects/kitchen/breakfast/protos/',
             'projects/objects/trees/protos/', 'projects/objects/advertising_board/protos/',
             'projects/vehicles/protos/abstract/', 'projects/vehicles/protos/', 'projects/vehicles/protos/citroen/']
    for path in paths:
        if os.path.isfile(path + url):
            return 'webots://' + path + url
    return False


def replace_url(file):
    proto = True if file[:-6] == '.proto' else False
    path = '/'.join(file.split('/')[0:-1]) + '/'
    with open(file, 'r') as fd:
        content = fd.read()
    start = 0
    keys = ['url', 'backUrl', 'bottomUrl', 'frontUrl', 'leftUrl', 'rightUrl', 'topUrl',
            'backIrradianceUrl', 'bottomIrradianceUrl', 'frontIrradianceUrl',
            'leftIrradianceUrl', 'rightIrradianceUrl', 'topIrradianceUrl',
            'sound', 'noiseMaskUrl', 'bumpSound', 'rollSound', 'slideSound', 'texture']
    for key in keys:
        while True:
            start = content.find('  ' + key + ' ', start)
            if start == -1:
                break
            start_quote = content.find('"', start + 1)
            end_quote = content.find('"', start_quote + 1)
            url = content[start_quote + 1:end_quote]
            start = end_quote + 1
            if url[:8] == 'https://' or url[:9] == 'webots://':
                continue
            found = find_path(path, url, proto)
            if not found:
                print('Could not find ' + url + ' in ' + file)
                continue
            content = content.replace('"' + url + '"', '"' + found + '"')
            # print('Replaced ' + url + ' with ' + found + ' in ' + file)
    with open(file, 'w', newline='\n') as fd:
        fd.write(content)
    return


def search(file):
    proto = True if file[:-6] == '.proto' else False
    path = '/'.join(file.split('/')[0:-1]) + '/'
    with open(file, 'r') as fd:
        content = fd.read()
    search = re.compile('(\\"[\\w,\\s,\\/]+\\.(?:png|jpg|hdr|obj|wav)\\")')
    result = search.findall(content)
    if len(result) != 0:
        for url in result:
            # print(file + ': ' + str(url))
            found = find_path(path, url[1:-1], proto)
            if not found:
                print('Could not find ' + url + ' in ' + file)
                continue
            content = content.replace(url, '"' + found + '"')
            # print('Replaced ' + url + ' with "' + found + '" in ' + file)
    with open(file, 'w', newline='\n') as fd:
        fd.write(content)
    return


skipped = ['SimpleBuilding', 'PublicToilet', 'DirectionPanel', 'SimpleTree', 'ProtoMismatchFieldType']

for path in Path('.').rglob('*.proto'):  # replace with '*.wbt' for world files
    path = str(path).replace('\\', '/')
    proto = os.path.splitext(os.path.basename(path))[0]
    if proto in skipped:
        continue
    search(path)
