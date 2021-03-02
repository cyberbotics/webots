import os
import re
from pathlib import Path


def find_path(path, url, proto):
    if os.path.isfile(path + url):
        return 'webots://' + path + url
    path = 'projects/default/worlds/'
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
    search = re.compile('(\\"[\\w,\\s,\\/]+\\.(?:png|jpg|hdr|PNG|JPG|HDR|jpeg|JPEG)\\")')
    result = search.findall(content)
    if len(result) != 0:
        for url in result:
            print(file + ': ' + str(url))
            found = find_path(path, url[1:-1], proto)
            if not found:
                print('Could not find ' + url + ' in ' + file)
                continue
            content = content.replace(url, '"' + found + '"')
            # print('Replaced ' + url + ' with "' + found + '" in ' + file)
    with open(file, 'w', newline='\n') as fd:
        fd.write(content)
    return


for path in Path('.').rglob('*.wbt'):  # replace with '*.wbt' for world files
    path = str(path).replace('\\', '/')
    # print(path)
    search(path)
    # replace_url(path)
