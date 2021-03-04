from pathlib import Path


def replace_url(file, version):
    with open(file, 'r') as fd:
        content = fd.read()
    content = content.replace('webots://', 'https://raw.githubusercontent.com/cyberbotics/webots/' + version + '/')
    with open(file, 'w', newline='\n') as fd:
        fd.write(content)
    return


for path in Path('.').rglob('*.proto'):
    replace_url(path, 'R2021a')
