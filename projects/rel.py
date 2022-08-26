import glob

for file in glob.glob('**/*.proto', recursive=True):
    unix_file = file.replace('\\', '/')
    search = '"webots://projects/' + unix_file[:unix_file.rfind('/') + 1]
    with open(file, 'r') as f:
        data = f.read()
    new_data = data.replace(search, '"')
    if new_data != data:
        print(unix_file)
        with open(file, 'w', newline='\n') as f:
            f.write(new_data)