import glob
import os


def count_lines(filename):
    f = open(filename, 'rb')
    lines = 0
    buf_size = 1024 * 1024
    read_f = f.raw.read
    buf = read_f(buf_size)
    while buf:
        lines += buf.count(b'\n')
        buf = read_f(buf_size)
    return lines


def count_files(extensions, name):
    line_counter = 0
    file_counter = 0
    if isinstance(extensions, str):
        extensions = [extensions]
    for extension in extensions:
        files = glob.glob(os.path.join('..', '..', '**', '*.' + extension), recursive=True)
        file_counter += len(files)
        for f in files:
            line_counter += count_lines(f)
    print((name + ':').ljust(12) + str(line_counter).ljust(6) + ' lines of codes in ' + str(file_counter).ljust(4) + ' files')


count_files(['cpp', 'hpp'], 'C++')
count_files(['c', 'h'], 'C')
count_files(['py'], 'Python')
count_files('js', 'JavaScript')
count_files('java', 'Java')
count_files('m', 'MATLAB')
count_files('vert', 'GLSL')
