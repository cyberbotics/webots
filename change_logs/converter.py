from lxml import html
import sys

with open('ChangeLog.html', 'r') as file:
    fileContent = file.read()
    fileContent = fileContent.replace('<font color="red"><b>', '**').replace('</b></font>', '**')
    fileContent = fileContent.replace('<b><font color="red">', '**').replace('</font></b>', '**')
    fileContent = fileContent.replace('<font color="red">', '**').replace('</font>', '**')
    fileContent = fileContent.replace('<br>', ' ').replace('<break>', ' ').replace('</tt>', '__').replace('<tt>', '__')
    tree = html.fromstring(fileContent)
    body = tree[1]
    content = ''
    for child in body:
        if child.tag == 'h1':
            if child.text:
                content += '# %s\n\n' % child.text
            else:  # links
                sys.exit('h1 tag failure')
        elif child.tag == 'h2':
            if child.text:
                content += '\n## %s\n\n' % child.text
            else:  # links
                for link in child:
                    if link.tag == 'a':
                        content += '\n## [%s](%s)\n\n' % (link.text, link.attrib['href'])
                    else:
                        sys.exit('h2 tag failure')
        elif child.tag == 'ul':
            for child1 in child:
                if child1.tag == 'li':
                    if child1.text:
                        content += '  - %s\n' % child1.text.replace('\'', '`')
                    else:
                        for child2 in child1:
                            if child2.tag == 'b':
                                content += '  - %s\n' % child2.text.replace('\'', '`')
                            else:
                                sys.exit('failure 2 ' + child2.tag)
                elif child1.tag == 'ul':
                    for child2 in child1:
                        if child2.tag == 'li':
                            content += '    - %s\n' % child2.text.replace('\'', '`')
                        else:
                            for child3 in child2:
                                if child3.tag == 'li':
                                    content += '      - %s\n' % child3.text.replace('\'', '`')
                                elif child3.tag == 'ul':
                                    for child4 in child3:
                                        if child4.tag == 'li':
                                            content += '        - %s\n' % child4.text.replace('\'', '`')
                                        else:
                                            sys.exit('failure 6 ' + child3.tag)
                                else:
                                    sys.exit('failure 3 ' + child3.tag)
                else:
                    sys.exit('failure 4 ' + child1.tag + ' ' + child1.text)
        else:
            sys.exit('failure 5 ' + child.tag + ' ' + child.text)
    print(content)
