"""Test module for the Markdown paragraphs."""

import re
import sys
import unittest

from books import Books


class TestParagraphs(unittest.TestCase):
    """Unit test of the Markdown lists."""

    def setUp(self):
        """Setup: get all the paragraphs."""
        self.paragraphs = []
        books = Books()
        for book in books.books:

            # we are not responsible of the content of the discord chats
            if book.name == 'discord':
                continue

            for md_path in book.md_paths:
                # Extract MD content.
                args = {} if sys.version_info[0] < 3 else {'encoding': 'utf-8'}
                with open(md_path, **args) as f:
                    content = f.read()

                # Remove annoying string sequences.
                # - Multiline code sections.
                content = re.sub(r'```.+?(?=```)```', '\n', content, flags=re.S)
                content = re.sub(r'\n`.+?(?=`\n)`\n', '\n', content, flags=re.S)
                # - Notes.
                content = re.sub(r'\n\s*>.+?(?=\n\n)', '\n', content, flags=re.S)
                content = re.sub(r'\n\s*>.+?(?=\n$)', '\n', content, flags=re.S)
                # - Showdown extensions.
                content = re.sub(r'%chart.+?(?=%end)%end', '\n', content, flags=re.S)
                content = re.sub(r'%figure.+?(?=%end)%end', '\n', content, flags=re.S)
                content = re.sub(r'%api.+?(?=%end)%end', '\n', content, flags=re.S)
                content = re.sub(r'%spoiler.+?(?=%end)%end', '\n', content, flags=re.S)
                content = re.sub(r'%tab-component.+?(?=%end)%end', '\n', content, flags=re.S)
                content = re.sub(r'%robot.*\n', '\n', content, flags=re.S)
                # - Headers.
                content = re.sub(r'^#.*', '\n', content)
                content = re.sub(r'\n#.*', '\n', content)
                # - Items.
                content = re.sub(r'\n\s*-.+?(?=\n\n)', '\n', content, flags=re.S)
                content = re.sub(r'\n\s*-.+?(?=\n$)', '\n', content, flags=re.S)
                content = re.sub(r'\n\s*\d+\..+?(?=\n\n)', '\n', content, flags=re.S)
                content = re.sub(r'\n\s*\d+\..+?(?=\n$)', '\n', content, flags=re.S)
                content = re.sub(r'\n    .+?(?=\n)', '\n', content, flags=re.S)
                content = re.sub(r'\n        .+?(?=\n)', '\n', content, flags=re.S)
                # - HTML statements
                for _ in range(10):
                    previous_content = content
                    content = re.sub(r'\n *<.+?>\n', '\n', content, flags=re.S)
                    if previous_content == content:
                        break
                content = re.sub(r'\n---\n', '\n', content, flags=re.S)
                # - Single hyperlinks
                content = re.sub(r'\n\!?\[.+\)\n', '\n', content, flags=re.S)
                # - Special statements
                content = re.sub(r'\nRelease {{.+?}}\n', '\n', content, flags=re.S)
                content = re.sub(r'\n\s*\*\*.+?\n', '\n', content, flags=re.S)
                content = re.sub(r'\n\s*\*.+?\*\n', '\n', content, flags=re.S)
                content = re.sub(r'\n\s*\{.+?\n', '\n', content, flags=re.S)

                # Extract paragraphs.
                for match in re.finditer(r'(?s)((?:[^\n][\n]?)+)', content):
                    paragraph = content[match.start():match.end() - 1]
                    # - Arrays.
                    if paragraph.startswith('| ') or paragraph.startswith('> '):
                        continue
                    self.paragraphs.append({'paragraph': paragraph, 'md': md_path})
        # Debug: Uncomment to display all the acquired paragraphs.
        # for p in self.paragraphs:
        #     print ('@@@')
        #     print (p)

    def test_one_sentence_per_line(self):
        """Test that each sentence is written on one line."""
        for p in self.paragraphs:
            lines = p['paragraph'].split('\n')
            for line in lines:
                line = line.strip()
                if not line or line == '&nbsp;':
                    continue
                self.assertTrue(
                    line.endswith('.') or line.endswith(':') or line.endswith('!') or line.endswith('?'),
                    msg='"%s": The following line does not end correctly: "%s"' % (p['md'], line)
                )
                self.assertFalse(
                    re.match(r'^[a-z]', line),
                    msg='"%s": The following line is starting with a lower case: "%s"' % (p['md'], line)
                )
