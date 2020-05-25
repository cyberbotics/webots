"""Test module for the Markdown items."""

import unittest
import re
import sys

from books import Books


class TestLists(unittest.TestCase):
    """Unit test of the Markdown items."""

    hyperlinkRE = re.compile(r'\!?\[([^\]]*)\]\s*\(([^\)]*)\)')
    hyperlinkStartRE = re.compile(r'^\!?\[([^\]]*)\]\s*\(([^\)]*)\)')

    def setUp(self):
        """Setup: get all the items."""
        self.items = []
        books = Books()
        for book in books.books:

            # we are not responsible of the content of the discord chats
            if book.name == 'discord':
                continue

            for md_path in book.md_paths:
                # Extract MD content.
                itemBuffer = ''
                skipUntil = ''
                args = {} if sys.version_info[0] < 3 else {'encoding': 'utf-8'}
                with open(md_path, **args) as f:
                    content = f.readlines()
                for line in content:
                    if skipUntil:
                        if skipUntil in line:
                            skipUntil = ''
                        else:
                            continue
                    if itemBuffer and not line.strip():
                        self.items.append({'item': itemBuffer, 'md': md_path})
                        itemBuffer = ''
                    if re.match(r'\s*```', line):
                        skipUntil = '```'
                        continue
                    elif re.match(r'^\s*- ', line) or re.match(r'^\s*\d+\. ', line):
                        if itemBuffer:
                            self.items.append({'item': itemBuffer, 'md': md_path})
                            itemBuffer = ''
                        itemBuffer += line
                    elif itemBuffer:
                        itemBuffer += line
                if itemBuffer:
                    self.items.append({'item': itemBuffer, 'md': md_path})

        # Debug: Uncomment to display all the acquired items.
        # for i in self.items:
        #     print ('@@@')
        #     print (i['item'])

    def test_one_sentence_per_line(self):
        """Test that each sentence is written on one line."""
        for p in self.items:
            lines = p['item'].split('\n')
            if len(lines) <= 2:
                # Test only the items having 2 lines or more, because they are for sure sentences.
                continue
            for line in lines:
                line = re.sub(r'^\s*- ', '', line)  # Remove item prefix.
                line = re.sub(r'^\s*\d+\. ', '', line)  # Remove number prefix.
                if re.match(TestLists.hyperlinkStartRE, line):  # line starts with an hyperlink.
                    continue
                line = re.sub(TestLists.hyperlinkRE, '', line)  # Remove hyperlinks.
                line = line.strip()
                if not line:  # If it remains something, then test it.
                    continue
                self.assertTrue(
                    line.endswith('.') or line.endswith(':') or line.endswith('!'),
                    msg='"%s": The following line does not end correctly: "%s"' % (p['md'], line)
                )
                self.assertFalse(
                    re.match(r'^[a-z]', line),
                    msg='"%s": The following line is starting with a lower case: "%s"' % (p['md'], line)
                )
