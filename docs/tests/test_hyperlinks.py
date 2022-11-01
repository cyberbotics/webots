"""Test module for the hyperlinks."""

import unittest
import re
import os
import sys

from books import Books


class TestHyperlinks(unittest.TestCase):
    """Unit test of the hyperlinks."""

    def setUp(self):
        """Setup: get all the hyperlinks."""
        self.hyperlinks = []

        books = Books()
        for book in books.books:

            # we don't want to maintain links posted on Discord
            if book.name == 'discord':
                continue

            for md_path in book.md_paths:
                # Extract MD content.
                args = {} if sys.version_info[0] < 3 else {'encoding': 'utf-8'}
                with open(md_path, **args) as f:
                    content = f.read()
                # Remove code statements
                content = re.sub(r'```.+?(?=```)```', '', content, flags=re.S)
                content = re.sub(r'`.+?(?=`)`', '', content, flags=re.S)
                # Remove charts
                content = re.sub(r'%chart.+?(?=%end)%end', '\n', content, flags=re.S)
                # Extract hyperlinks.
                for m in re.finditer(r'[^\!](\[([^\]]*)\]\s*\(([^\)]*)\))', content):
                    hyperlinkMD = m.group(1)
                    hyperlinkName = m.group(2)
                    hyperlinkUrl = m.group(3)
                    self.hyperlinks.append({
                        'md': hyperlinkMD,
                        'name': hyperlinkName,
                        'url': hyperlinkUrl,
                        'file': md_path
                    })
        # # Debug: Uncomment to display all the acquired hyperlinks.
        # for h in self.hyperlinks:
        #     print (h)

    def test_underscores_in_hyperlinks_are_protected(self):
        """Test that every underscores appearing in hyperlinks are protected."""
        for h in self.hyperlinks:
            self.assertTrue(
                re.search(r'[^\\]_', h['name']) is None,
                msg='Hyperlink "%s" contains unprotected underscore(s) in "%s".' % (h['md'], h['file'])
            )

    def test_hyperlinks_do_not_contain_prohibited_characters(self):
        """Test that hyperlinks are not containing prohibited characters (such as '<')."""
        for h in self.hyperlinks:
            self.assertTrue(
                re.search(r'[<>]', h['name']) is None,
                msg='Hyperlink "%s" contains forbidden characters in "%s".' % (h['md'], h['file'])
            )

    def test_tag_hyperlinks(self):
        """Test that a tag-like hyperlinks are valid."""
        for h in self.hyperlinks:
            if h['name'] in ['C++', 'Java', 'Python', 'ROS', 'MATLAB']:
                self.assertTrue(
                    '.md' in h['url'],
                    msg='Hyperlink "%s" is wrongly detected as a tag in "%s".' % (h['md'], h['file'])
                )

    def test_github_file_exists(self):
        """Test that the github file pointed by the link exists."""
        for h in self.hyperlinks:
            if h['url'].startswith('https://github.com/cyberbotics/webots/tree/released'):
                path = h['url'].replace('https://github.com/cyberbotics/webots/tree/released',
                                        os.path.normpath(os.environ['WEBOTS_HOME']))
                self.assertTrue(
                    os.path.isfile(path) or os.path.isdir(path),
                    msg='Hyperlink "%s" is pointing to a non-existing file or directory "%s" (in file "%s").' %
                        (h['md'], path, h['file'])
                )
