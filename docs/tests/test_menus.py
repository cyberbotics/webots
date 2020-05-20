"""Test module of the menu.md files."""
import unittest
from books import Books

import os
import re
import sys


class TestMenus(unittest.TestCase):
    """Unit test of the menus."""

    def setUp(self):
        """Setup."""
        self.menus = []
        books = Books()
        for book in books.books:
            self.menus.append(os.path.join(book.path, 'menu.md'))

    def test_menu_files_exist(self):
        """The menu.md files exist in each book."""
        self.assertGreater(len(self.menus), 0, msg='No menu found')
        for menu in self.menus:
            self.assertTrue(
                os.path.isfile(menu),
                msg='File "%s" does not exist' % (menu)
            )

    def test_menu_refer_valid_files(self):
        """The menu.md refer valid files."""
        for menu in self.menus:
            args = {} if sys.version_info[0] < 3 else {'encoding': 'utf-8'}
            with open(menu, **args) as f:
                content = f.readlines()
            self.assertGreater(len(content), 0, msg='Menu file is empty')
            match_counter = 0
            for line_number in range(0, len(content)):
                line = content[line_number].strip()
                if len(line) == 0:
                    continue
                elif line == '# Table of Contents':
                    continue
                elif line == '# Archive':
                    continue
                else:
                    match = re.match(r'^- \[(.*)\]\((.*)\)$', line)
                    self.assertIsNotNone(
                        match, msg='Line %d of "%s" does not match '
                        'the expected pattern' % (line_number, menu)
                    )
                    for i in [1, 2]:
                        self.assertTrue(
                            match.group(i) and len(match.group(i)) > 0,
                            msg='Line %d of "%s" does not match' %
                            (line_number, menu)
                        )
                    match_counter += 1

                    target_file = os.path.join(
                        os.path.dirname(menu), match.group(2)
                    )
                    self.assertTrue(
                        os.path.isfile(target_file),
                        msg='Invalid reference "%s" in menu "%s"' %
                        (target_file, menu)
                    )
            self.assertGreater(
                match_counter, 0,
                msg='Menu "%s" do not contain valid references' % (menu)
            )

    def test_all_book_files_are_referred_by_the_menu_file(self):
        """All the book files are referred to by the menu files."""
        for menu in self.menus:
            book_path = os.path.dirname(menu)
            # list md files in bookPath
            args = {} if sys.version_info[0] < 3 else {'encoding': 'utf-8'}
            with open(menu, 'r', **args) as menu_file:
                menu_content = menu_file.read()
            for file_path in os.listdir(book_path):
                if (file_path.endswith(".md") and
                        file_path != ('index.md') and
                        file_path != "menu.md"):
                    self.assertIn(
                        file_path, menu_content,
                        msg='"%s" not referenced in "%s"' %
                        (file_path, menu)
                    )
