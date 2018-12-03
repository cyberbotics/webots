# -*- coding: utf-8 -*-
"""Test module of the MD files."""
import unittest
from books import Books

import os


class TestMDFiles(unittest.TestCase):
    """Unit test of the MD files."""

    def test_md_files_exist(self):
        """Test that the MD files exist."""
        books = Books()
        self.assertGreater(len(books.books), 0, msg='No books found')
        for book in books.books:
            self.assertGreater(
                len(book.md_paths), 0,
                msg='No MD files found in book "%s"' % book
            )
            for md_filename in book.md_paths:
                self.assertTrue(
                    os.path.isfile(md_filename),
                    msg='MD file "%s" is not existing' % (md_filename)
                )
                with open(md_filename) as f:
                    content = f.read()
                self.assertGreater(
                    len(content), 0,
                    msg='MD file "%s" is empty' % (md_filename)
                )

    def test_md_filenames_dont_contain_prohibited_keywords(self):
        """Test that the MD filenames don't contain prohibited keywords."""
        books = Books()
        for book in books.books:
            # 'advertising' causes issues with ad blocking software.
            self.assertFalse(
                'advertising' in book.md_paths,
                msg='MD file "%s" contains "advertising"' % (book.md_paths)
            )

    def test_md_file_contents_dont_contain_prohibited_keywords(self):
        """Test that the MD file contents don't contain prohibited keywords."""
        books = Books()
        for book in books.books:
            for md_path in book.md_paths:
                with open(md_path) as f:
                    content = f.readlines()
                for line in content:
                    # π crashes QtWebEngine on Windows.
                    self.assertFalse(
                        'π' in line,
                        msg='MD file "%s" contains "π"' % (md_path)
                    )
