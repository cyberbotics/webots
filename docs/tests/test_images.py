"""Test module of the images."""
import unittest
from books import Books

import fnmatch
import os
import re


class TestImages(unittest.TestCase):
    """Unit test of the images."""

    def test_images_are_valid(self):
        """Test that the MD files refer to valid URLs."""
        books = Books()
        for book in books.books:
            for md_path in book.md_paths:
                with open(md_path) as f:
                    content = f.read()
                for match in re.finditer(r"!\[(.*?)\]\((.*?)\)", content):
                    # remove parameters
                    is_youtube_video = match.group(1) == "youtube video"
                    if not is_youtube_video:
                        image_ref = match.group(2).split(' ')[0]
                        image_path = os.path.join(book.path, image_ref)
                        self.assertTrue(
                            os.path.isfile(image_path),
                            msg='%s: "%s" not found' % (md_path, image_path)
                        )

    def test_all_images_are_used(self):
        """Test that all the image files are referenced somewhere."""
        books = Books()
        for book in books.books:
            # search for all images
            images_paths = []  # ['image/sonar.png', 'image/sphere.png', ...]
            for root, dirnames, filenames in os.walk(book.path):
                if 'scenes' in root.replace(books.project_path, ''):
                    continue
                for filename in fnmatch.filter(filenames, '*.png'):
                    image_path = os.path.join(root, filename)
                    image_path = image_path[(len(book.path) + 1):]
                    images_paths.append(image_path.replace('\\', '/'))
            self.assertGreater(
                len(images_paths), 0,
                msg='No image found in book "%s"' % book.name
            )

            # check the image reference can be found in at least one MD file
            for image_path in images_paths:
                found = False
                for md_path in book.md_paths:
                    if image_path in open(md_path).read():
                        found = True
                        break
                self.assertTrue(
                    found, msg='Image "%s" not referenced in any MD file.' % image_path
                )
