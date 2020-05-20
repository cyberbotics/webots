"""Test module of the images."""
import unittest
from books import Books

import fnmatch
import os
import re
import sys


class TestImages(unittest.TestCase):
    """Unit test of the images."""

    def test_images_are_valid(self):
        """Test that the MD files refer to valid URLs."""
        books = Books()
        for book in books.books:
            for md_path in book.md_paths:
                args = {} if sys.version_info[0] < 3 else {'encoding': 'utf-8'}
                with open(md_path, **args) as f:
                    content = f.read()
                for match in re.finditer(r"!\[(.*?)\]\((.*?)\)", content):
                    # remove parameters
                    is_youtube_video = match.group(1) == "youtube video"
                    image_ref = match.group(2).split(' ')[0]
                    if not is_youtube_video and not image_ref.startswith('http'):
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
                for filename in fnmatch.filter(filenames, '*.png') + fnmatch.filter(filenames, '*.jpg'):
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
                    args = {} if sys.version_info[0] < 3 else {'encoding': 'utf-8'}
                    with open(md_path, **args) as file:
                        if (image_path in file.read() or
                                image_path.replace('.png', '.thumbnail.jpg') in images_paths or
                                image_path.replace('.png', '.thumbnail.png') in images_paths):
                            found = True
                            break
                self.assertTrue(
                    found, msg='Image "%s" not referenced in any MD file.' % image_path
                )
                # in case of thumbnail make sure the original file is available
                if image_path.endswith('.thumbnail.jpg'):
                    self.assertTrue(
                        image_path.replace('.thumbnail.jpg', '.png') in images_paths,
                        msg='Missing original file for thumbnail "%s".' % image_path
                    )


if __name__ == '__main__':
    unittest.main()
