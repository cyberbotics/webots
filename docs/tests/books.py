"""Get the books paths."""
import os


class Book:
    """Helper class to retrieve the book paths."""

    def __init__(self, path, name):
        """Constructor. Initialize the properties."""
        self._path = path
        self._name = name
        self._md_paths = []
        for filename in os.listdir(path):
            if not filename.endswith('.md'):
                continue
            md_path = os.path.join(path, filename)
            self._md_paths.append(md_path)

    @property
    def path(self):
        """Hold the path property."""
        return self._path

    @property
    def name(self):
        """Hold the name property."""
        return self._name

    @property
    def md_paths(self):
        """Hold the MD paths."""
        return self._md_paths


class Books:
    """Helper class to creat/retrieve the books."""

    def __init__(self):
        """Constructor. Initialize the properties."""
        self._project_path = os.path.abspath(os.path.join(
            os.path.dirname(os.path.realpath(__file__)), os.pardir))
        # discover the project path
        self._books = []
        for bookName in ["guide", "reference", "blog", "discord", "automobile"]:
            path = os.path.join(self._project_path, bookName)
            self._books.append(Book(path, bookName))

    @property
    def project_path(self):
        """Hold the path containing the books."""
        return self._project_path

    @property
    def books(self):
        """Hold the book names."""
        return self._books
