from PIL import Image


class RegularImage:
    @classmethod
    def load_from_file(cls, filename):
        """Parse an image."""
        ri = RegularImage()
        ri.im = Image.open(filename)
        ri.width, ri.height = ri.im.size
        return ri

    @classmethod
    def create_black_image(cls, width, height):
        """Create an black image."""
        ri = RegularImage()
        ri.im = Image.new('RGB', (width, height))
        ri.width = width
        ri.height = height
        return ri

    def __init__(self):
        """Constructor: simply reset the fields."""
        self.width = -1
        self.height = -1
        self.im = None

    def is_valid(self):
        return self.width * self.height > 0 and self.im is not None

    def get_pixel(self, x, y):
        assert x >= 0 and x < self.width
        assert y >= 0 and y < self.height
        return self.im.getpixel((x, y))

    def set_pixel(self, x, y, pixel):
        assert x >= 0 and x < self.width
        assert y >= 0 and y < self.height
        self.im.putpixel((x, y), (int(pixel[0]), int(pixel[1]), int(pixel[2])))

    def save(self, filename):
        """Save the image to a file."""
        assert self.is_valid()
        self.im.save(filename)

    def to_pil(self):
        """PIL image getter."""
        assert self.is_valid()
        return self.im
