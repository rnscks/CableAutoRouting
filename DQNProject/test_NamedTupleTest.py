import unittest
from collections import namedtuple  

Point = namedtuple('Point',['x', 'y'])

class TestNamedTupleCreation(unittest.TestCase):
    def test_namedtuple_creation(self):
        p = Point(11, y=2)
        self.assertEqual(p.x, 11)
        self.assertEqual(p.y, 2)
        print("NamedTuple creation test passed!")
