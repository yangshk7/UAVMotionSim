import sys

from uavsim import __version__


def test_environment():
    assert sys.version_info >= (3, 11)
    assert __version__
