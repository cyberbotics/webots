"""Helper functions to handle easily Python data structures."""


def grouper(n, L):
    """Group a flat list into a list of tuples of size n."""
    # src: http://stackoverflow.com/questions/1624883/alternative-way-to-split-a-list-into-groups-of-n
    return list(zip(*[L[i::n] for i in range(n)]))
