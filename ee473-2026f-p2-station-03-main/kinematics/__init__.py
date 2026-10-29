"""Robot-independent kinematics.

Nothing in this package may contain a dimension specific to one arm. Hardware lives in
robot/widowx.py. Keeping that boundary is what lets the test suite run with no arm attached, and
it is what makes Project 3 an extension of this library rather than a rewrite of it.
"""
