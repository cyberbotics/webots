#!/usr/bin/env python

import rospy


def main():
    """TODO."""
    rospy.init_node('webots', disable_signals=True)

    paused = rospy.get_param("~paused", "")
    print(paused)
    print("----")


if __name__ == '__main__':
    main()
