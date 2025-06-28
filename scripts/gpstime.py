# -*- coding: utf-8 -*-

import math as m

# GPS is now ahead of UTC by 18 seconds
GPS_LEAP_SECOND = 18


def unix_second_to_gps_time(seconds):
    second_gps = seconds + GPS_LEAP_SECOND - 315964800
    week = m.floor(second_gps / 604800)
    sow = second_gps - week * 604800
    return week, sow


def gps_time_to_unix_second(week, sow):
    return sow + week * 604800 + 315964800 - GPS_LEAP_SECOND
