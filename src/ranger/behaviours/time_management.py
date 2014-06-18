import datetime # for timedelta
import time # for timestamps

from robots.helpers.helpers import enum

# 1 -> physical time
# 3600 -> 1 real second correspond to 1 hour
TIME_ACCELERATION = 1
INIT_TIME = datetime.datetime.now()

PERIODS = enum(MORNING=[datetime.time(9, 30), datetime.time(11, 30)],
                AFTERNOON=[datetime.time(14, 00), datetime.time(16, 00)])

# discrete time periods (moving time windows), in secs
IMMEDIATE=10
CURRENT=60 # last minute
RECENT=60*5

# discrete time periods (semantic)
PROXIMAL=-1 # within an activity period, min(time since period start, 15 minutes)
ACTIVITY_PERIOD=-2 # since the period start
DAY=-3 # since the start of the MORNING period

def get_current_time():
    return (INIT_TIME + (datetime.datetime.now() - INIT_TIME) * TIME_ACCELERATION).time()

def delta_seconds(time1, time2):
    date1 = datetime.datetime.combine(datetime.datetime.today(), time1)
    date2 = datetime.datetime.combine(datetime.datetime.today(), time2)
    return (date2 - date1).total_seconds()

def get_current_period(now):
    period = None
    if PERIODS.MORNING[0] < now < PERIODS.MORNING[1]:
        period = PERIODS.MORNING
    elif PERIODS.AFTERNOON[0] < now < PERIODS.AFTERNOON[1]:
        period = PERIODS.AFTERNOON

    return period


def time_elapsed_in(timezone):
    if timezone in [IMMEDIATE, CURRENT, RECENT]:
        return timezone

    now = get_current_time()
    period = get_current_period(now)

    if timezone == PROXIMAL:
        if period is None:
            return None
        return min(15 * 60, delta_seconds(period[0], now))
    elif timezone == ACTIVITY_PERIOD:
        if period is None:
            return None
        return delta_seconds(period[0], now)
    elif timezone == DAY:
        if now < PERIODS.MORNING[0]:
            return None
        return delta_seconds(PERIODS.MORNING[0], now)

    raise RuntimeError("shouldn't be here...")

def ratio_time_elapsed_in(timezone):
    if timezone in [IMMEDIATE, CURRENT, RECENT, PROXIMAL]:
        raise RuntimeError("ratio_time_elapsed_in does not mean anything for a moving time window!")

    now = datetime.datetime.now().time()
    secs = time_elapsed_in(timezone)

    if timezone == ACTIVITY_PERIOD:
        period = get_current_period(now)
        return float(secs) / delta_seconds(period[1], period[0])
    elif timezone == DAY:
        return float(secs) / delta_seconds(PERIODS.AFTERNOON[1], PERIODS.MORNING[0])

    raise RuntimeError("shouldn't be here...")
