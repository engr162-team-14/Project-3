import inspect

"""This provides a lineno() function to make it easy to grab the line
number that we're on. Danny Yoo (dyoo@hkn.eecs.berkeley.edu)
"""
def lineno():
    """Returns the current line number in our program."""
    return inspect.currentframe().f_back.f_lineno

def mu_run(test_func):
    line_fail = test_func()
    if line_fail == -1:
        print("\nPassed: %s" % (test_func.__name__))
    else:
        print("\nFailed: %s at line %d" % (test_func.__name__, line_fail))

def mu_check(condition, line_fail_arr, line):
    if not condition and line_fail_arr[0] == -1:
        line_fail_arr[0] = line