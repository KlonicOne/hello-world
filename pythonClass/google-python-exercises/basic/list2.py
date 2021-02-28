#!/usr/bin/python -tt
# Copyright 2010 Google Inc.
# Licensed under the Apache License, Version 2.0
# http://www.apache.org/licenses/LICENSE-2.0

# Google's Python Class
# http://code.google.com/edu/languages/google-python-class/

# Additional basic list exercises

# D. Given a list of numbers, return a list where
# all adjacent == elements have been reduced to a single element,
# so [1, 2, 2, 3] returns [1, 2, 3]. You may create a new list or
# modify the passed in list.
def remove_adjacent(nums):
    ret_value_list = []
    # sort list
    nums.sort()
    # only take each value once in return
    for i in range(len(nums)):
        # check if that we are not on last element
        if i < (len(nums) - 1):
            # num is smaller then next, we append in return value list
            if nums[i] < nums[i + 1]:
                ret_value_list.append(nums[i])
            else:
                continue
        else:
            # take over last element
            ret_value_list.append(nums[i])
    # list returned with eliminated doubles
    return ret_value_list


# E. Given two lists sorted in increasing order, create and return a merged
# list of all the elements in sorted order. You may modify the passed in lists.
# Ideally, the solution should work in "linear" time, making a single
# pass of both lists.
def linear_merge(list1, list2):
    ret_value_list = list1 + list2
    ret_value_list.sort()
    return ret_value_list

def merge_remove_adjacent(in_list1, in_list2):
    merge_list = linear_merge(in_list1, in_list2)
    ret_list = remove_adjacent(merge_list)
    return ret_list


# Note: the solution above is kind of cute, but unforunately list.pop(0)
# is not constant time with the standard python list implementation, so
# the above is not strictly linear time.
# An alternate approach uses pop(-1) to remove the endmost elements
# from each list, building a solution list which is backwards.
# Then use reversed() to put the result back in the correct order. That
# solution works in linear time, but is more ugly.


# Simple provided test() function used in main() to print
# what each function returns vs. what it's supposed to return.
def test(got, expected):
    if got == expected:
        prefix = ' OK '
    else:
        prefix = '  X '
    print '%s got: %s expected: %s' % (prefix, repr(got), repr(expected))


# Calls the above functions with interesting inputs.
def main():
    print 'remove_adjacent'
    test(remove_adjacent([1, 2, 2, 3, 3, 3, 3]), [1, 2, 3])
    test(remove_adjacent([2, 2, 3, 3, 3]), [2, 3])
    test(remove_adjacent([]), [])

    print
    print 'linear_merge'
    test(linear_merge(['aa', 'xx', 'zz'], ['bb', 'cc']),
         ['aa', 'bb', 'cc', 'xx', 'zz'])
    test(linear_merge(['aa', 'xx'], ['bb', 'cc', 'zz']),
         ['aa', 'bb', 'cc', 'xx', 'zz'])
    test(linear_merge(['aa', 'aa'], ['aa', 'bb', 'bb']),
         ['aa', 'aa', 'aa', 'bb', 'bb'])

    print
    print 'merge and remove adjacent'
    test(merge_remove_adjacent(['aa', 'aa', 'cc'], ['aa', 'bb', 'bb']),
         ['aa', 'bb', 'cc'])

if __name__ == '__main__':
    main()
