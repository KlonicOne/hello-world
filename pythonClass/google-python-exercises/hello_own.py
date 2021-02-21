#!/usr/bin/env python

import sys

# Defines a "repeat" function that takes 2 arguments.
def repeat(s, exclaim):
    """
    Returns the string 's' repeated 3 times.
    If exclaim is true, add exclamation marks.
    """

    result = s + s + s # can also use "s * 3" which is faster (Why?)
    if exclaim:
        result = result + '!!!'
    return result

# Gather input in main function
def main():
  print len(sys.argv)
  for s in sys.argv[1:]:
    print 'Hello there', repeat(s, True)
  
if __name__ == '__main__':
  main()
