#!/usr/bin/env python
from support.EqualPriorityQueue import EqualPriorityQueue as EPQ


if __name__=="__main__":
    queue = EPQ()

    queue.put(3, "Hello")
    queue.put(1, "Test")
    assert queue.queue == [(1, "Test"), (3, "Hello")]

    queue.remove(1)
    assert queue.queue == [(3, "Hello")]

    test = queue.get()
    assert test == "Hello"


