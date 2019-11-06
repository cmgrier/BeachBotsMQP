#!/usr/bin/env python

from data.Task import Task


if __name__=="__main__":
    task = Task(120,"avoid")
    assert task.type is "avoid"
    assert task.priority is 1
    assert task.isComplete is False
    assert task.isActive is False
    assert task.zone is 120