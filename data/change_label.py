#!/usr/bin/env python

import os

directory = '/home/js0823/github-repo/darknet/trashnet/data/labels'

for filename in os.listdir(directory):
  if filename.endswith(".txt"):
    with open(os.path.join(directory, filename)) as fileobj:
      line = fileobj.readline()
    print(os.path.join(directory, filename))
    print(line)
    #line = '0' + line[1:]
    #with open(os.path.join(directory, filename), 'w') as fileobj:
    #  fileobj.write(line)
