#!/usr/bin/env python

import glob, os

dataset_path = '/home/js0823/github-repo/trashbot/data/images'

percentage_test = 5

file_train = open('train.txt', 'w')
file_test = open('test.txt', 'w')

counter = 1
index_test = round(100 / percentage_test)

for pathAndFilename in glob.iglob(os.path.join(dataset_path, "*.jpg")):
    title, ext = os.path.splitext(os.path.basename(pathAndFilename))

    if counter == index_test + 1:
        counter = 1
        file_test.write(dataset_path + "/" + title + '.jpg' + "\n")
    else:
        file_train.write(dataset_path + "/" + title + '.jpg' + "\n")
        counter = counter + 1
