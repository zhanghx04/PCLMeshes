from ntpath import join
import os
import sys

folder = "./checkpoint/"

sub_folder = sys.argv[1]

sub_folder = sub_folder[2:-4]

path = os.path.join(folder, sub_folder)

os.mkdir(path)
