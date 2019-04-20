import os

directory = "/home/js0823/Downloads/custom_photos"

def convert_filename():
  i = 1
  for filename in os.listdir(directory):
    dst = "custom" + str(i) + ".jpg"
    src = directory + '/' + filename
    dst = directory + '/' + dst

    os.rename(src, dst)
    i += 1
  
def convert_resolution():
  for filename in os.listdir(directory):
    dst = directory + '/' + filename
    os.system("convert -resize 512X384 {0} {1}".format(dst, dst))

if __name__ == '__main__':
    convert_resolution()
