import cv2
import os

subdir = '/image_1'
srcDir = os.path.abspath(os.getcwd())+subdir

for i, file in enumerate(os.listdir(srcDir)):
  fpath = os.path.join(srcDir, file)
  image = cv2.imread(fpath)
  gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
  cv2.imwrite(fpath, gray)

print("\n\n --->> end of process... \n\n")
