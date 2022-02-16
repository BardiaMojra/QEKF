
import os

srcDir = './'

def main():

  for i, file in enumerate(os.listdir(srcDir)):
    #print(i, file)
    newfile = file
    newfile = newfile.replace(' ', '_')
    newfile = newfile.replace('&', 'n')
    newfile = newfile.replace(',', '')
    newfile = newfile.replace('and', 'n')
    newfile = newfile.replace("'", '')
    newfile = newfile.replace('-', '_')
    newfile = newfile.replace('__', '_')
    newfile = newfile.replace('__', '_')
    src = srcDir+file
    dest = srcDir+newfile
    os.rename(src,dest)
    #print(i, file)

  print('files renamed... ')
  for i, file in enumerate(os.listdir(srcDir)):
    print(i, file)


if __name__ == '__main__':

  main()


# EOF
