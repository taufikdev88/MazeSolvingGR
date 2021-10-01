import os

programpath = os.getcwd() + '\program'
tmp = ""
for filename in os.listdir(programpath):
  with open(os.path.join(programpath, filename), 'r') as f:
    tmp += f.readline()
    tmp += '\n'
    f.close()

with open(os.path.join(os.getcwd(), 'fullprogram.txt'), 'w') as f:
  f.write(tmp)
  f.close()