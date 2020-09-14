import sys
filename = sys.argv[1]

f = open (filename, "r")
fw = open (filename+".padded", "w")
print (filename)

for line in f:
    item = line.strip().split(",")
    result = item[0][:-6]+"."+ item[0][-6:] +" "+item[1]+" "+item[2]+" "+item[3]+" "+"0 0 0 1\n"
    fw.write(result)