import sys
filename = sys.argv[1]

f = open (filename, "r")
fw = open (filename+".noangle", "w")
print (filename)

for line in f:
    item = line.strip().split(" ")
    result = "{} {} {} {} {} {} {} {}\n".format(
        item[0], item[1], item[2], item[3],
        0, 0, 0, 1
    )
    fw.write(result)