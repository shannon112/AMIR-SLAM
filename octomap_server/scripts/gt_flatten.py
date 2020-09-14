import sys
filename = sys.argv[1]

f = open (filename, "r")
fw = open (filename+".flatten", "w")
print (filename)

for line in f:
    item = line.strip().split(" ")
    result = "{} {} {} {} {} {} {} {}\n".format(
        item[0], item[1], item[2], 0,
        item[4], item[5], item[6], item[7]
    )
    fw.write(result)