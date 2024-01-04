#! /bin/python
import sys
# generate explicit template instantiation declartions

def output(s):
    sys.stdout.write(s)

def main(class_name, count):
    for i in range(int(count)):
        num = i+1
        output("template class {}<{}>; ".format(class_name, num))
        if num%10 == 0:
            output("\n")

if __name__ == '__main__':
    if len(sys.argv) > 2:
        main(sys.argv[1], sys.argv[2])
    else:
        print("[Error] Please input your class name and count after python xxx.py. For example: python xxx.py class_name 10")

# python generate_template.py class_name 100