'''
@Author: Lin Sinan
@Github: https://github.com/linsinan1995
@Email: mynameisxiaou@gmail.com
@LastEditors: Lin Sinan
@Description: 
         
'''

def parse():
    line = input().strip().split()
    return list(map(int, line))


if __name__ == "__main__":
    nNode,nEdge,s,t = parse()
    print(nNode,nEdge,s-1,t-1)

    for i in range(nEdge):
        s, t, cap, cost = parse()
        if s == 0 or t == 0: 
            raise Exception("data format error!\n")
        print(s-1, t-1, cap, cost)