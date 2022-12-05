from collections import deque

qX = deque(maxlen=2)

for i in range(0, 10):
    qX.append(i)

print(qX)
print(qX[0])
print(qX[-1])
print(2*3)

