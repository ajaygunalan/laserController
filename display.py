from collections import deque

qX = deque(maxlen=10)

for i in range(1, 11):
    qX.append(i)

print(qX)
print(sum(qX)/len(qX))


