# Source: https://gist.github.com/nickv2002/459e4dfd94ae2b221c7d
import numpy as np
import pylab as p
from scipy import ndimage
from collections import deque


queueSize = 10

# Initializing a queue
q = deque(maxlen=queueSize)

# Adding elements to a queue
q.append('a')
q.append('b')
q.append('c')

print("Initial queue")
print(q)

# Removing elements from a queue
print("\nElements dequeued from the queue")
print(q.popleft())
print(q.popleft())
print(q.popleft())

print("\nQueue after removing elements")
print(q)

# Uncommenting q.popleft()
# will raise an IndexError
# as queue is now empty


if __name__ == '__main__':
    def test ():
        x = np.linspace (0, 1, 102)
        x[3::10] = 1.5
        p.plot (x)
        result = ndimage.median_filter(x, size=3)
        p.plot(result)
        p.show ()

    test ()
