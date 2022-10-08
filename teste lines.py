from matplotlib import pyplot
import numpy, weakref

a = numpy.arange(int(1e3))
fig = pyplot.Figure()
ax  = fig.add_subplot(1, 1, 1)
lines = ax.plot(a)

l = lines.pop(0)
wl = weakref.ref(l)  # create a weak reference to see if references still exist
#                      to this object
print(wl) # not dead
l.remove()
print(wl)  # not dead
del l
print(wl) # dead  (remove either of the steps above and this is still live)