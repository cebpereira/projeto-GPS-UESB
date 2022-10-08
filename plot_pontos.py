import matplotlib.pyplot as plt
import sys
import re

file_name = "map.osm.txt"
x = list()
y = list()
with open(file_name) as fp:
    for line in fp:
        points = re.findall(r'[-+]?\d+.\d+', line)
        x.append(float(points[1]))
        y.append(float(points[2]))
print(points)
plt.plot(x, y, 'ro')

def on_click(event):
    if event.button == 3:
        x, y = event.xdata, event.ydata
        print(x, y)

plt.connect('button_press_event', on_click)

plt.show()