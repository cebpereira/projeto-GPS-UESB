from haversine import haversine, Unit

v1 = (-13.8671863, -40.0721722)
v2 = (-13.8655826, -40.0775393)
distance = haversine(v1, v2, unit = 'm')

print(distance)