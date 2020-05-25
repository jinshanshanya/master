import numpy as np
import pandas as pd
from math import *
#from pyproj import Proj, transform
#from decimal import localcontext
#from decimal import Decimal

pi = np.pi
#B0 = 31.3027612492
#L0 = 121.291513755

def transformation(lat, lon, B0, L0):
    B = lat * pi / 180
    L = lon * pi / 180
    B0 = B0 * pi / 180
    L0 = L0 * pi / 180
    a = 6378137
    b = 6356752.3142
    e = sqrt((1 - (b / a) ** 2))
    ee = sqrt(((a / b) ** 2 - 1))
    K = ((a ** 2 * cos(B0)) / b) / sqrt(1 + (ee) ** 2 * (cos(B0)) ** 2)  # constant
    X = K * (L - L0)
    Y0 = K * log(tan(pi / 4 + B0 / 2) * ((1 - e * sin(B0)) / (1 + e * sin(B0))) ** (e / 2))
    Y = K * log(tan(pi / 4 + B / 2) * ((1 - e * sin(B)) / (1 + e * sin(B))) ** (e / 2)) - Y0
    return X, Y


# with localcontext() as ctx:
#     ctx.prec = 11
#def python_trans(lat, lon):
#   inProj = Proj(init='epsg:3857')
#  outProj = Proj(init='epsg:4326')
#    x2, y2 = transform(inProj, outProj, lat, lon)
#    # print(x2[0], y2[0])
#    return x2, y2


point = pd.read_csv('waypoint2.csv')
with pd.option_context('display.precision', 12):
    # print(point)
    B0 = np.double(point.loc[0, ['Lat']])
    L0 = np.double(point.loc[0, ['Lon']])
    # print(B0, L0)
    for i in range(len(point)):
        waypoint = point.loc[i, ["Lat", "Lon", "Heading"]].values.astype(float)
        # print(waypoint)
        # print(waypoint[[1]])
        lat = np.double(waypoint[[0]])
        lon = np.double(waypoint[[1]])
        x, y = transformation(lat, lon, B0, L0)
        # print(lat, lon)
        # func(lat, lon)
        # x, y = python_trans(lat, lon)
        line = "%s,%s,%s" % (x, y, waypoint[[2]][0])
        # print(line)
        with open("b.csv", 'a+') as f:
            f.write(line + '\n')
