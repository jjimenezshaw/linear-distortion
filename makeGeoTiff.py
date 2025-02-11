import pyproj
from osgeo import gdal, ogr, osr
import numpy as np
import math
import os

crss = {}


def getUtmZone(longitude):
    zone = int(math.floor((longitude + 180.0) / 6.0) + 1.0)
    return zone


def getCrs(longitude):
    zone = getUtmZone(longitude)
    s = f"+proj=utm +zone={zone} +ellps=GRS80 +units=m +no_defs +type=crs"
    #s = f"EPSG:258{zone}"
    if s not in crss:
        crss[s] = pyproj.Proj(s)
    return crss[s]


def getLinearFactor(lon, lat):
    crs = getCrs(lon)
    factors = crs.get_factors(lon, lat)
    return factors.meridional_scale


def getElevationFactor(lat, elevation):
    a = 6378137
    es = 0.00669437999014
    # e = 0.081819190842622
    sinlat = math.sin(lat * math.pi / 180)
    r = a * math.sqrt(1 - es) / (1 - es * sinlat * sinlat)
    return r / (r + elevation)


dmsfile = "ellipsoidal.tif"

from osgeo import gdal

ds = gdal.Open(dmsfile, gdal.GA_ReadOnly)

transform = ds.GetGeoTransform()
nodata = ds.GetRasterBand(1).GetNoDataValue()


def pixel2coord(x, y):
    xoff, a, b, yoff, d, e = transform
    xp = a * x + b * y + a * 0.5 + b * 0.5 + xoff
    yp = d * x + e * y + d * 0.5 + e * 0.5 + yoff
    return (xp, yp)


rb = ds.GetRasterBand(1)
xsize = ds.RasterXSize
ysize = ds.RasterYSize
img_array = rb.ReadAsArray()


driver = gdal.GetDriverByName("GTiff")
dst_filename = os.path.join(".", "output_out.tif")
dst_ds = driver.Create(
    dst_filename, xsize=xsize, ysize=ysize, bands=1, eType=gdal.GDT_Float32
)

dst_ds.SetGeoTransform(transform)
dst_ds.SetProjection(ds.GetProjection())
dst_ds.GetRasterBand(1).SetNoDataValue(nodata)

raster = np.zeros((ysize, xsize), dtype=np.float32)
count = 0
for row in range(0, ysize):
    print(row, ysize)
    elem = []
    for col in range(0, xsize):
        coord = pixel2coord(col, row)
        elevation = img_array[row][col]
        if elevation == nodata:
            d = nodata
        else:
            f = getLinearFactor(coord[0], coord[1])
            elevationFactor = getElevationFactor(coord[1], elevation)
            d = (f * elevationFactor - 1) * 1e6  # ppm
        elem.append(d)
    raster[count] = elem
    count += 1

dst_ds.GetRasterBand(1).WriteArray(raster)
