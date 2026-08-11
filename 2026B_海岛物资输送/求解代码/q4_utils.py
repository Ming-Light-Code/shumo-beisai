import numpy as np

NM_TO_M = 1852.0
TAN60 = np.sqrt(3)
SEA_Y_NM = 5.0
SEA_X_NM = 4.0
GRID_RES = 0.02
OVERLAP_MAX = 0.20

def load_data():
    path = r'C:\Users\ming\Desktop\数模备赛\depth_data.npy'
    depth = np.load(path)
    y_coords = np.arange(depth.shape[0]) * GRID_RES
    x_coords = np.arange(depth.shape[1]) * GRID_RES
    return depth, y_coords, x_coords

def bilinear_interp(depth, y_coords, x_coords, y_query, x_query):
    i = int(np.clip(np.searchsorted(y_coords, y_query) - 1, 0, len(y_coords) - 2))
    j = int(np.clip(np.searchsorted(x_coords, x_query) - 1, 0, len(x_coords) - 2))
    y0, y1 = float(y_coords[i]), float(y_coords[i+1])
    x0, x1 = float(x_coords[j]), float(x_coords[j+1])
    wy = (y_query - y0) / (y1 - y0) if y1 != y0 else 0.0
    wx = (x_query - x0) / (x1 - x0) if x1 != x0 else 0.0
    return (1-wy)*(1-wx)*float(depth[i,j]) + (1-wy)*wx*float(depth[i,j+1]) + wy*(1-wx)*float(depth[i+1,j]) + wy*wx*float(depth[i+1,j+1])

def get_strip_half_widths(depth, y_coords, x_coords, y_nm, x_vals):
    return np.array([bilinear_interp(depth, y_coords, x_coords, y_nm, float(x)) for x in x_vals]) * TAN60 / NM_TO_M

print("Utils loaded OK")
