"""Python version of the laser_trace.cpp provided in the skeleton code."""

def map_calc_range(ox, oy, oa, map_width, map_height, map_origin_x, map_origin_y, map_resolution, max_range, map_cells)
	
	x0 = (floor((ox - map_origin_x) / map_resolution + 0.5) + map_width / 2)
	y0 = (floor((oy - map_origin_y) / map_resolution + 0.5) + map_height / 2)
	x1 = (floor(((ox + max_range * cos(oa)) - map_origin_x) / map_resolution + 0.5) + map_width / 2)
	y1 = (floor(((oy + max_range * sin(oa)) - map_origin_y) / map_resolution + 0.5) + map_height / 2)
	
	dx = x1 - x0
    dy = y1 - y0
    
    xsign = 1 if dx > 0 else -1
    ysign = 1 if dy > 0 else -1
    
    dx = abs(dx)
    dy = abs(dy)
    
    if dx > dy:
        xx, xy, yx, yy = xsign, 0, 0, ysign
    else:
    	dx, dy = dy, dx
    	xx, xy, yx, yy = 0, ysign, xsign, 0
    
    D = 2*dy - dx
    y = 0
    
    while x in range(dx + 1):
    	if D >= 0:
    		y += 1
    		D -= 2*dx
    	D += 2*dy
	
	return max_range
	
	
def bresenham(x0, y0, x1, y1):
    """Line from integers (x0, y0) to (x1, y1)"""
    dx = x1 - x0
    dy = y1 - y0
    
    xsign = 1 if dx > 0 else -1
    ysign = 1 if dy > 0 else -1
    
    dx = abs(dx)
    dy = abs(dy)
    
    if dx > dy:
        xx, xy, yx, yy = xsign, 0, 0, ysign
    else:
    	dx, dy = dy, dx
    	xx, xy, yx, yy = 0, ysign, xsign, 0
    
    D = 2*dy - dx
    y = 0
    
    for x in range(dx + 1):
    	yield x0 + x*xx, y*yx, y0 + x*xy, y*yy
    	if D >= 0:
    		y += 1
    		D -= 2*dx
    	D += 2*dy
