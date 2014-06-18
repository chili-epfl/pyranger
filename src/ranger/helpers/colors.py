import colorsys

def clamp(v, minv=0, maxv=1):
    return min(max(minv, v), maxv)

def to_hls(color):
    r, g, b = color
    return colorsys.rgb_to_hls(r/255., g/255., b/255.)

def from_hls(h, l, s):
    r,g,b = colorsys.hls_to_rgb(clamp(h), clamp(l), clamp(s))
    return int(r * 255), int(g * 255), int(b * 255)

BLUE_TO_RED = [(  0,128,255), 
                (23,139,231 ),
                (46,151,208 ),
                (69,162,185 ),
                (92,174,162 ),
                (115,185,139),
                (139,197,115),
                (162,208,92 ),
                (185,220,69 ),
                (208,231,46 ),
                (231,243,23 ),
                (255,255,0  ),
                (255,243,0  ),
                (255,231,0  ),
                (255,220,0  ),
                (255,208,0  ),
                (255,197,0  ),
                (255,185,0  ),
                (255,174,0  ),
                (255,162,0  ),
                (255,151,0  ),
                (255,139,0  ),
                (255,128,0  ),
                (255,112,0  ),
                (255, 96,0  ),
                (255,80,0   ),
                (255,64,0   ),
                (255,48,0   ),
                (255,32,0)]

def get_ramp(ramp = BLUE_TO_RED, robot = None):
    if robot is None:
        return ramp
    else:
        adapted_ramp = []
        for color in ramp:
            h, l, s = to_hls(color)
            adapted_ramp.append(from_hls(h, l * robot.innerstate.energy, s * (robot.innerstate.arousal + 1)))
        return adapted_ramp
