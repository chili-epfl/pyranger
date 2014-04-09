
# 'big' SHARP IR sensor
GP2Y0A02YK = [#  m   -> val
                [0   ,3000],
                [0.1 ,2522],
                [0.15,3070],
                [0.2 ,2841],
                [0.5 ,1456],
                [0.6 ,1233],
                [0.8 , 950],
                [1   , 774],
                [1.5 , 585],
                [2   , 467],
                [2.1 ,   0]]

# 'small' SHARP IR sensors
GP2Y0A41SK0F = [
                [0    ,3100],
                [0.025,3050],
                [0.08 ,1550],
                [0.14 , 920],
                [0.245, 540],
                [0.4  , 300],
                [0.5  ,   0]]

SCALE = [
          [0, 200],
          [1, 1450],
          [25, 36480]
        ]

def linear_interpolation(val, refs):
    negative = (refs[0][1] > refs[-1][1])

    if (negative and val > refs[0][1]) or (not negative and val < refs[0][1]):
        return refs[0][0]

    pd, pv = refs[0]

    for d, v in refs:
        if (negative and val > v) or (not negative and val < v):
            a = float(pd - d) / (pv - v)
            b = float(d * pv - v * pd) / (pv -v)
            return a * val + b
        pd, pv = d, v

    return refs[-1][0]


