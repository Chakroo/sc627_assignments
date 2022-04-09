from math import atan2, pi, cos, sin, sqrt
from numpy import multiply, any

# Given three collinear points p, q, r, the function checks if point q lies on line segment 'pr'
def onSegment(p, q, r):
    if (q[0] <= max(p[0], r[0]) and q[1] >= min(p[1], r[0]) and q[1] <= max(p[1], r[1]) and q[1] >= min(p[1], r[1])):
        OS = 1
    else:
        OS = 0
    return OS

# finds the orientation of p, q and r points on cartesian plane.O=0 for collinear, O=1 for clockwise and O=2 for counterclockwise
def orientation(p, q, r):
    val = ((q[1] - p[1]) * (r[0] - q[0])) - ((q[0] - p[0]) * (r[1] - q[1]))
    if (val == 0):
        O = 0  # Collinear
    elif(val > 0):
        O = 1  # Clockwise
    else:
        O = 2  # CounterClockwise
    return O

# finds whether the cartesian line segment p1q1 intersects p2q2. At Intersection I = 1, else I = 0
def doIntersect(p1, q1, p2, q2):
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)
    # General case
    if (o1 != o2 and o3 != o4):
        I = 1
    # Special Cases. p1, q1 and p2 are collinear and p2 lies on segment p1q1
    elif (o1 == 0 and onSegment(p1, p2, q1)):
        I = 1
    # p1, q1 and q2 are collinear and q2 lies on segment p1q1
    elif (o2 == 0 and onSegment(p1, q2, q1)):
        I = 1
    # p2, q2 and p1 are collinear and p1 lies on segment p2q2
    elif (o3 == 0 and onSegment(p2, p1, q2)):
        I = 1
    # p2, q2 and q1 are collinear and q1 lies on segment p2q2
    elif (o4 == 0 and onSegment(p2, q1, q2)):
        I = 1
    else:
        I = 0
    return I

#finds whether the cartesian point p = [px,py] lies in the cartesian matrix P = [ax,ay; bx,by;...] and finds the position of the matrix
def computePointExistInArray(p,P):
    l = len(P)
    for i in range(l):
        if P[i][0]==p[0]:
            if P[i][1]==p[1]:
                Index = i
                Exists = True
                break
            else:
                continue
        else:
            continue
        i = i + 1
    return [Exists,Index]

#calculates slope of line between two cartesian coordinates p1=[x1,y1] and p2=[x2,y2]
def computeSlopeBetweenTwoPoints(p1,p2):
    a1=p2[0]-p1[0]
    b1=p2[1]-p1[1]
    m = b1/a1
    return m

#calculates distance between two cartesian coordinates p1=[x1,y1] and p2=[x2,y2]
def computeDistBetweenTwoPoints(p1,p2):
    a1= p2[0]-p1[0]
    b1= p2[1]-p1[1]
    d = sqrt(a1**2 + b1**2)
    return d

#calculates unit vector of line between two cartesian coordinates p1=[x1,y1] and p2=[x2,y2]
def computeVectorBetweenTwoPoints(p1,p2):
    a1= p2[0]-p1[0]
    b1= p2[1]-p1[1]
    d = computeDistBetweenTwoPoints(p1,p2)
    V = [a1/d,b1/d]
    return V

#calculates ax+by+c = 0 form for two cartesian points p1=[x1,y1] and p2=[x2,y2]
def computeLineThroughTwoPoints(p1,p2):
    x1 = p1[0]
    y1 = p1[1]
    x2 = p2[0]
    y2 = p2[1]
    a1 = y1-y2
    b1 = x2-x1
    c1 = x1*y2 - y1*x2
    s = sqrt(a1**2 + b1**2)
    a = a1 / s
    b = b1 / s
    c = c1 / s
    return [a,b,c]

# calculates distance d of point q = [x3,y3] to line between p1 and p2 calculates Distplacement vector V from line to point
def computeDistancePointToLine(q,p1,p2):
    x3 = q[0]
    y3 = q[1]
    [a,b,c] = computeLineThroughTwoPoints(p1,p2)
    d = (abs(a*x3 + b*y3 + c))/sqrt(a^2 + b^2)
    return d

#calculates distance d of point q = [x3,y3] to segment p1 = [x1,y1] and p2 = [x2,y2] 
#calculates w where w = 0 if the segment point closest to q is strictly inside the segment, 
#w = 1 if the closest point is p1, and w = 2 if the closest point is p2 calculates v = vector from segment to point q
def computeDistancePointToSegment(q,p1,p2):
    v1 = p2-p1 # AB
    v2 = q-p1 # AE
    v3 = q-p2 # BE
    
    D1 = multiply(v1,v2)
    S1 = D1[0]+D1[1] # P1 P2 . P1 Q ; dot product of these two vectors
    D2 = multiply(v1,v3)
    S2 = D2[0]+D2[1] # P1 P2 . P2 Q ; dot product of these two vectors
    if S2 > 0:
        w = 2  #point closest to P2 point
        d = computeDistBetweenTwoPoints(p2,q)
        v = computeVectorBetweenTwoPoints(p2,q)
    elif S1 < 0:
        w = 1  #point closest to P1 point
        d = computeDistBetweenTwoPoints(p1,q)
        v = computeVectorBetweenTwoPoints(p1,q)
    else:
         w=0   #point strictly inside the segment
         d = computeDistancePointToLine(q,p1,p2)
         vv=computeVectorBetweenTwoPoints(p1,p2)
         v[0]=vv[1]
         v[1]=-vv[0]
    vx = v[0]
    vy = v[1]
    return [d,w,vx,vy]

# calculates minimum distance between polygon P (described by vertices in
# matrix form)and point Q
# calculates w where w = 0 if the segment point closest to q is strictly
# inside the segment, w = 1 if the closest point is p1, 
# and w = 2 if the closest point is p2
# calculates v = vector to closest point
# calculates point p closest to q
def computeDistancePointToPolygon(P,q):
    n = len(P)

    distance = [0]*n
    w = [0]*n
    vx = [0]*n
    vy= [0]*n

    for i in range(n):
        if i==n:
            j=1
        else:
            j=i+1
        ar1 = [P[i][0], P[i][1]]
        ar2 = [P[j][0], P[j][1]]
        [distance[i],w[i],vx[i],vy[i]] = computeDistancePointToSegment(q,ar1,ar2)
    
    D = min(distance)
    [Exists,I]=computePointExistInArray(D,distance)
    wid = w[I]
    vec = [vx[I],vy[I]]

    if I==n-1:
        J=1
    else:
        J=I+1

    if wid==1:
        p = [P[I][0], P[I][1]]
    else:
        p = [P[J][0], P[J][1]]

    return [D,wid,vec,p]

#unit vector is given by the cartesian coordiante of a point in direction of the vector from origin.
def computeTangentVectorToPolygon(P,q,Clockwise):
    [D,wid,vec,p] = computeDistancePointToPolygon(P,q)
    if Clockwise==1:
        TVx = vec[1]
        TVy = -vec[0]
    else:
        TVx = -vec[1]
        TVy = vec[0]
   
    return [TVx,TVy]