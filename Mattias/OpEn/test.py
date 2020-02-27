
import math

(x,y)=(1,1)

r=math.degrees(math.atan2(y,x))
(x,y)=(-1,-1.2)
e=math.degrees(math.atan2(y,x))


dae=e if e>=0 else 180+abs(180+e)
dar=r if r>=0 else 180+abs(180+r)
print('r',r)
print('e',e)
print('dar',dar)

print('dae',dae)
print('abs',abs(dar-dae))

if abs(dar-dae)<=180:
    print('rutn cloxkwise')
else:
    print('rutn anticloxkwise')
