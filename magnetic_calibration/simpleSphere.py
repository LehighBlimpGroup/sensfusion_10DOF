
import numpy as np

#least squares fit to a sphere

def ls_sphere(xx,yy,zz):
   asize = np.size(xx)
   print('Sphere input size is ' + str(asize))
   J=np.zeros((asize,4))
   ABC=np.zeros(asize)
   K=np.zeros(asize)

   for ix in range(asize):
      x=xx[ix]
      y=yy[ix]
      z=zz[ix]

      J[ix,0]=x*x + y*y + z*z
      J[ix,1]=x
      J[ix,2]=y
      J[ix,3]=z
      K[ix]=1.0

   K=K.transpose() #not required here
   JT=J.transpose()
   JTJ = np.dot(JT,J)
   InvJTJ=np.linalg.inv(JTJ)

   ABC= np.dot(InvJTJ, np.dot(JT,K))
   #If A is negative, R will be negative
   A=ABC[0]
   B=ABC[1]
   C=ABC[2]
   D=ABC[3]

   xofs=-B/(2*A)
   yofs=-C/(2*A)
   zofs=-D/(2*A)
   R=np.sqrt(4*A + B*B + C*C + D*D)/(2*A)
   if R < 0.0: R = -R
   return (xofs,yofs,zofs,R)

if __name__ == '__main__':
# Test of least squares fit to a Sphere
# Samples have random noise added to X, Y, and Z
# True center is at (40,80,120); true radius is 400

# Output:
# Sphere input size is 9
# Center at (    43.5,    79.8,   123.3) Radius:   401.2


   xyz = np.array([
[    44.5,  472.7,   14.4 ],
[    35.5,  182.9,  504.9 ],
[    36.3, -312.9,  225.8 ],
[    43.4, -263.2,  -81.7 ],
[  -150.3,   78.0, -219.2 ],
[   388.7,   74.6,  -85.4 ],
[   142.3,   80.4, -265.2 ],
[  -347.3,  -24.2,  117.9 ],
[   428.3,  184.5,  125.7 ]
        ])

   xx=xyz[:,0]
   yy=xyz[:,1]
   zz=xyz[:,2]
   ans = ls_sphere(xx,yy,zz)

   ss="Center at (%8.1f,%8.1f,%8.1f) Radius:%8.1f" % ans
   print(ss)