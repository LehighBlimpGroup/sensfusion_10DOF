import numpy as np

def magDotAccErr(mag,acc,mdg,params):
   #offset and transformation matrix from parameters
   ofs=params[0:3]
   mat=np.reshape(params[3:12],(3,3))
   #subtract offset, then apply transformation matrix
   mc=mag-ofs
   mm=np.dot(mat,mc)
   #calculate dot product from corrected mags
   mdg1=np.dot(mm,acc)
   err=mdg-mdg1
   return err


def analyticPartialRow(mag,acc,target,params):
   err0=magDotAccErr(mag,acc,target,params)
   # ll=len(params)
   slopeArr=np.zeros(12)
   slopeArr[0]=  -(params[3]*acc[0] + params[ 4]*acc[1] + params[ 5]*acc[2])
   slopeArr[1]=  -(params[6]*acc[0] + params[ 7]*acc[1] + params[ 8]*acc[2])
   slopeArr[2]=  -(params[9]*acc[0] + params[10]*acc[1] + params[11]*acc[2])

   slopeArr[ 3]= (mag[0]-params[0])*acc[0]
   slopeArr[ 4]= (mag[1]-params[1])*acc[0]
   slopeArr[ 5]= (mag[2]-params[2])*acc[0]

   slopeArr[ 6]= (mag[0]-params[0])*acc[1]
   slopeArr[ 7]= (mag[1]-params[1])*acc[1]
   slopeArr[ 8]= (mag[2]-params[2])*acc[1]

   slopeArr[ 9]= (mag[0]-params[0])*acc[2]
   slopeArr[10]= (mag[1]-params[1])*acc[2]
   slopeArr[11]= (mag[2]-params[2])*acc[2]

   return (err0,slopeArr)

def numericPartialRow(mag,acc,target,params,step,mode):

   err0=errFn(mag,acc,target,params,mode)

   ll=len(params)
   slopeArr=np.zeros(ll)

   for ix in range(ll):

      params[ix]=params[ix]+step[ix]
      errA=errFn(mag,acc,target,params,mode)
      params[ix]=params[ix]-2.0*step[ix]
      errB=errFn(mag,acc,target,params,mode)
      params[ix]=params[ix]+step[ix]
      slope=(errB-errA)/(2.0*step[ix])
      slopeArr[ix]=slope

   return (err0,slopeArr)


def errFn(mag,acc,target,params,mode):
   if mode == 1: return magDotAccErr(mag,acc,target,params)
   return radiusErr(mag,target,params)

def radiusErr(mag,target,params):
   #offset and transformation matrix from parameters
   (ofs,mat)=param9toOfsMat(params)

   #subtract offset, then apply transformation matrix
   mc=mag-ofs
   mm=np.dot(mat,mc)

   radius = np.sqrt(mm[0]*mm[0] +mm[1]*mm[1] + mm[2]*mm[2] )
   err=target-radius
   return err


def ellipsoid_iterate(mag,accel,verbose):

   magCorrected=copy.deepcopy(mag)
   # Obtain an estimate of the center and radius
   # For an ellipse we estimate the radius to be the average distance
   # of all data points to the center
   (centerE,magR,magSTD)=ellipsoid_estimate2(mag,verbose)

   #Work with normalized data
   magScaled=mag/magR
   centerScaled = centerE/magR

   (accNorm,accR)=normalize3(accel)

   params=np.zeros(12)
   #Use the estimated offsets, but our transformation matrix is unity
   params[0:3]=centerScaled
   mat=np.eye(3)
   params[3:12]=np.reshape(mat,(1,9))

   #initial dot based on centered mag, scaled with average radius
   magCorrected=applyParams12(magScaled,params)
   (avgDot,stdDot)=mgDot(magCorrected,accNorm)

   nSamples=len(magScaled)
   sigma = errorEstimate(magScaled,accNorm,avgDot,params)
   if verbose: print 'Initial Sigma',sigma

   # pre allocate the data.  We do not actually need the entire
   # D matrix if we calculate DTD (a 12x12 matrix) within the sample loop
   # Also DTE (dimension 12) can be calculated on the fly.

   D=np.zeros([nSamples,12])
   E=np.zeros(nSamples)

   #If numeric derivatives are used, this step size works with normalized data.
   step=np.ones(12)
   step/=5000

   #Fixed number of iterations for testing.  In production you check for convergence

   nLoops=5

   for iloop in range(nLoops):
      # Numeric or analytic partials each give the same answer
      for ix in range(nSamples):
         #(f0,pdiff)=numericPartialRow(magScaled[ix],accNorm[ix],avgDot,params,step,1)
         (f0,pdiff)=analyticPartialRow(magScaled[ix],accNorm[ix],avgDot,params)
         E[ix]=f0
         D[ix]=pdiff
      #Use the pseudo-inverse
      DT=D.T
      DTD=np.dot(DT,D)
      DTE=np.dot(DT,E)
      invDTD=inv(DTD)
      deltas=np.dot(invDTD,DTE)


      p2=params + deltas

      sigma = errorEstimate(magScaled,accNorm,avgDot,p2)

      # add some checking here on the behavior of sigma from loop to loop
      # if satisfied, use the new set of parameters for the next iteration

      params=p2

      # recalculste gain (magR) and target dot product
      magCorrected=applyParams12(magScaled,params)
      (mc,mcR)=normalize3(magCorrected)
      (avgDot,stdDot)=mgDot(mc,accNorm)
      magR *= mcR
      magScaled=mag/magR

      if verbose:
         print 'iloop',iloop,'sigma',sigma

   return (params,magR)


# =============================

   

