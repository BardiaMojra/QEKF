import numpy as np
import scipy as sc #
import scipy.linalg as lalg
from scipy.spatial.transform import Rotation as sc_R



''' NBUG libraries '''
from nbug import *
from pdb import set_trace as st

''' private libraries '''
from quest import *
from utils import *

def ransac_QuEst_h(m1,m2,model,max_iters,threshold,min_inliers,nbug=True,\
                   return_all=True,_dtype=np.float128):
  """fit model parameters to data using the RANSAC algorithm
    This implementation written from pseudocode found at
    http://en.wikipedia.org/w/index.php?title=RANSAC&oldid=116358182
    input:
      data - a set of observed data points
      model - a model that can be fitted to data points
      datum_min - min number of data values required to fit the model
      iters_max  - max number of iterations allowed in the algorithm
      threshold - threshold value for determining when a data point fits a model
      inliers_min - the number of close data values required to assert that a model fits well to data
    return:
      bestfit - model parameters which best fit the data """
  st()
  data = np.concatenate((m1,m2), axis=1, dtype=_dtype)
  iterations = 0
  bestfit = None
  besterr = np.inf
  best_inlier_idxs = None
  for i in range(max_iters):
    #todo explore ways to get points with SOME distance from each other
    #todo test enforcing min distance between correspondences
    maybe_idxs, test_idxs = random_partition(min_inliers,m1.shape[0])

    maybeinliers = data[maybe_idxs,:]
    test_points = data[test_idxs]
    maybemodel = model.fit(maybeinliers)
    test_err = model.get_error( test_points, maybemodel)
    also_idxs = test_idxs[test_err < threshold] # select indices of rows with accepted points
    alsoinliers = data[also_idxs,:]
    if nbug:
      print ('test_err.min()'+test_err.min())
      print ('test_err.max()'+test_err.max())
      print ('np.mean(test_err)'+np.mean(test_err))
      print ('iteration: {}', iterations)
      print('len(alsoinliers): {}', len(alsoinliers))
      print('\n\n')
    if len(alsoinliers) > min_inliers:
      betterdata = np.concatenate( (maybeinliers, alsoinliers) )
      bettermodel = model.fit(betterdata)
      better_errs = model.get_error( betterdata, bettermodel)
      thiserr = np.mean( better_errs )
      if thiserr < besterr:
        bestfit = bettermodel
        besterr = thiserr
        best_inlier_idxs = np.concatenate( (maybe_idxs, also_idxs) )
    iterations+=1
  if bestfit is None:
    raise ValueError("did not meet fit acceptance criteria")
  if return_all:
    return bestfit, {'inliers':best_inlier_idxs}
  else:
    return bestfit


''' private routines '''
def random_partition(n,n_data):
  #todo try loading sparse random data points
  """return n random rows of data (and also the other len(data)-n rows)"""
  all_idxs = np.arange( n_data )
  np.random.shuffle(all_idxs)
  idxs1 = all_idxs[:n]
  idxs2 = all_idxs[n:]
  return idxs1, idxs2
class QuEst_SampsonDist_Model:
  """QuEst_RANSAC estimates the pose between two camera views using RANSAC and
  QuEst algorithm. """
  def __init__(self, kps1, kps2, debug=False):
    self.kps1 = kps1 # inputs X
    self.kps2 = kps2 # outputs Y
    self.debug = debug
  def fit(self,m1,m2,_dtype=np.float128):
    ''' pose estimation: estimates the relative rotation and translation
    between two camera views using the 5 point quaternion algorithm.
    Input:
      x1, x2 - Two sets of N matched feature point coordinates. x1, x2
               are 3xN matrices with each column corresponding to a point.
      x      - If a single argument is supplied it is assumed that it
               is in the form x = [x1; x2]
    Output:
      M      - A structure that contains the estimated pose.
    Copyright (c) 2016, Kaveh Fathian. The University of Texas at Dallas. '''
    # A = np.vstack([data[:,i] for i in self.input_columns]).T
    # B = np.vstack([data[:,i] for i in self.output_columns]).T
    # npprint('A',A)
    # npprint('',A)
    # x,resids,rank,s = scipy.linalg.lstsq(A,B) # replace with QuEst
    # est pose with QuEst
    qs = QuEst(m=m1, n=m2)
    npprint('qs', qs)
    st()
    # pose = QuEst_Ver1_1(A,B) # run QuEst algorithm
    # find the best solution
    qs_residues = get_residues(m1,m2,qs) # Scoring function
    npprint('qs', qs)
    npprint('qs_residues', qs_residues)
    npprint('idx', idx)
    npprint('qs[idx]', qs[idx])

    idx = np.amin(qs_residues[:])
    # resMin, mIdx = min(abs(qs_residues))
    q = qs[idx]
    t, dep1, dep2, res = get_Txyz(m1,m2,q)
    # t = pose.T(:,mIdx);

    # make a fundamental matrix from the recovered rotation and translation
    q_arr = np.asarray([q.w,q.x,q.y,q.z], dtype=_dtype)
    R = sc_R.from_quat(q_arr).as_matrix()


    npprint('t',t)
    st()

    Tx = skew(t/np.norm(t))
    npprint('Tx',Tx)
    st()
    F = Tx @ R
    npprint('F',F)
    st()

    return {'q':q, 't':t, 'm1':m1, 'm2':m2, 'F':F}


  def get_error(self,m1,m2,F):
    ''' use Sampson distance to compute the first order approximation
    geometric error of the fitted fundamental matrix given a set of matched
    correspondences '''

    # todo working here


    B_fit = m1 ,model
    err_per_point = np.sum((B-B_fit)**2,axis=1) # sum squared error per row


x1 = x(1:3,:);    % Extract x1 and x2 from x
x2 = x(4:6,:);

F = M.F;

if iscell(F)        % We have several solutions each of which must be tested
    nF = length(F); % Number of solutions to test
    bestM = M;
    bestM.F = F{1}; % Initial allocation of best solution
    ninliers = 0;   % Number of inliers
    for k = 1 : nF
        x2tFx1 = zeros(1,length(x1));
        for n = 1:length(x1)
            x2tFx1(n) = x2(:,n)'*F{k}*x1(:,n);
        end

        Fx1  = F{k}*x1;
        Ftx2 = F{k}'*x2;

        % Evaluate distances
        d =  x2tFx1.^2 ./ ...
         (Fx1(1,:).^2 + Fx1(2,:).^2 + Ftx2(1,:).^2 + Ftx2(2,:).^2);
        inliers = find(abs(d) < t);     % Indices of inlying points
        if length(inliers) > ninliers   % Record best solution
          ninliers = length(inliers);
          bestM.F = F{k};
          bestInliers = inliers;
        end
    end
else     % We just have one solution
    x2tFx1 = zeros(1,length(x1));
    for n = 1:length(x1)
        x2tFx1(n) = x2(:,n)'*F*x1(:,n);
    end

    Fx1  = F*x1;
    Ftx2 = F'*x2;

    % Evaluate distances
    d =  x2tFx1.^2 ./ ...
         (Fx1(1,:).^2 + Fx1(2,:).^2 + Ftx2(1,:).^2 + Ftx2(2,:).^2);

    bestInliers = find(abs(d) < t);     % Indices of inlying points
    bestM = M;                          % Copy M directly to bestM

end

end







    return err_per_point

def test():
  # generate perfect input data
  n_samples = 500
  n_inputs = 1
  n_outputs = 1
  A_exact = 20*np.random.random((n_samples,n_inputs) )
  perfect_fit = 60*np.random.normal(size=(n_inputs,n_outputs) ) # the model
  B_exact = np.dot(A_exact,perfect_fit)
  assert B_exact.shape == (n_samples,n_outputs)
  # add a little gaussian noise (linear least squares alone should handle this well)
  A_noisy = A_exact + np.random.normal(size=A_exact.shape )
  B_noisy = B_exact + np.random.normal(size=B_exact.shape )
  if 1:
    # add some outliers
    n_outliers = 100
    all_idxs = np.arange( A_noisy.shape[0] )
    np.random.shuffle(all_idxs)
    outlier_idxs = all_idxs[:n_outliers]
    non_outlier_idxs = all_idxs[n_outliers:]
    A_noisy[outlier_idxs] =  20*np.random.random((n_outliers,n_inputs) )
    B_noisy[outlier_idxs] = 50*np.random.normal(size=(n_outliers,n_outputs) )
  # setup model


  all_data = np.hstack( (A_noisy,B_noisy) )
  input_columns = range(n_inputs) # the first columns of the array

  output_columns = [n_inputs+i for i in range(n_outputs)] # the last columns of the array

  debug = False

  model = LinearLeastSquaresModel(input_columns,output_columns,debug=debug)

  linear_fit,resids,rank,s = scipy.linalg.lstsq(all_data[:,input_columns],
                                                all_data[:,output_columns])

  # run RANSAC algorithm

  ransac_fit, ransac_data = ransac(all_data,model,
                                    50, 1000, 7e3, 300, # misc. parameters
                                    debug=debug,return_all=True)
  if 1:
    import pylab
    sort_idxs = np.argsort(A_exact[:,0])
    A_col0_sorted = A_exact[sort_idxs] # maintain as rank-2 array
    if 1:
        pylab.plot( A_noisy[:,0], B_noisy[:,0], 'k.', label='data' )
        pylab.plot( A_noisy[ransac_data['inliers'],0], B_noisy[ransac_data['inliers'],0], 'bx', label='RANSAC data' )
    else:
        pylab.plot( A_noisy[non_outlier_idxs,0], B_noisy[non_outlier_idxs,0], 'k.', label='noisy data' )
        pylab.plot( A_noisy[outlier_idxs,0], B_noisy[outlier_idxs,0], 'r.', label='outlier data' )
    pylab.plot( A_col0_sorted[:,0],
                np.dot(A_col0_sorted,ransac_fit)[:,0],
                label='RANSAC fit' )
    pylab.plot( A_col0_sorted[:,0],
                np.dot(A_col0_sorted,perfect_fit)[:,0],
                label='exact system' )
    pylab.plot( A_col0_sorted[:,0],
                np.dot(A_col0_sorted,linear_fit)[:,0],
                label='linear fit' )
    pylab.legend()
    pylab.show()
