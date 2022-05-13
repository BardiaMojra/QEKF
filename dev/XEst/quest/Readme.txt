A Benchmarking Package for Relative 
Pose Estimation Algorithms



Run “Batch_Benchmark.m” in MATLAB to benchmark camera pose estimation algorithms
* 8-point algorithm
* Kneip (5-point)
* Kukelova (5-point)
* Nister (5-point)
* Li & Hartley (5-point)
* Stewenius (5-point)
* QuEst (5-point)

on real world image datasets 
* KITTI
* TUM 
* ICL
* NAIST.

We have only included the first 10 images from each dataset. This is sufficient to run the code and see how algorithms can be benchmarked.  You need to download the full image datasets from the links below (they’re free, but you may need to create an account):

KITTI: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
TUM: https://vision.in.tum.de/data/datasets/rgbd-dataset
ICL: https://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html
NAIST: http://ypcex.naist.jp/trakmark/sequences/naist.package02/index.html#sequence

For each dataset, copy the sequence images to their associated locations in the “Datasets” folder. You can then run “Batch_Benchmark.m” to get the full benchmark results.


Notes:
Several algorithms come in the “.mexw64” format. You need to run the code on a 64-bit Windows machine, otherwise, you will get an error. If you do not have access to such a machine, try calling the algorithms with their “.m” format in the code. You should be able to replace “.mexw64” with “.m” for all algorithms except for the Kneip’s and Li’s algorithms.

This package is tested on MATLAB R2016b, 64-bit. MATLAB’s Computer Vision System Toolbox and Statistics and Machine Learning Toolbox are required.


We have included the results that we got after running the benchmark in the folder “Benchmark Results”. You can run “Table_Benchmark.m” to generate an Excel file with the averaged estimation errors for all sequences in each dataset. Note that the results you get from benchmarking the algorithms may differ slightly from ours due to the random nature of RANSAC.


For any questions or comments please email:
Kaveh Fathian: kaveh.fathian@utdallas.edu,  kaveh.fathian@gmail.com
Pablo Ramirez: jpi.ramirez@ugto.mx
