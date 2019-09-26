// General preprocessing
1) take all points, grid them into 1x1 pixels
2) In each pixel, retain the bottom-most point
3) Delete points which have high curvature (probably vegetation) -> fine DEM

// Coarse Scarring
4) Repeat above but in 15x15m pixels -> coarse DEM
5) Remove points from fine DEM which differ too much from coarse DEM
6) Interpolate to fill holes

// Clustering
4) Run Euclidean clustering on fine DEM
5a) Keep only the first cluster (probably the ground)
5b) OR keep all clusters above some threshold size (A > A_biggest-house)
6) Iterate through removed points and re-attach points which are BELOW the DEM

// Conditional Region-Growing
4) Run Conditional Growing on the DEM
5) Do not allow new points which differ too greatly in curvature, distance, or normal direction from the seed

// Plane SAC-ish
4) At each pixel, fit a plane to all neighbor pixels 




I should write a region-growing implementation for rasters...
How much more efficient is explicit index knowledge for a raster than for a 2D KDTree though? 



There's almost definitely OpenCV stuff for patching holes in raster images I bet...
