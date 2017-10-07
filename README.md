# dbscan

Simple dbscan implementation in c++11, uses pseudo code as in https://en.wikipedia.org/wiki/DBSCAN

Main function is dbscan, in the last argument you can provide a distance function, the default one calculates the euclidean distance of two points in 2 dimension. So the Point type in the list of points must have x and y members.
