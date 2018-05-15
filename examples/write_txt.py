import os
import argparse
import numpy as np

def write_txt(filepath, points):
    """
    Write a point cloud to txt file in the following format::

        num_points
        point1x point1y point1z
        point2x point2y point2z
        ...

    :param filepath: path to file to read
    :type filepath: str
    :param points: points
    :type points: numpy.ndarray
    """

    with open(filepath, 'w') as f:
        f.write(str(points.shape[0]) + '\n')

        for i in range(points.shape[0]):
            f.write(str(points[i, 0]) + ' ' + str(points[i, 1]) + ' ' + str(points[i, 2]) + '\n')

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Write example point cloud as TXT.')
    parser.add_argument('output', type=str, help='TXT file for example point cloud.')

    args = parser.parse_args()
    write_txt(args.output, np.random.random((100, 3)))
