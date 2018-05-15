import os
import argparse
import numpy as np

def read_txt(filepath):
    """
    Read a point cloud from txt file in the following format::

        num_points
        point1x point1y point1z
        point2x point2y point2z
        ...

    :param filepath: path to file to read
    :type filepath: str
    :return: points
    :rtype: numpy.ndarray
    """

    assert os.path.exists(filepath)

    with open(filepath, 'r') as f:
        lines = f.readlines()
        lines = [line.strip() for line in lines if line.strip() != '']

        num_points = int(lines[0])
        points = np.zeros((num_points, 3))
        assert num_points == len(lines) - 1

        for i in range(0, num_points):
            line = lines[i + 1]

            parts = line.split(' ')
            assert len(parts) == 3, "invalid line: %s" % line

            for j in range(3):
                points[i, j] = float(parts[j])

    return points

def write_ply(filepath, points):
    """
    Write the given points to PLY.

    :param filepath: path to file to write
    :type filepath: str
    :param points: pointy
    :type points: numpy.ndarray
    """

    with open(filepath, 'w') as f:
        f.write('ply\n')
        f.write('format ascii 1.0\n')
        #f.write('format binary_little_endian 1.0\n')
        #f.write('format binary_big_endian 1.0\n')
        f.write('element vertex ' + str(points.shape[0]) + '\n')
        f.write('property float x\n')
        f.write('property float y\n')
        f.write('property float z\n')
        f.write('property uchar red\n')
        f.write('property uchar green\n')
        f.write('property uchar blue\n')
        f.write('end_header\n')

        for n in range(points.shape[0]):
            f.write(str(points[n, 0]) + ' ' + str(points[n, 1]) + ' ' + str(points[n, 2]))
            f.write(' 0 0 0\n')

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Convert TXT to PLY.')
    parser.add_argument('input', type=str, help='The input directory containing TXT files.')
    parser.add_argument('output', type=str, help='The output directory for PLY files.')

    args = parser.parse_args()
    if not os.path.exists(args.input):
        print('Input directory does not exist.')
        exit(1)

    if not os.path.exists(args.output):
        os.makedirs(args.output)
        print('Created output directory.')
    else:
        print('Output directory exists; potentially overwriting contents.')

    for filename in os.listdir(args.input):
        filepath = args.input + '/' + filename
        points = read_txt(filepath)
        print('Read %s.' % filepath)

        filepath = args.output + '/' + filename[:-4] + '.ply'
        write_ply(filepath, points)
        print('Wrote %s.' % filepath)
