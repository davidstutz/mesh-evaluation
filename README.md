# Mesh Evaluation

This is a parallel C++ implementation for efficiently computing distances
(in particular, accuracy and completeness) between meshes or between
point clouds and meshes.

If you use this tool, please cite the following papers:

    @inproceedings{Stutz2018CVPR,
        title = {Learning 3D Shape Completion from Laser Scan Data with Weak Supervision },
        author = {Stutz, David and Geiger, Andreas},
        booktitle = {IEEE Conference on Computer Vision and Pattern Recognition (CVPR)},
        publisher = {IEEE Computer Society},
        year = {2018}
    }
    @misc{Stutz2017,
        author = {David Stutz},
        title = {Learning Shape Completion from Bounding Boxes with CAD Shape Priors},
        month = {September},
        year = {2017},
        institution = {RWTH Aachen University},
        address = {Aachen, Germany},
        howpublished = {http://davidstutz.de/},
    }

## Overview

The implementation allows to compute mesh-to-mesh distance as well as
points-to-mesh distance. The main use case is evaluating 3D (surface) reconstruction
or shape completion algorithms that generate a mesh as output which is then
evaluated against a mesh or point cloud ground truth. The implementation
follows the idea of Jensen et al. [1] and computes accuracy and completeness. 
Accuracy is the distance of the reconstruction (i.e. the input) to the
ground truth (i.e. the reference); completeness is the distance from
ground truth to reconstruction. When input and reference are meshes, both
accuracy and completeness is computed, when the reference is a point cloud,
only completeness is computed.

    [1] Rasmus Ramsbøl Jensen, Anders Lindbjerg Dahl, George Vogiatzis, Engil Tola, Henrik Aanæs:
        Large Scale Multi-view Stereopsis Evaluation. CVPR 2014: 406-413

To compute mesh-to-mesh distance, the implementation first samples a fixed
number (e.g. 10k) points on the input mesh, and then computes the distance
of these points to the closest face of the reference mesh. For this, the
triangle-point distance from [christopherbatty/SDFGen](https://github.com/christopherbatty/SDFGen)
is used. For sampling points, we first compute the (relative) area of each
face and sample points on each face proportional to its area. This ensures
uniform sampling from the input mesh.

Meshes are assumed to be available in [OFF](http://segeval.cs.princeton.edu/public/off_format.html)
format; point clouds are assumed to be in a simple TXT format as described below.
Utilities to read and convert to/from these formats are also provided.

## Installation

Requirements for C++ tool:

* CMake;
* Boost;
* Eigen;
* OpenMP;
* C++11.

Requirements for Python tools:

* Numpy.

On Ubuntu and related Linux distributions, these requirements can be installed
as follows:

    sudo apt-get install build-essential cmake libboost-all-dev libeigen3-dev

For using the Python tools, also make sure to install numpy, h5py and skimage (or PyMCubes):

    pip install numpy

To build, **first adapt `cmake/FindEigen3.cmake` to include the correct path
to Eigen3's include directory and remove `NO_CMAKE_SYSTEM_PATH` if necessary**, and run:

    mkdir build
    cd build
    cmake ..
    make

To test the installation you can run (form within the `build` directory):

    ../bin/evaluate ../examples/input/ ../examples/reference_off/ ../examples/output.txt
    ../bin/evaluate ../examples/input/ ../examples/reference_txt/ ../examples/output.txt

Now install [MeshLab](http://www.meshlab.net/) to visualize the OFF files
in `examples/input` and `examples/reference_off` for comparison. For visualizing
the point clouds in `examples/reference_ply`, use (from within `build`):

    ../examples/txt_to_ply.py ../examples/reference_txt/ ../examples/reference_ply

and then open the `.ply` files using MeshLab.

## Usage

Using the `--help` option will give a detailed summary of available options:

    $ ../bin/evaluate --help
    Allowed options:
      --help                  produce help message
      --input arg             input, either single OFF file or directory containing
                              OFF files where the names correspond to integers 
                              (zero padding allowed) and are consecutively numbered
                              starting with zero
      --reference arg         reference, either single OFF or TXT file or directory
                              containing OFF or TXT files where the names 
                              correspond to integers (zero padding allowed) and are
                              consecutively numbered starting with zero (the file 
                              names need to correspond to those found in the input 
                              directory); for TXT files, accuracy cannot be 
                              computed
      --output arg            output file, a TXT file containing accuracy and 
                              completeness for each input-reference pair as well as
                              overall averages
      --n_points arg (=10000) number points to sample from meshes in order to 
                              compute distances

The tool is able to evaluate single input-reference pairs where the reference
is either an OFF file or a TXT file and the input has to be an OFF file. 
Alternatively, the tool can evaluate multiple input-reference pairs. Then,
the input meshes need to be stored in a directory and named according to
consecutive integers (see e.g. `examples/input`). The reference files
should follow the same convention.

Using `--n_points` the number of samples points to compute mesh-to-mesh distances
can be controlled. Less points will reduce runtime but also make the distance less
reliable; this means that the random sampling has more influence.

The output file is a TXT file storing accuracy/completeness for each input-reference
pair as well as overall averages as follows:

    0 0.824145 0.832041 # 1st input-reference pair
    1 0.747299 0.702853 # 2nd input-reference pair
    # ...
    10 1.46198 1.36116 # 10th input-reference pair
    0.868443 0.89825 # averages for accuracy and completeness

## License

License for source code corresponding to:

D. Stutz, A. Geiger. **Learning 3D Shape Completion from Laser Scan Data with Weak Supervision.** IEEE Conference on Computer Vision and Pattern Recognition (CVPR), 2018.

Note that the source code is based on the following projects for which separate licenses apply:

* `triangle_point/README.md`
* [Tronic/cmake-modules](https://github.com/Tronic/cmake-modules)

Copyright (c) 2018 David Stutz, Max-Planck-Gesellschaft

**Please read carefully the following terms and conditions and any accompanying documentation before you download and/or use this software and associated documentation files (the "Software").**

The authors hereby grant you a non-exclusive, non-transferable, free of charge right to copy, modify, merge, publish, distribute, and sublicense the Software for the sole purpose of performing non-commercial scientific research, non-commercial education, or non-commercial artistic projects.

Any other use, in particular any use for commercial purposes, is prohibited. This includes, without limitation, incorporation in a commercial product, use in a commercial service, or production of other artefacts for commercial purposes.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

You understand and agree that the authors are under no obligation to provide either maintenance services, update services, notices of latent defects, or corrections of defects with regard to the Software. The authors nevertheless reserve the right to update, modify, or discontinue the Software at any time.

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software. You agree to cite the corresponding papers (see above) in documents and papers that report on research using the Software.
