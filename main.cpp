#include <cstdio>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <cfloat>

// Eigen
#include <Eigen/Dense>
#include <unsupported/Eigen/CXX11/Tensor>

// Boost
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

// OpenMP
#include <omp.h>

#include "triangle_point/poitri.h"

/** \brief Compute triangle point distance and corresponding closest point.
 * \param[in] point point
 * \param[in] v1 first vertex
 * \param[in] v2 second vertex
 * \param[in] v3 third vertex
 * \param[out] ray corresponding closest point
 * \return distance
 */
float triangle_point_distance(const Eigen::Vector3f point, const Eigen::Vector3f v1, const Eigen::Vector3f v2, const Eigen::Vector3f v3,
    Eigen::Vector3f &closest_point) {

  Vec3f x0(point.data());
  Vec3f x1(v1.data());
  Vec3f x2(v2.data());
  Vec3f x3(v3.data());

  Vec3f r(0);
  float distance = point_triangle_distance(x0, x1, x2, x3, r);

  for (int d = 0; d < 3; d++) {
    closest_point(d) = r[d];
  }

  return distance;
}

/** \brief Point cloud class forward declaration. */
class PointCloud;

/** \brief Just encapsulating vertices and faces. */
class Mesh {
public:
  /** \brief Empty constructor. */
  Mesh() {

  }

  /** \brief Reading an off file and returning the vertices x, y, z coordinates and the
   * face indices.
   * \param[in] filepath path to the OFF file
   * \param[out] mesh read mesh with vertices and faces
   * \return success
   */
  static bool from_off(const std::string filepath, Mesh& mesh) {

    std::ifstream* file = new std::ifstream(filepath.c_str());
    std::string line;
    std::stringstream ss;
    int line_nb = 0;

    std::getline(*file, line);
    ++line_nb;

    if (line != "off" && line != "OFF") {
      std::cout << "[Error] Invalid header: \"" << line << "\", " << filepath << std::endl;
      return false;
    }

    size_t n_edges;
    std::getline(*file, line);
    ++line_nb;

    int n_vertices;
    int n_faces;
    ss << line;
    ss >> n_vertices;
    ss >> n_faces;
    ss >> n_edges;

    for (size_t v = 0; v < n_vertices; ++v) {
      std::getline(*file, line);
      ++line_nb;

      ss.clear();
      ss.str("");

      Eigen::Vector3f vertex;
      ss << line;
      ss >> vertex(0);
      ss >> vertex(1);
      ss >> vertex(2);

      mesh.add_vertex(vertex);
    }

    size_t n;
    for (size_t f = 0; f < n_faces; ++f) {
      std::getline(*file, line);
      ++line_nb;

      ss.clear();
      ss.str("");

      size_t n;
      ss << line;
      ss >> n;

      if(n != 3) {
        std::cout << "[Error] Not a triangle (" << n << " points) at " << (line_nb - 1) << std::endl;
        return false;
      }

      Eigen::Vector3i face;
      ss >> face(0);
      ss >> face(1);
      ss >> face(2);

      mesh.add_face(face);
    }

    if (n_vertices != mesh.num_vertices()) {
      std::cout << "[Error] Number of vertices in header differs from actual number of vertices." << std::endl;
      return false;
    }

    if (n_faces != mesh.num_faces()) {
      std::cout << "[Error] Number of faces in header differs from actual number of faces." << std::endl;
      return false;
    }

    file->close();
    delete file;

    return true;
  }

  /** \brief Write mesh to OFF file.
   * \param[in] filepath path to OFF file to write
   * \return success
   */
  bool to_off(const std::string filepath) const {
    std::ofstream* out = new std::ofstream(filepath, std::ofstream::out);
    if (!static_cast<bool>(out)) {
      return false;
    }

    (*out) << "OFF" << std::endl;
    (*out) << this->num_vertices() << " " << this->num_faces() << " 0" << std::endl;

    for (unsigned int v = 0; v < this->num_vertices(); v++) {
      (*out) << this->vertices[v](0) << " " << this->vertices[v](1) << " " << this->vertices[v](2) << std::endl;
    }

    for (unsigned int f = 0; f < this->num_faces(); f++) {
      (*out) << "3 " << this->faces[f](0) << " " << this->faces[f](1) << " " << this->faces[f](2) << std::endl;
    }

    out->close();
    delete out;

    return true;
  }

  /** \brief Add a vertex.
   * \param[in] vertex vertex to add
   */
  void add_vertex(Eigen::Vector3f& vertex) {
    this->vertices.push_back(vertex);
  }

  /** \brief Get the number of vertices.
   * \return number of vertices
   */
  int num_vertices() const {
    return static_cast<int>(this->vertices.size());
  }

  /** \brief Get a vertex.
   * \param[in] v vertex index
   * \return vertex
   */
  Eigen::Vector3f vertex(int v) const {
    assert(v >= 0 && v < this->num_vertices());
    return this->vertices[v];
  }

  /** \brief Add a face.
   * \param[in] face face to add
   */
  void add_face(Eigen::Vector3i& face) {
    this->faces.push_back(face);
  }

  /** \brief Get the number of faces.
   * \return number of faces
   */
  int num_faces() const {
    return static_cast<int>(this->faces.size());
  }

  /** \brief Get a face.
   * \param[in] f face index
   * \return face
   */
  Eigen::Vector3i face(int f) const {
    assert(f >= 0 && f < this->num_faces());
    return this->faces[f];
  }

  /** \brief Sample points from the mesh
   * \param[in] mesh mesh to sample from
   * \param[in] n batch index in points
   * \param[in] points pre-initialized tensor holding points
   */
  bool sample(const int N, PointCloud &point_cloud) const;

private:

  /** \brief Vertices as (x,y,z)-vectors. */
  std::vector<Eigen::Vector3f> vertices;

  /** \brief Faces as list of vertex indices. */
  std::vector<Eigen::Vector3i> faces;
};

/** \brief Class representing a point cloud in 3D. */
class PointCloud {
public:
  /** \brief Constructor. */
  PointCloud() {

  }

  /** \brief Copy constructor.
   * \param[in] point_cloud point cloud to copy
   */
  PointCloud(const PointCloud &point_cloud) {
    this->points.clear();

    for (unsigned int i = 0; i < point_cloud.points.size(); i++) {
      this->points.push_back(point_cloud.points[i]);
    }
  }

  /** \brief Destructor. */
  ~PointCloud() {

  }

  /** \brief Read point cloud from txt file.
   * \param[in] filepath path to file to read
   * \param[out] point_cloud
   * \return success
   */
  static bool from_txt(const std::string &filepath, PointCloud &point_cloud) {
    std::ifstream file(filepath.c_str());
    std::string line;
    std::stringstream ss;

    std::getline(file, line);
    ss << line;

    int n_points = 0;
    ss >> n_points;

    if (n_points < 0) {
      return false;
    }

    for (int i = 0; i < n_points; i++) {
      std::getline(file, line);

      ss.clear();
      ss.str("");
      ss << line;

      Eigen::Vector3f point(0, 0, 0);
      ss >> point(0);
      ss >> point(1);
      ss >> point(2);

      point_cloud.add_point(point);
    }

    return true;
  }

  /** \brief Add a point to the point cloud.
   * \param[in] point point to add
   */
  void add_point(const Eigen::Vector3f &point) {
    this->points.push_back(point);
  }

  /** \brief Get number of points.
   * \return number of points
   */
  unsigned int num_points() const {
    return this->points.size();
  }

  /** \brief Compute distance to mesh.
    * \param[in] mesh
    * \param[out] distances per point distances
    * \param[out] distance
    * \return success
    */
  bool compute_distance(const Mesh &mesh, float &_distance) {
    _distance = 0;

    if (this->num_points() <= 0) {
      std::cout << "[Error] no points in this point clouds" << std::endl;
      return false;
    }

    if (mesh.num_faces() <= 0) {
      std::cout << "[Error] no faces in given mesh" << std::endl;
      return false;
    }

    #pragma omp parallel
    {
      #pragma omp for
      for (unsigned int i = 0; i < this->points.size(); i++) {

        float min_distance = FLT_MAX;
        for (int f = 0; f < mesh.num_faces(); f++) {
          Eigen::Vector3f closest_point;
          Eigen::Vector3f v1 = mesh.vertex(mesh.face(f)(0));
          Eigen::Vector3f v2 = mesh.vertex(mesh.face(f)(1));
          Eigen::Vector3f v3 = mesh.vertex(mesh.face(f)(2));

          triangle_point_distance(this->points[i], v1, v2, v3, closest_point);
          float distance = (this->points[i] - closest_point).norm();

          if (distance < min_distance) {
            min_distance = distance;
          }
        }

        #pragma omp atomic
        _distance += min_distance;
      }
    }

    _distance /= this->num_points();
    return true;
  }

private:
  /** \brief The points of the point cloud. */
  std::vector<Eigen::Vector3f> points;

};

/** \brief Sample points from the mesh
 * \param[in] mesh mesh to sample from
 * \param[in] n batch index in points
 * \param[in] points pre-initialized tensor holding points
 */
bool Mesh::sample(const int N, PointCloud &point_cloud) const {

  // Stores the areas of faces.
  std::vector<float> areas(this->num_faces());
  float sum = 0;

  // Build a probability distribution over faces.
  for (int f = 0; f < this->num_faces(); f++) {
    Eigen::Vector3f a = this->vertices[this->faces[f][0]];
    Eigen::Vector3f b = this->vertices[this->faces[f][1]];
    Eigen::Vector3f c = this->vertices[this->faces[f][2]];

    // Angle between a->b and a->c.
    Eigen::Vector3f ab = b - a;
    Eigen::Vector3f ac = c - a;
    float cos_angle = ab.dot(ac)/(ab.norm()*ac.norm());
    float angle = std::acos(cos_angle);

    // Compute triangle area.
    float area = std::max(0., 0.5*ab.norm()*ac.norm()*std::sin(angle));
    //std::cout << area << " " << std::pow(area, 1./4.) << " " << angle << " " << ab.norm() << " " << ac.norm() << " " << std::sin(angle) << std::endl;

    // Accumulate.
    //area = std::sqrt(area);
    areas[f] = area;
    sum += area;
    //areas.push_back(1);
    //sum += 1;
  }

  //std::cout << sum << std::endl;
  if (sum < 1e-6) {
    std::cout << "[Error] face area sum of " << sum << std::endl;
    return false;
  }

  for (int f = 0; f < this->num_faces(); f++) {
    //std::cout << areas[f] << " ";
    areas[f] /= sum;
    //std::cout << areas[f] << std::endl;
  }

  std::vector<float> cum_areas(areas.size());
  cum_areas[0] = areas[0];

  for (int f = 1; f < this->num_faces(); f++) {
    cum_areas[f] = areas[f] + cum_areas[f - 1];
  }

  for (int f = 0; f < this->num_faces(); f++) {
    int n = std::max(static_cast<int>(areas[f]*N), 1);

    for (int i = 0; i < n; i++) {
      float r1 = 0;
      float r2 = 0;
      do {
        r1 = static_cast<float>(std::rand())/static_cast<float>(RAND_MAX);
        r2 = static_cast<float>(std::rand())/static_cast<float>(RAND_MAX);
      }
      while (r1 + r2 > 1.f);

      int s = std::rand()%3;
      //std::cout << face << " " << areas[face] << std::endl;

      Eigen::Vector3f a = this->vertices[this->faces[f](s)];
      Eigen::Vector3f b = this->vertices[this->faces[f]((s + 1)%3)];
      Eigen::Vector3f c = this->vertices[this->faces[f]((s + 2)%3)];

      Eigen::Vector3f ab = b - a;
      Eigen::Vector3f ac = c - a;

      Eigen::Vector3f point = a + r1*ab + r2*ac;
      point_cloud.add_point(point);
    }
  }

  return true;
}

/** \brief Read all files in a directory matching the given extension.
 * \param[in] directory path to directory
 * \param[out] files read file paths
 * \param[in] extension extension to filter for
 */
void read_directory(const boost::filesystem::path directory, std::map<int, boost::filesystem::path>& files, const std::vector<std::string> &extensions) {

  files.clear();
  boost::filesystem::directory_iterator end;

  for (boost::filesystem::directory_iterator it(directory); it != end; ++it) {
    bool filtered = true;
    for (unsigned int i = 0; i < extensions.size(); i++) {
      if (it->path().extension().string() == extensions[i]) {
        filtered = false;
      }
    }

    if (!filtered) {
      int number = std::stoi(it->path().filename().string());
      files.insert(std::pair<int, boost::filesystem::path>(number, it->path()));
    }
  }
}

/** \brief Main entrance point of the script.
 * Expects one parameter, the path to the corresponding config file in config/.
 */
int main(int argc, char** argv) {
  boost::program_options::options_description desc("Allowed options");
  desc.add_options()
      ("help", "produce help message")
      ("input",  boost::program_options::value<std::string>(), "input, either single OFF file or directory containing OFF files where the names correspond to integers (zero padding allowed) and are consecutively numbered starting with zero")
      ("reference", boost::program_options::value<std::string>(), "reference, either single OFF or TXT file or directory containing OFF or TXT files where the names correspond to integers (zero padding allowed) and are consecutively numbered starting with zero (the file names need to correspond to those found in the input directory); for TXT files, accuracy cannot be computed")
      ("output", boost::program_options::value<std::string>(), "output file, a TXT file containing accuracy and completeness for each input-reference pair as well as overall averages")
      ("n_points", boost::program_options::value<int>()->default_value(10000), "number points to sample from meshes in order to compute distances");

  boost::program_options::positional_options_description positionals;
  positionals.add("input", 1);
  positionals.add("reference", 1);
  positionals.add("output", 1);

  boost::program_options::variables_map parameters;
  boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(desc).positional(positionals).run(), parameters);
  boost::program_options::notify(parameters);

  if (parameters.find("help") != parameters.end()) {
    std::cout << desc << std::endl;
    return 1;
  }

  boost::filesystem::path input(parameters["input"].as<std::string>());
  if (!boost::filesystem::is_directory(input) && !boost::filesystem::is_regular_file(input)) {
    std::cout << "Input is neither directory nor file." << std::endl;
    return 1;
  }

  boost::filesystem::path reference(parameters["reference"].as<std::string>());
  if (!boost::filesystem::is_directory(reference) && !boost::filesystem::is_regular_file(reference)) {
    std::cout << "Reference is neither directory nor file." << std::endl;
    return 1;
  }

  boost::filesystem::path output(parameters["output"].as<std::string>());
  if (boost::filesystem::is_regular_file(output)) {
    std::cout << "Output file already exists; overwriting." << std::endl;
  }

  int N_points = parameters["n_points"].as<int>();
  std::cout << "Using " << N_points << " points." << std::endl;

  std::map<int, boost::filesystem::path> input_files;
  std::map<int, boost::filesystem::path> reference_files;

  if (boost::filesystem::is_regular_file(input)) {
    if (input.extension().string() != ".off") {
      std::cout << "Only OFF files supported as input." << std::endl;
      return 1;
    }

    input_files.insert(std::pair<int, boost::filesystem::path>(0, input));
  }
  else {
    read_directory(input, input_files, {".off"});

    if (input_files.size() <= 0) {
      std::cout << "Could not find any OFF files in input directory." << std::endl;
      return 1;
    }

    std::cout << "Read " << input_files.size() << " input files." << std::endl;
  }

  if (boost::filesystem::is_regular_file(reference)) {
    if (reference.extension().string() != ".off" && reference.extension().string() != ".txt") {
      std::cout << "Only OFF or TXT files supported as reference." << std::endl;
      return 1;
    }

    reference_files.insert(std::pair<int, boost::filesystem::path>(0, reference));
  }
  else {
    read_directory(reference, reference_files, {".off", ".txt"});

    if (input_files.size() <= 0) {
      std::cout << "Could not find any OFF or TXT files in reference directory." << std::endl;
      return 1;
    }

    std::cout << "Read " << reference_files.size() << " reference files." << std::endl;
  }

  std::map<int, float> accuracies;
  std::map<int, float> completenesses;

  for (std::map<int, boost::filesystem::path>::iterator it = input_files.begin(); it != input_files.end(); it++) {

    int n = it->first;
    if (reference_files.find(n) == reference_files.end()) {
      std::cout << "Could not find the reference file corresponding to " << input_files[n] << "." << std::endl;
      return 1;
    }

    boost::filesystem::path input_file = input_files[n];
    boost::filesystem::path reference_file = reference_files[n];

    Mesh input_mesh;
    bool success = Mesh::from_off(input_file.string(), input_mesh);

    if (!success) {
      std::cout << "Could not read " << input_file << "." << std::endl;
      return 1;
    }

    if (reference_file.extension().string() == ".off") {
      Mesh reference_mesh;
      success = Mesh::from_off(reference_file.string(), reference_mesh);

      if (!success) {
        std::cout << "Could not read " << reference_file << "." << std::endl;
        return 1;
      }

      PointCloud input_point_cloud;
      success = input_mesh.sample(N_points, input_point_cloud);

      if (success) {
        float accuracy = 0;
        success = input_point_cloud.compute_distance(reference_mesh, accuracy);

        if (success) {
          accuracies[n] = accuracy;
          std::cout << "Computed accuracy for " << input_file << "." << std::endl;
        }
        else {
          std::cout << "Could not compute accuracy for " << input_file << "." << std::endl;
        }
      }
      else {
        std::cout << "Could not compute accuracy for " << input_file << "." << std::endl;
      }

      PointCloud reference_point_cloud;
      reference_mesh.sample(N_points, reference_point_cloud);

      if (success) {
        float completeness = 0;
        success = reference_point_cloud.compute_distance(input_mesh, completeness);

        if (success) {
          completenesses[n] = completeness;
          std::cout << "Computed completeness for " << input_file << "." << std::endl;
        }
        else {
          std::cout << "Could not compute completeness for " << input_file << "." << std::endl;
        }
      }
      else {
        std::cout << "Could not compute completeness for " << input_file << "." << std::endl;
      }
    }
    else if (reference_file.extension().string() == ".txt") {
      PointCloud reference_point_cloud;
      success = PointCloud::from_txt(reference_file.string(), reference_point_cloud);

      if (!success) {
        std::cout << "Could not read " << reference_file << "." << std::endl;
        return 1;
      }

      float completeness = 0;
      success = reference_point_cloud.compute_distance(input_mesh, completeness);

      if (success) {
        completenesses[n] = completeness;
        std::cout << "Computed completeness for " << input_file << "." << std::endl;
      }
      else {
        std::cout << "Could not compute completeness for " << input_file << "." << std::endl;
      }
    }
    else {
      std::cout << "Reference file " << reference_file << " has invalid extension." << std::endl;
    }
  }

  std::ofstream* out = new std::ofstream(output.string(), std::ofstream::out);
  if (!static_cast<bool>(*out)) {
    std::cout << "Could not open " << output << std::endl;
    exit(1);
  }

  float accuracy = 0;
  float completeness = 0;

  for (std::map<int, boost::filesystem::path>::iterator it = input_files.begin(); it != input_files.end(); it++) {
    int n = it->first;

    (*out) << n << " ";
    if (accuracies.find(n) != accuracies.end()) {
      (*out) << accuracies[n];
      accuracy += accuracies[n];
    }
    else {
      (*out) << "-1";
    }

    (*out) << " ";
    if (completenesses.find(n) != completenesses.end()) {
      (*out) << completenesses[n];
      completeness += completenesses[n];
    }
    else {
      (*out) << "-1";
    }

    (*out) << std::endl;
  }


  if (accuracies.size() > 0) {
    accuracy /= accuracies.size();
    (*out) << accuracy;
    std::cout << "Accuracy (input to reference): " << accuracy << std::endl;
  }
  else {
    (*out) << "-1";
    std::cout << "Could not compute accuracy." << std::endl;
  }

  (*out) << " ";
  if (completenesses.size() > 0) {
    completeness /= completenesses.size();
    (*out) << completeness;
  std::cout << "Completeness (reference to input): " << completeness << std::endl;
  }
  else {
    (*out) << "-1";
    std::cout << "Could not compute completeness." << std::endl;
  }

  out->close();
  delete out;
  std::cout << "Wrote " << output << "." << std::endl;

  exit(0);
}