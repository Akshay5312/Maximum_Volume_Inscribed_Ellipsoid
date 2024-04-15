
#include <fstream>
#include <filesystem>

#include <Eigen/Dense>
#include <drake/geometry/optimization/hpolyhedron.h>


// Test Drake HPolyhedron::MaximumVolumeInscribedEllipsoid()
void read_polytope(std::string file_path, Eigen::MatrixXd &A, Eigen::VectorXd &b)
{
  drake::log()->info("Reading polytope from file: {}", file_path);
  std::ifstream file(file_path);
  std::string line;

  if (file.is_open())
  {
    while (std::getline(file, line))
    {
      if (line == "A{")
      {
        A = Eigen::MatrixXd::Zero(0, 0);
        while (std::getline(file, line))
        {
          if (line == "}")
          {
            break;
          }
          std::istringstream is_line(line);
          std::string value;
          std::vector<double> row;
          while (std::getline(is_line, value, ' '))
          {
            if (value == " " || value == "\n" || value == "")
            {
              continue;
            }
            row.push_back(std::stod(value));
          }
          A.conservativeResize(A.rows() + 1, row.size());
          
          A.row(A.rows() - 1) = Eigen::VectorXd::Map(row.data(), row.size());
        }
      }
      else if (line == "b{")
      {
        std::getline(file, line);
        std::istringstream is_line(line);
        std::string value;
        std::vector<double> row;
        while (std::getline(is_line, value, ' '))
        {
          if (value == " " || value == "\n" || value == "")
          {
            continue;
          }
          row.push_back(std::stod(value));
        }
        b = Eigen::VectorXd::Map(row.data(), row.size());
      }
    }

    file.close();
  }
}


int main()
{
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
  std::filesystem::path filePath = std::filesystem::current_path() / "example_polytope.txt";
  read_polytope(filePath, A, b);

  drake::geometry::optimization::HPolyhedron polytope(A, b);

  bool is_empty = polytope.IsEmpty();
  bool is_bounded = polytope.IsBounded();

  assert(!is_empty);
  assert(is_bounded);


  drake::log()->info("A: \n{}", A);
  drake::log()->info("b: \n{}", b);

  auto chebyshev_center = polytope.ChebyshevCenter();

  drake::log()->info("Chebyshev center: \n{}", chebyshev_center.transpose());

  auto maximum_ellipsoid = polytope.MaximumVolumeInscribedEllipsoid();

  drake::log()->info("polytope ellipsoid center: \n{}", maximum_ellipsoid.center().transpose());

  return 0;
}
