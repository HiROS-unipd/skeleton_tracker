#ifndef hiros_skeleton_tracker_Munkres_h
#define hiros_skeleton_tracker_Munkres_h

// OpenCV dependencies
#include <opencv2/opencv.hpp>

namespace hiros {
namespace skeletons {

class Munkres {
 public:
  Munkres();
  ~Munkres();

  cv::Mat_<int> solve(const cv::Mat_<double>& matrix, const bool& min = true);

  bool match(const unsigned int& row, const unsigned int& col) const;
  bool colHasMatch(const unsigned int& col) const;
  int findMatchInCol(const unsigned int& col) const;
  bool rowHasMatch(const unsigned int& row) const;
  int findMatchInRow(const unsigned int& row) const;

 private:
  enum Step { one, two, three, four, five, six, done };

  void preprocess(const cv::Mat_<double>& matrix_in, const bool& min);
  void initializeVectors();

  void stepOne();
  void stepTwo();
  void stepThree();
  void stepFour();
  void stepFive();
  void stepSix();

  double getMaxValue(const std::vector<std::vector<double>>& mat) const;
  double getMaxValue(const cv::Mat_<double>& mat) const;
  double getMinInRow(const unsigned int& row) const;
  void findZero(int& row, int& col) const;
  int findZeroInRow(const unsigned int& row) const;
  bool starInRow(const unsigned int& row) const;
  int findStarInRow(const unsigned int& row) const;
  int findStarInCol(const unsigned int& col) const;
  int findPrimeInRow(const unsigned int& row) const;
  double getMinValue() const;
  void augmentPath();
  void clearCovers();
  void erasePrimes();

  cv::Mat_<double> cv_matrix_in_padded_{};
  cv::Mat_<int> cv_matrix_out_{};

  std::vector<std::vector<double>> matrix_in_{};
  std::vector<std::vector<int>> matrix_out_{};

  unsigned int max_dim_{};
  unsigned int rows_{};
  unsigned int cols_{};

  std::vector<unsigned int> row_cover_{};
  std::vector<unsigned int> col_cover_{};

  unsigned int path_row_0_{};
  unsigned int path_col_0_{};
  unsigned int path_count_{};

  std::vector<std::vector<unsigned int>> path_{};

  Step step_{};
};

}  // namespace skeletons
}  // namespace hiros
#endif
