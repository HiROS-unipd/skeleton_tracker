// Internal dependencies
#include "skeleton_tracker/Munkres.h"

hiros::skeletons::Munkres::Munkres() {}

hiros::skeletons::Munkres::~Munkres() {}

cv::Mat_<int> hiros::skeletons::Munkres::solve(
    const cv::Mat_<double>& matrix_in, const bool& min) {
  preprocess(matrix_in, min);

  while (step_ != Step::done) {
    switch (step_) {
      case Step::one:
        stepOne();
        break;

      case Step::two:
        stepTwo();
        break;

      case Step::three:
        stepThree();
        break;

      case Step::four:
        stepFour();
        break;

      case Step::five:
        stepFive();
        break;

      case Step::six:
        stepSix();
        break;

      case Step::done:
        break;
    }
  }

  // Conversion from array to cv::Mat
  cv_matrix_out_ = cv::Mat_<int>(matrix_in.rows, matrix_in.cols, CV_64F);
  for (auto r{0}; r < matrix_in.rows; ++r) {
    for (auto c{0}; c < matrix_in.cols; ++c) {
      cv_matrix_out_(r, c) = matrix_out_[static_cast<unsigned int>(r)]
                                        [static_cast<unsigned int>(c)];
    }
  }

  return cv_matrix_out_;
}

bool hiros::skeletons::Munkres::match(const unsigned int& row,
                                      const unsigned int& col) const {
  return (matrix_out_[row][col] == 1);
}

bool hiros::skeletons::Munkres::colHasMatch(const unsigned int& col) const {
  for (auto r{0}; r < cv_matrix_out_.rows; ++r) {
    if (cv_matrix_out_(r, static_cast<int>(col)) == 1) {
      return true;
    }
  }

  return false;
}

int hiros::skeletons::Munkres::findMatchInCol(const unsigned int& col) const {
  for (auto r{0}; r < cv_matrix_out_.rows; ++r) {
    if (cv_matrix_out_(r, static_cast<int>(col)) == 1) {
      return r;
    }
  }

  return -1;
}

bool hiros::skeletons::Munkres::rowHasMatch(const unsigned int& row) const {
  for (auto c{0}; c < cv_matrix_out_.cols; ++c) {
    if (cv_matrix_out_(static_cast<int>(row), c) == 1) {
      return true;
    }
  }

  return false;
}

int hiros::skeletons::Munkres::findMatchInRow(const unsigned int& row) const {
  for (auto c{0}; c < cv_matrix_out_.cols; ++c) {
    if (cv_matrix_out_(static_cast<int>(row), c) == 1) {
      return c;
    }
  }

  return -1;
}

void hiros::skeletons::Munkres::preprocess(const cv::Mat_<double>& matrix_in,
                                           const bool& min) {
  path_row_0_ = 0;
  path_col_0_ = 0;
  path_count_ = 0;

  step_ = Step::one;

  max_dim_ =
      static_cast<unsigned int>(std::max(matrix_in.rows, matrix_in.cols));
  rows_ = max_dim_;
  cols_ = max_dim_;

  // Pad input matrix with max matrix value to make it square
  cv_matrix_in_padded_ = cv::Mat_<double>(
      static_cast<int>(rows_), static_cast<int>(cols_), getMaxValue(matrix_in));

  for (auto r{0}; r < matrix_in.rows; ++r) {
    for (auto c{0}; c < matrix_in.cols; ++c) {
      cv_matrix_in_padded_(r, c) = matrix_in(r, c);
    }
  }

  initializeVectors();

  // Change cost matrix according to the type of optimum to find (minimum or
  // maximum)
  if (!min) {
    auto max_val{getMaxValue(matrix_in_)};

    for (auto r{0u}; r < rows_; ++r) {
      for (auto c{0u}; c < cols_; ++c) {
        matrix_in_[r][c] = max_val - matrix_in_[r][c];
      }
    }
  }
}

void hiros::skeletons::Munkres::initializeVectors() {
  matrix_in_.resize(rows_);
  matrix_out_.resize(rows_);
  path_.resize(rows_ * cols_, std::vector<unsigned int>(2));
  row_cover_.resize(rows_);
  col_cover_.resize(cols_);

  for (auto r{0u}; r < rows_; ++r) {
    matrix_in_[r].resize(cols_);
    matrix_out_[r].resize(cols_);

    matrix_in_[r] = cv_matrix_in_padded_.row(static_cast<int>(r));
    std::fill(matrix_out_[r].begin(), matrix_out_[r].end(), 0);
  }

  for (auto& p : path_) {
    std::fill(p.begin(), p.end(), 0);
  }

  std::fill(row_cover_.begin(), row_cover_.end(), 0);
  std::fill(col_cover_.begin(), col_cover_.end(), 0);
}

void hiros::skeletons::Munkres::stepOne() {
  double min_in_row{};

  for (auto r{0u}; r < rows_; ++r) {
    min_in_row = getMinInRow(r);

    for (auto c{0u}; c < cols_; ++c) {
      matrix_in_[r][c] -= min_in_row;
    }
  }

  step_ = Step::two;
}

void hiros::skeletons::Munkres::stepTwo() {
  for (auto r{0u}; r < rows_; ++r) {
    for (auto c{0u}; c < cols_; ++c) {
      if (matrix_in_[r][c] == 0. && row_cover_[r] == 0 && col_cover_[c] == 0) {
        matrix_out_[r][c] = 1;
        row_cover_[r] = 1;
        col_cover_[c] = 1;
      }
    }
  }

  for (auto r{0u}; r < rows_; ++r) {
    row_cover_[r] = 0;
  }

  for (auto c{0u}; c < cols_; ++c) {
    col_cover_[c] = 0;
  }

  step_ = Step::three;
}

void hiros::skeletons::Munkres::stepThree() {
  auto col_count{0u};

  for (auto r{0u}; r < rows_; ++r) {
    for (auto c{0u}; c < cols_; ++c) {
      if (matrix_out_[r][c] == 1) {
        col_cover_[c] = 1;
        ++col_count;
        break;
      }
    }
  }

  if (col_count >= rows_ || col_count >= cols_) {
    step_ = Step::done;
  } else {
    step_ = Step::four;
  }
}

void hiros::skeletons::Munkres::stepFour() {
  auto done{false};
  auto r{-1};
  auto c{-1};

  while (!done) {
    findZero(r, c);

    if (r == -1) {
      done = true;

      step_ = Step::six;
    } else {
      matrix_out_[static_cast<unsigned int>(r)][static_cast<unsigned int>(c)] =
          2;

      if (starInRow(static_cast<unsigned int>(r))) {
        c = findStarInRow(static_cast<unsigned int>(r));

        row_cover_[static_cast<unsigned int>(r)] = 1;
        col_cover_[static_cast<unsigned int>(c)] = 0;
      } else {
        done = true;
        path_row_0_ = static_cast<unsigned int>(r);
        path_col_0_ = static_cast<unsigned int>(c);

        step_ = Step::five;
      }
    }
  }
}

void hiros::skeletons::Munkres::stepFive() {
  auto done{false};
  auto r{-1};
  auto c{-1};

  path_count_ = 1;
  path_[path_count_ - 1][0] = path_row_0_;
  path_[path_count_ - 1][1] = path_col_0_;

  while (!done) {
    r = findStarInCol(path_[path_count_ - 1][1]);
    if (r != -1) {
      ++path_count_;
      path_[path_count_ - 1][0] = static_cast<unsigned int>(r);
      path_[path_count_ - 1][1] = path_[path_count_ - 2][1];
    } else {
      done = true;
    }

    if (!done) {
      c = findPrimeInRow(path_[path_count_ - 1][0]);
      ++path_count_;
      path_[path_count_ - 1][0] = path_[path_count_ - 2][0];
      path_[path_count_ - 1][1] = static_cast<unsigned int>(c);
    }
  }

  augmentPath();
  clearCovers();
  erasePrimes();

  step_ = Step::three;
}

void hiros::skeletons::Munkres::stepSix() {
  auto minval{getMinValue()};

  for (auto r{0u}; r < rows_; ++r) {
    for (auto c{0u}; c < cols_; ++c) {
      if (row_cover_[r] == 1) {
        matrix_in_[r][c] += minval;
      }

      if (col_cover_[c] == 0) {
        matrix_in_[r][c] -= minval;
      }
    }
  }

  step_ = Step::four;
}

double hiros::skeletons::Munkres::getMaxValue(
    const std::vector<std::vector<double>>& mat) const {
  if (mat.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  auto max_val{std::numeric_limits<double>::lowest()};

  for (auto& row : mat) {
    for (auto& elem : row) {
      max_val = std::max(max_val, elem);
    }
  }

  return max_val;
}

double hiros::skeletons::Munkres::getMaxValue(
    const cv::Mat_<double>& mat) const {
  if (mat.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  auto max_val{std::numeric_limits<double>::lowest()};

  for (auto& elem : mat) {
    max_val = std::max(max_val, elem);
  }

  return max_val;
}

double hiros::skeletons::Munkres::getMinInRow(const unsigned int& row) const {
  auto min_val{std::numeric_limits<double>::max()};

  for (auto& elem : matrix_in_[row]) {
    min_val = std::min(min_val, elem);
  }

  return min_val;
}

void hiros::skeletons::Munkres::findZero(int& row, int& col) const {
  row = -1;
  col = -1;

  for (auto r{0u}; r < rows_; ++r) {
    col = findZeroInRow(r);

    if (col != -1) {
      row = static_cast<int>(r);
      break;
    }
  }
}

int hiros::skeletons::Munkres::findZeroInRow(const unsigned int& row) const {
  for (auto c{0u}; c < cols_; ++c) {
    if (matrix_in_[row][c] == 0. && row_cover_[row] == 0 &&
        col_cover_[c] == 0) {
      return static_cast<int>(c);
    }
  }

  return -1;
}

bool hiros::skeletons::Munkres::starInRow(const unsigned int& row) const {
  for (auto c{0u}; c < cols_; ++c) {
    if (matrix_out_[row][c] == 1) {
      return true;
    }
  }

  return false;
}

int hiros::skeletons::Munkres::findStarInRow(const unsigned int& row) const {
  for (auto c{0u}; c < cols_; ++c) {
    if (matrix_out_[row][c] == 1) {
      return static_cast<int>(c);
    }
  }

  return -1;
}

int hiros::skeletons::Munkres::findStarInCol(const unsigned int& col) const {
  for (auto r{0u}; r < rows_; ++r) {
    if (matrix_out_[r][col] == 1) {
      return static_cast<int>(r);
    }
  }

  return -1;
}

int hiros::skeletons::Munkres::findPrimeInRow(const unsigned int& row) const {
  for (auto c{0u}; c < cols_; ++c) {
    if (matrix_out_[row][c] == 2) {
      return static_cast<int>(c);
    }
  }

  return -1;
}

double hiros::skeletons::Munkres::getMinValue() const {
  auto min_val{std::numeric_limits<double>::max()};

  for (auto r{0u}; r < rows_; ++r) {
    for (auto c{0u}; c < cols_; ++c) {
      if (row_cover_[r] == 0 && col_cover_[c] == 0) {
        min_val = std::min(min_val, matrix_in_[r][c]);
      }
    }
  }

  return min_val;
}

void hiros::skeletons::Munkres::augmentPath() {
  for (auto p{0u}; p < path_count_; ++p) {
    if (matrix_out_[path_[p][0]][path_[p][1]] == 1) {
      matrix_out_[path_[p][0]][path_[p][1]] = 0;
    } else {
      matrix_out_[path_[p][0]][path_[p][1]] = 1;
    }
  }
}

void hiros::skeletons::Munkres::clearCovers() {
  std::fill(row_cover_.begin(), row_cover_.end(), 0);
  std::fill(col_cover_.begin(), col_cover_.end(), 0);
}

void hiros::skeletons::Munkres::erasePrimes() {
  for (auto& row : matrix_out_) {
    for (auto& elem : row) {
      if (elem == 2) {
        elem = 0;
      }
    }
  }
}
