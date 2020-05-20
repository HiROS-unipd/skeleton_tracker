// Internal dependencies
#include "skeleton_tracker/Munkres.h"

hiros::track::Munkres::Munkres() {}

hiros::track::Munkres::~Munkres() {}

cv::Mat_<int> hiros::track::Munkres::solve(const cv::Mat_<double>& t_matrix_in, const bool& t_min)
{
  preprocess(t_matrix_in, t_min);

  while (m_step != Step::done) {
    switch (m_step) {
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
  cv::Mat_<int> matrix_out(t_matrix_in.rows, t_matrix_in.cols, CV_64F);
  for (int r = 0; r < t_matrix_in.rows; ++r) {
    for (int c = 0; c < t_matrix_in.cols; ++c) {
      matrix_out(r, c) = m_matrix_out[static_cast<unsigned int>(r)][static_cast<unsigned int>(c)];
    }
  }

  return matrix_out;
}

void hiros::track::Munkres::preprocess(const cv::Mat_<double>& t_matrix_in, const bool& t_min)
{
  m_path_row_0 = 0;
  m_path_col_0 = 0;
  m_path_count = 0;

  m_step = Step::one;

  m_max_dim = static_cast<unsigned int>(std::max(t_matrix_in.rows, t_matrix_in.cols));
  m_rows = m_max_dim;
  m_cols = m_max_dim;

  // Pad input matrix with max matrix value to make it square
  m_matrix_in_padded = cv::Mat_<double>(static_cast<int>(m_rows), static_cast<int>(m_cols), getMaxValue(t_matrix_in));

  for (int r = 0; r < t_matrix_in.rows; ++r) {
    for (int c = 0; c < t_matrix_in.cols; ++c) {
      m_matrix_in_padded(r, c) = t_matrix_in(r, c);
    }
  }

  initializeVectors();

  // Change cost matrix according to the type of optimum to find (minimum or maximum)
  if (!t_min) {
    double max_val = getMaxValue(m_matrix_in);

    for (unsigned int r = 0; r < m_rows; ++r) {
      for (unsigned int c = 0; c < m_cols; ++c) {
        m_matrix_in[r][c] = max_val - m_matrix_in[r][c];
      }
    }
  }
}

void hiros::track::Munkres::initializeVectors()
{
  m_matrix_in.resize(m_rows);
  m_matrix_out.resize(m_rows);
  m_path.resize(m_rows * m_cols, std::vector<unsigned int>(2));
  m_row_cover.resize(m_rows);
  m_col_cover.resize(m_cols);

  for (unsigned int r = 0; r < m_rows; ++r) {
    m_matrix_in[r].resize(m_cols);
    m_matrix_out[r].resize(m_cols);

    m_matrix_in[r] = m_matrix_in_padded.row(static_cast<int>(r));
    std::fill(m_matrix_out[r].begin(), m_matrix_out[r].end(), 0);
  }

  for (auto& p : m_path) {
    std::fill(p.begin(), p.end(), 0);
  }

  std::fill(m_row_cover.begin(), m_row_cover.end(), 0);
  std::fill(m_col_cover.begin(), m_col_cover.end(), 0);
}

void hiros::track::Munkres::stepOne()
{
  double min_in_row;

  for (unsigned int r = 0; r < m_rows; ++r) {
    min_in_row = getMinInRow(r);

    for (unsigned int c = 0; c < m_cols; ++c) {
      m_matrix_in[r][c] -= min_in_row;
    }
  }

  m_step = Step::two;
}

void hiros::track::Munkres::stepTwo()
{
  for (unsigned int r = 0; r < m_rows; ++r) {
    for (unsigned int c = 0; c < m_cols; ++c) {
      if (m_matrix_in[r][c] == 0. && m_row_cover[r] == 0 && m_col_cover[c] == 0) {
        m_matrix_out[r][c] = 1;
        m_row_cover[r] = 1;
        m_col_cover[c] = 1;
      }
    }
  }

  for (unsigned int r = 0; r < m_rows; ++r) {
    m_row_cover[r] = 0;
  }

  for (unsigned int c = 0; c < m_cols; ++c) {
    m_col_cover[c] = 0;
  }

  m_step = Step::three;
}

void hiros::track::Munkres::stepThree()
{
  unsigned int col_count = 0;

  for (unsigned int r = 0; r < m_rows; ++r) {
    for (unsigned int c = 0; c < m_cols; ++c) {
      if (m_matrix_out[r][c] == 1) {
        m_col_cover[c] = 1;
        ++col_count;
        break;
      }
    }
  }

  if (col_count >= m_rows || col_count >= m_cols) {
    m_step = Step::done;
  }
  else {
    m_step = Step::four;
  }
}

void hiros::track::Munkres::stepFour()
{
  bool done = false;
  int r = -1;
  int c = -1;

  while (!done) {
    findZero(r, c);

    if (r == -1) {
      done = true;

      m_step = Step::six;
    }
    else {
      m_matrix_out[static_cast<unsigned int>(r)][static_cast<unsigned int>(c)] = 2;

      if (starInRow(static_cast<unsigned int>(r))) {
        c = findStarInRow(static_cast<unsigned int>(r));

        m_row_cover[static_cast<unsigned int>(r)] = 1;
        m_col_cover[static_cast<unsigned int>(c)] = 0;
      }
      else {
        done = true;
        m_path_row_0 = static_cast<unsigned int>(r);
        m_path_col_0 = static_cast<unsigned int>(c);

        m_step = Step::five;
      }
    }
  }
}

void hiros::track::Munkres::stepFive()
{
  bool done = false;
  int r = -1;
  int c = -1;

  m_path_count = 1;
  m_path[m_path_count - 1][0] = m_path_row_0;
  m_path[m_path_count - 1][1] = m_path_col_0;

  while (!done) {
    r = findStarInCol(m_path[m_path_count - 1][1]);
    if (r != -1) {
      ++m_path_count;
      m_path[m_path_count - 1][0] = static_cast<unsigned int>(r);
      m_path[m_path_count - 1][1] = m_path[m_path_count - 2][1];
    }
    else {
      done = true;
    }

    if (!done) {
      c = findPrimeInRow(m_path[m_path_count - 1][0]);
      ++m_path_count;
      m_path[m_path_count - 1][0] = m_path[m_path_count - 2][0];
      m_path[m_path_count - 1][1] = static_cast<unsigned int>(c);
    }
  }

  augmentPath();
  clearCovers();
  erasePrimes();

  m_step = Step::three;
}

void hiros::track::Munkres::stepSix()
{
  double minval = getMinValue();

  for (unsigned int r = 0; r < m_rows; ++r) {
    for (unsigned int c = 0; c < m_cols; ++c) {
      if (m_row_cover[r] == 1) {
        m_matrix_in[r][c] += minval;
      }

      if (m_col_cover[c] == 0) {
        m_matrix_in[r][c] -= minval;
      }
    }
  }

  m_step = Step::four;
}

double hiros::track::Munkres::getMaxValue(const std::vector<std::vector<double>>& t_mat) const
{
  if (t_mat.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  double max_val = std::numeric_limits<double>::lowest();

  for (auto& row : t_mat) {
    for (auto& elem : row) {
      max_val = std::max(max_val, elem);
    }
  }

  return max_val;
}

double hiros::track::Munkres::getMaxValue(const cv::Mat_<double>& t_mat) const
{
  if (t_mat.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  double max_val = std::numeric_limits<double>::lowest();

  for (auto& elem : t_mat) {
    max_val = std::max(max_val, elem);
  }

  return max_val;
}

double hiros::track::Munkres::getMinInRow(const unsigned int& t_row) const
{
  double min_val = std::numeric_limits<double>::max();

  for (auto& elem : m_matrix_in[t_row]) {
    min_val = std::min(min_val, elem);
  }

  return min_val;
}

void hiros::track::Munkres::findZero(int& t_row, int& t_col) const
{
  t_row = -1;
  t_col = -1;

  for (unsigned int r = 0; r < m_rows; ++r) {
    t_col = findZeroInRow(r);

    if (t_col != -1) {
      t_row = static_cast<int>(r);
      break;
    }
  }
}

int hiros::track::Munkres::findZeroInRow(const unsigned int& t_row) const
{
  for (unsigned int c = 0; c < m_cols; ++c) {
    if (m_matrix_in[t_row][c] == 0. && m_row_cover[t_row] == 0 && m_col_cover[c] == 0) {
      return static_cast<int>(c);
    }
  }

  return -1;
}

bool hiros::track::Munkres::starInRow(const unsigned int& t_row) const
{
  for (unsigned int c = 0; c < m_cols; ++c) {
    if (m_matrix_out[t_row][c] == 1) {
      return true;
    }
  }

  return false;
}

int hiros::track::Munkres::findStarInRow(const unsigned int& t_row) const
{
  for (unsigned int c = 0; c < m_cols; ++c) {
    if (m_matrix_out[t_row][c] == 1) {
      return static_cast<int>(c);
    }
  }

  return -1;
}

int hiros::track::Munkres::findStarInCol(const unsigned int& t_col) const
{
  for (unsigned int r = 0; r < m_rows; ++r) {
    if (m_matrix_out[r][t_col] == 1) {
      return static_cast<int>(r);
    }
  }

  return -1;
}

int hiros::track::Munkres::findPrimeInRow(const unsigned int& t_row) const
{
  for (unsigned int c = 0; c < m_cols; ++c) {
    if (m_matrix_out[t_row][c] == 2) {
      return static_cast<int>(c);
    }
  }

  return -1;
}

double hiros::track::Munkres::getMinValue() const
{
  double min_val = std::numeric_limits<double>::max();

  for (unsigned int r = 0; r < m_rows; ++r) {
    for (unsigned int c = 0; c < m_cols; ++c) {
      if (m_row_cover[r] == 0 && m_col_cover[c] == 0) {
        min_val = std::min(min_val, m_matrix_in[r][c]);
      }
    }
  }

  return min_val;
}

void hiros::track::Munkres::augmentPath()
{
  for (unsigned int p = 0; p < m_path_count; ++p) {
    if (m_matrix_out[m_path[p][0]][m_path[p][1]] == 1) {
      m_matrix_out[m_path[p][0]][m_path[p][1]] = 0;
    }
    else {
      m_matrix_out[m_path[p][0]][m_path[p][1]] = 1;
    }
  }
}

void hiros::track::Munkres::clearCovers()
{
  std::fill(m_row_cover.begin(), m_row_cover.end(), 0);
  std::fill(m_col_cover.begin(), m_col_cover.end(), 0);
}

void hiros::track::Munkres::erasePrimes()
{
  for (auto& row : m_matrix_out) {
    for (auto& elem : row) {
      if (elem == 2) {
        elem = 0;
      }
    }
  }
}
