// Internal dependencies
#include "skeleton_tracker/Munkres.h"

cv::Mat_<int> hiros::track::utils::Munkres::solve(const cv::Mat_<double>& t_matrix_in, bool t_min)
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

      default:
        std::cerr << "Error: out of range step" << std::endl;
        m_step = Step::done;
        break;
    }
  }

  // Conversion from array to cv::Mat
  cv::Mat_<int> matrix_out(t_matrix_in.rows, t_matrix_in.cols, CV_64F);
  for (int i = 0; i < t_matrix_in.rows; i++) {
    for (int j = 0; j < t_matrix_in.cols; j++) {
      matrix_out(i, j) = m_matrix_out[i][j];
    }
  }

  releaseMemory();

  return matrix_out;
}

void hiros::track::utils::Munkres::preprocess(const cv::Mat_<double>& t_matrix_in, bool t_min)
{
  m_max_dim = std::max(t_matrix_in.rows, t_matrix_in.cols);
  m_rows = m_max_dim;
  m_cols = m_max_dim;

  double min_val, max_val;
  cv::minMaxLoc(t_matrix_in, &min_val, &max_val);

  // Pad input matrix with max matrix value to make it square
  m_matrix_in_padded = cv::Mat_<double>(m_rows, m_cols, max_val);

  for (int r = 0; r < t_matrix_in.rows; ++r) {
    for (int c = 0; c < t_matrix_in.cols; ++c) {
      m_matrix_in_padded(r, c) = t_matrix_in(r, c);
    }
  }

  initializeMemory();

  // Change cost matrix according to the type of optimum to find (minimum or maximum)
  if (!t_min) {
    double max_value = getMaxValue();

    for (int r = 0; r < m_rows; ++r) {
      for (int c = 0; c < m_cols; ++c) {
        m_matrix_in[r][c] = max_value - m_matrix_in[r][c];
      }
    }
  }
}

void hiros::track::utils::Munkres::initializeMemory()
{
  m_matrix_in = new double*[m_rows];
  m_matrix_out = new int*[m_rows];

  m_row_cover = new int[m_rows];
  m_col_cover = new int[m_cols];

  m_path = new int*[m_rows * m_cols];

  for (int r = 0; r < m_rows; ++r) {
    m_matrix_in[r] = new double[m_cols];
    m_matrix_out[r] = new int[m_cols];

    m_row_cover[r] = 0;

    for (int c = 0; c < m_cols; ++c) {
      m_matrix_in[r][c] = m_matrix_in_padded(r, c);
      m_matrix_out[r][c] = 0;

      m_col_cover[c] = 0;
    }
  }

  for (int p = 0; p < m_rows * m_cols; ++p) {
    m_path[p] = new int[2];
  }
}

void hiros::track::utils::Munkres::releaseMemory()
{
  for (int r = 0; r < m_max_dim; ++r) {
    delete[] m_matrix_in[r];
  }
  delete[] m_matrix_in;

  for (int i = 0; i < m_rows; i++) {
    delete[] m_matrix_out[i];
  }
  delete[] m_matrix_out;

  delete[] m_row_cover;
  delete[] m_col_cover;

  for (int p = 0; p < m_rows * m_cols; ++p) {
    delete[] m_path[p];
  }
  delete[] m_path;
}

double hiros::track::utils::Munkres::getMaxValue()
{
  double max = -std::numeric_limits<double>::max();

  for (int r = 0; r < m_rows; ++r) {
    for (int c = 0; c < m_cols; ++c) {
      max = std::max(max, m_matrix_in[r][c]);
    }
  }

  return max;
}

void hiros::track::utils::Munkres::stepOne()
{
  double min_in_row;

  for (int r = 0; r < m_rows; ++r) {
    min_in_row = getMinInRow(r);

    for (int c = 0; c < m_cols; ++c) {
      m_matrix_in[r][c] -= min_in_row;
    }
  }

  m_step = Step::two;
}

void hiros::track::utils::Munkres::stepTwo()
{
  for (int r = 0; r < m_rows; ++r) {
    for (int c = 0; c < m_cols; ++c) {
      if (m_matrix_in[r][c] == 0. && m_row_cover[r] == 0 && m_col_cover[c] == 0) {
        m_matrix_out[r][c] = 1;
        m_row_cover[r] = 1;
        m_col_cover[c] = 1;
      }
    }
  }

  for (int r = 0; r < m_rows; ++r) {
    m_row_cover[r] = 0;
  }

  for (int c = 0; c < m_cols; ++c) {
    m_col_cover[c] = 0;
  }

  m_step = Step::three;
}

void hiros::track::utils::Munkres::stepThree()
{
  int col_count = 0;

  for (int r = 0; r < m_rows; ++r) {
    for (int c = 0; c < m_cols; ++c) {
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

void hiros::track::utils::Munkres::stepFour()
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
      m_matrix_out[r][c] = 2;

      if (starInRow(r)) {
        c = findStarInRow(r);
        m_row_cover[r] = 1;
        m_col_cover[c] = 0;
      }
      else {
        done = true;
        m_path_row_0 = r;
        m_path_col_0 = c;

        m_step = Step::five;
      }
    }
  }
}

void hiros::track::utils::Munkres::stepFive()
{
  bool done = false;
  int r = -1;
  int c = -1;

  m_path_count = 1;
  m_path[m_path_count - 1][0] = m_path_row_0;
  m_path[m_path_count - 1][1] = m_path_col_0;

  while (!done) {
    r = findStarInCol(m_path[m_path_count - 1][1]);
    if (r > -1) {
      ++m_path_count;
      m_path[m_path_count - 1][0] = r;
      m_path[m_path_count - 1][1] = m_path[m_path_count - 2][1];
    }
    else {
      done = true;
    }

    if (!done) {
      c = findPrimeInRow(m_path[m_path_count - 1][0]);
      ++m_path_count;
      m_path[m_path_count - 1][0] = m_path[m_path_count - 2][0];
      m_path[m_path_count - 1][1] = c;
    }
  }

  augmentPath();
  clearCovers();
  erasePrimes();

  m_step = Step::three;
}

void hiros::track::utils::Munkres::stepSix()
{
  double minval = getMinValue();

  for (int r = 0; r < m_rows; ++r) {
    for (int c = 0; c < m_cols; ++c) {
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

double hiros::track::utils::Munkres::getMinInRow(int t_row)
{
  double min = m_matrix_in[t_row][0];

  for (int c = 1; c < m_cols; ++c) {
    min = std::min(min, m_matrix_in[t_row][c]);
  }

  return min;
}

void hiros::track::utils::Munkres::findZero(int& t_row, int& t_col)
{
  t_row = -1;
  t_col = -1;

  for (int r = 0; r < m_rows; ++r) {
    t_col = findZeroInRow(r);

    if (t_col != -1) {
      t_row = r;
      break;
    }
  }
}

int hiros::track::utils::Munkres::findZeroInRow(int t_row)
{
  for (int c = 0; c < m_cols; ++c) {
    if (m_matrix_in[t_row][c] == 0. && m_row_cover[t_row] == 0 && m_col_cover[c] == 0) {
      return c;
    }
  }

  return -1;
}

bool hiros::track::utils::Munkres::starInRow(int t_row)
{
  for (int c = 0; c < m_cols; ++c) {
    if (m_matrix_out[t_row][c] == 1) {
      return true;
    }
  }

  return false;
}

int hiros::track::utils::Munkres::findStarInRow(int t_row)
{
  for (int c = 0; c < m_cols; ++c) {
    if (m_matrix_out[t_row][c] == 1) {
      return c;
    }
  }

  return -1;
}

int hiros::track::utils::Munkres::findStarInCol(int t_col)
{
  for (int r = 0; r < m_rows; ++r) {
    if (m_matrix_out[r][t_col] == 1) {
      return r;
    }
  }

  return -1;
}

int hiros::track::utils::Munkres::findPrimeInRow(int t_row)
{
  for (int c = 0; c < m_cols; ++c) {
    if (m_matrix_out[t_row][c] == 2) {
      return c;
    }
  }

  return -1;
}

void hiros::track::utils::Munkres::augmentPath()
{
  for (int p = 0; p < m_path_count; ++p) {
    if (m_matrix_out[m_path[p][0]][m_path[p][1]] == 1) {
      m_matrix_out[m_path[p][0]][m_path[p][1]] = 0;
    }
    else {
      m_matrix_out[m_path[p][0]][m_path[p][1]] = 1;
    }
  }
}

void hiros::track::utils::Munkres::clearCovers()
{
  for (int r = 0; r < m_rows; ++r) {
    m_row_cover[r] = 0;
  }

  for (int c = 0; c < m_cols; ++c) {
    m_col_cover[c] = 0;
  }
}

void hiros::track::utils::Munkres::erasePrimes()
{
  for (int r = 0; r < m_rows; ++r) {
    for (int c = 0; c < m_cols; ++c) {
      if (m_matrix_out[r][c] == 2) {
        m_matrix_out[r][c] = 0;
      }
    }
  }
}

double hiros::track::utils::Munkres::getMinValue()
{
  double min = std::numeric_limits<double>::max();

  for (int r = 0; r < m_rows; ++r) {
    for (int c = 0; c < m_cols; ++c) {
      if (m_row_cover[r] == 0 && m_col_cover[c] == 0) {
        min = std::min(min, m_matrix_in[r][c]);
      }
    }
  }

  return min;
}
