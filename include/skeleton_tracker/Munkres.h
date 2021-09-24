#ifndef hiros_skeleton_tracker_Munkres_h
#define hiros_skeleton_tracker_Munkres_h

// OpenCV dependencies
#include <opencv2/opencv.hpp>

namespace hiros {
  namespace track {

    class Munkres
    {
    public:
      Munkres();
      ~Munkres();

      cv::Mat_<int> solve(const cv::Mat_<double>& t_matrix, const bool& t_min = true);

      bool match(const unsigned int& t_row, const unsigned int& t_col) const;
      bool colHasMatch(const unsigned int& t_col) const;
      int findMatchInCol(const unsigned int& t_col) const;
      bool rowHasMatch(const unsigned int& t_row) const;
      int findMatchInRow(const unsigned int& t_row) const;

    private:
      enum Step
      {
        one,
        two,
        three,
        four,
        five,
        six,
        done
      };

      void preprocess(const cv::Mat_<double>& t_matrix_in, const bool& t_min);
      void initializeVectors();

      void stepOne();
      void stepTwo();
      void stepThree();
      void stepFour();
      void stepFive();
      void stepSix();

      double getMaxValue(const std::vector<std::vector<double>>& t_mat) const;
      double getMaxValue(const cv::Mat_<double>& t_mat) const;
      double getMinInRow(const unsigned int& t_row) const;
      void findZero(int& t_row, int& t_col) const;
      int findZeroInRow(const unsigned int& t_row) const;
      bool starInRow(const unsigned int& t_row) const;
      int findStarInRow(const unsigned int& t_row) const;
      int findStarInCol(const unsigned int& t_col) const;
      int findPrimeInRow(const unsigned int& t_row) const;
      double getMinValue() const;
      void augmentPath();
      void clearCovers();
      void erasePrimes();

      cv::Mat_<double> m_cv_matrix_in_padded;
      cv::Mat_<int> m_cv_matrix_out;

      std::vector<std::vector<double>> m_matrix_in;
      std::vector<std::vector<int>> m_matrix_out;

      unsigned int m_max_dim;
      unsigned int m_rows;
      unsigned int m_cols;

      std::vector<unsigned int> m_row_cover;
      std::vector<unsigned int> m_col_cover;

      unsigned int m_path_row_0;
      unsigned int m_path_col_0;
      unsigned int m_path_count;

      std::vector<std::vector<unsigned int>> m_path;

      Step m_step;
    };

  } // namespace track
} // namespace hiros
#endif
