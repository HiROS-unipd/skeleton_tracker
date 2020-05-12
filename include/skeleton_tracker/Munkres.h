#ifndef hiros_skeleton_tracker_Munkres_h
#define hiros_skeleton_tracker_Munkres_h

// OpenCV dependencies
#include <opencv2/opencv.hpp>

namespace hiros {
  namespace track {
    namespace utils {

      class Munkres
      {
      public:
        cv::Mat_<int> solve(const cv::Mat_<double>& t_matrix, bool t_min = true);

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

        void preprocess(const cv::Mat_<double>& t_matrix_in, bool t_min);
        void initializeMemory();
        void releaseMemory();
        double getMaxValue();

        void stepOne();
        void stepTwo();
        void stepThree();
        void stepFour();
        void stepFive();
        void stepSix();

        double getMinInRow(int t_row);
        void findZero(int& t_row, int& t_col);
        int findZeroInRow(int t_row);
        bool starInRow(int t_row);
        int findStarInRow(int t_row);
        int findStarInCol(int t_col);
        int findPrimeInRow(int t_row);
        void augmentPath();
        void clearCovers();
        void erasePrimes();
        double getMinValue();

        cv::Mat_<double> m_matrix_in_padded;

        double** m_matrix_in;
        int** m_matrix_out;

        int m_max_dim;
        int m_rows;
        int m_cols;

        int* m_row_cover;
        int* m_col_cover;

        int m_path_row_0 = 0;
        int m_path_col_0 = 0;
        int m_path_count = 0;

        int** m_path;

        Step m_step = one;
      };

    } // namespace utils
  } // namespace track
} // namespace hiros
#endif
