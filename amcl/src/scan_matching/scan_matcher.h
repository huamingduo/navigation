#pragma once

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Eigen>

//#define DEBUG
//#define ANIMATE

namespace scan_matching {

struct Option {
  int map_stack_depth;
  double min_score;
  double linear_search_window;
  double angular_search_window;
  double shrink_scale;
  double kernal_size;
};

struct SearchParam {
  double linear_search_window;
  double angular_search_window;
  double linear_search_step;
  double angular_search_step;

  SearchParam(const double& linear_window, const double& angular_window,
    const double linear_step, const double& angular_step)
    : linear_search_window(linear_window), angular_search_window(angular_window),
      linear_search_step(linear_step), angular_search_step(angular_step) {}
};

struct Map {
  double resolution;
  Eigen::Array2d origin;
//  std::vector<uint8_t> data;
  cv::Mat data;
  Eigen::Array2i shape;

  Map(const double& reso, const Eigen::Array2d& ori, const cv::Mat& map) 
  : resolution(reso), origin(ori), data(map), shape(data.cols, data.rows) {}

//  uint32_t GetFlatIndex(const Eigen::Array2d& coord) {
//    Eigen::Array2d xy_index{(coord - origin) / resolution};
//    return static_cast<uint32_t>(std::round(xy_index.x() + xy_index.y() * shape.y()));
//  }
//  uint8_t GetOccupancyValue(const Eigen::Array2d& coord) {
//    // TODO: have to check if the index exceeds the limits
//    return data[GetFlatIndex(coord)];
//  }
};

struct Scan {
  std::array<double, 3> angle_properties;
  std::array<double, 2> range_properties;
  std::vector<double> ranges;
};

struct PointCloud {
  std::vector<Eigen::Array2d> points;
  size_t size() const {return points.size();}
};

struct DiscretizedPointCloud {
  std::vector<Eigen::Array2i> points;
  size_t size() const {return points.size();}
};

struct Candidate {
  int map_idx;
  double score;
  Eigen::Isometry2d transform;
  DiscretizedPointCloud point_cloud;

  Candidate(const int& i, const double& s, const Eigen::Isometry2d& t,
    const DiscretizedPointCloud& p) : map_idx(i), score(s), transform(t), point_cloud(p) {}
  bool operator<(const Candidate& other) const {return score < other.score;}
  bool operator>(const Candidate& other) const {return score > other.score;}
};

PointCloud ScanToPointCloud(const Scan& scan);
PointCloud TransformPointCloud(const Eigen::Isometry2d& transform, const PointCloud& pc);

class ScanMatcher {
  public:
    ScanMatcher(const Option& option);
    ScanMatcher(const Map& map, const Option& option);
    ~ScanMatcher() {}

    void SetMap(const Map& map);
    bool StartRelocalization(const Scan& scan, Eigen::Isometry2d& transform);

  private:
    bool GenerateInitialCandidates(const PointCloud& pc, const Eigen::Isometry2d& transform,
      std::vector<Candidate>& candidates);
    bool GenerateCandidates(const PointCloud& pc, const Candidate& candidate,
      std::vector<Candidate>& candidates);
    bool ScoreCandidates(std::vector<Candidate>& candidates);
    bool EvaluateCandidate(Candidate& candidate);

    DiscretizedPointCloud DiscretizePointCloud(const PointCloud& pc, const int& map_idx) const;
    
  private:
    const Option option_;
    int current_map_idx_;
    std::vector<SearchParam> search_params_;
    std::vector<Map> map_stack_;
    std::vector<Candidate> candidates_;
}; // class

} // namespace
