#include <chrono>
#include <unordered_set>

#include "scan_matcher.h"

namespace scan_matching {

PointCloud ScanToPointCloud(const Scan& scan) {
  PointCloud point_cloud;
  double angle{scan.angle_properties[0]};
  for (const auto& range : scan.ranges) {
    if (angle > scan.angle_properties[1]) {
      break;
    }
    if (range < scan.range_properties[0] || range > scan.range_properties[1]) {
      angle += scan.angle_properties[2];
      continue;
    }
    point_cloud.points.push_back(Eigen::Array2d(range * std::cos(angle), range * std::sin(angle)));
    angle += scan.angle_properties[2];
  }
  return point_cloud;
}

PointCloud TransformPointCloud(const Eigen::Isometry2d& transform, const PointCloud& pc) {
  PointCloud transformed;
  for (const auto& point : pc.points) {
    transformed.points.push_back(transform * point);
  }
  return transformed;
}


ScanMatcher::ScanMatcher(const Option& option)
    : option_(option), current_map_idx_(option.map_stack_depth-1) {}

void ScanMatcher::SetMap(const Map& map) {
  // map
  map_stack_.reserve(option_.map_stack_depth);
  map_stack_.push_back(map);
  const cv::Mat kernal = cv::getStructuringElement(cv::MORPH_RECT,
    cv::Size(option_.kernal_size, option_.kernal_size));
  for (int i=0; i<option_.map_stack_depth-1; ++i) {
    cv::Mat grid;
    map_stack_[i].data.copyTo(grid);
    cv::erode(grid, grid, kernal);
    cv::resize(grid, grid, cv::Size(),
      option_.shrink_scale, option_.shrink_scale, cv::INTER_NEAREST);
    map_stack_.push_back(Map(map_stack_[i].resolution/option_.shrink_scale,
      map_stack_[i].origin, grid));
  }

  // search param
  search_params_.reserve(option_.map_stack_depth);
  for (int i=0; i<option_.map_stack_depth; ++i) {
    const double resolution{map_stack_[option_.map_stack_depth-i-1].resolution};
    search_params_.push_back(SearchParam(
      option_.linear_search_window * std::pow(option_.shrink_scale, i),
      option_.angular_search_window * std::pow(option_.shrink_scale, i),
      resolution, M_PI/2.*resolution));
  }
  std::reverse(search_params_.begin(), search_params_.end());
}

bool ScanMatcher::StartRelocalization(const Scan& scan, Eigen::Isometry2d& transform) {
  const auto start{std::chrono::steady_clock::now()};
  auto temp{start};
  const PointCloud init_pc{ScanToPointCloud(scan)};

  std::chrono::duration<double> duration{std::chrono::steady_clock::now()-temp};
  std::cout << "=================================================================" << std::endl;
#ifdef DEBUG
  std::cout << "time for converting scan message: " << duration.count() << " seconds" << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;
#endif
  double score{0.};
  while (current_map_idx_ >= 0) {
#ifdef DEBUG
    std::cout << std::endl << "time statistics for iteration " <<
      (option_.map_stack_depth - current_map_idx_) << std::endl;
    std::cout << "initial guess: " << std::endl << transform.matrix() << std::endl;
#endif
    Candidate init_candidate{current_map_idx_, 0., transform, DiscretizedPointCloud()};
    if (!GenerateCandidates(init_pc, init_candidate)) {
      std::cerr << "generate candidates failed" << std::endl;
      return false;
    }
#ifdef DEBUG
    duration = std::chrono::steady_clock::now() - temp;
    temp = std::chrono::steady_clock::now();
    std::cout << "time for generating candidates: " << duration.count() << " seconds" << std::endl;
#endif
    score = ScoreCandidates(transform);
#ifdef DEBUG
    duration = std::chrono::steady_clock::now() - temp;
    temp = std::chrono::steady_clock::now();
    std::cout << "time for scoring candidates: " << duration.count() << " seconds" << std::endl;
    std::cout << "estimate result: " << std::endl << transform.matrix() << std::endl;
    std::cout << "with score: " << score << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;

    cv::Mat image;
    map_stack_[current_map_idx_].data.copyTo(image);
    cv::cvtColor(image, image, CV_GRAY2BGR);
    
    const DiscretizedPointCloud dpc = DiscretizePointCloud(
      TransformPointCloud(transform, init_pc), current_map_idx_);
#endif

    --current_map_idx_;
  }

  current_map_idx_ = option_.map_stack_depth - 1;

  duration = std::chrono::steady_clock::now() - start;
  temp = std::chrono::steady_clock::now();
  std::cout << std::endl << "total correlative scan matching time spend: "
    << duration.count() <<  " seconds" << std::endl;
  if (score < 0.4) {
    std::cout << "relocalize failed with score " << score << std::endl;
  } else {
    std::cout << "relocalize succeeded with score " << score << std::endl;
  }
  std::cout << "-----------------------------------------------------------------"
    << std::endl << std::endl;
 
  duration = std::chrono::steady_clock::now() - start;
  std::cout << "total scan matching time : " << duration.count() << " seconds" << std::endl;
  std::cout << "=================================================================" << std::endl;

  return true;
}

bool ScanMatcher::GenerateCandidates(const PointCloud& pc, const Candidate& candidate) {
  candidates_.clear();
  const Map map = map_stack_[candidate.map_idx];
  const SearchParam param = search_params_[candidate.map_idx];
  // TODO: boundary check
  for (double angle = -param.angular_search_window; angle <= param.angular_search_window;
      angle += param.angular_search_step) {
    for (double x_offset = -param.linear_search_window; x_offset <= param.linear_search_window;
        x_offset += param.linear_search_step) {
      for (double y_offset = -param.linear_search_window; y_offset <= param.linear_search_window;
          y_offset += param.linear_search_step) {
        Eigen::Isometry2d transform{candidate.transform};
        transform.translate(Eigen::Vector2d(x_offset, y_offset));
        transform.rotate(angle);
        const DiscretizedPointCloud temp{DiscretizePointCloud(
          TransformPointCloud(transform, pc), candidate.map_idx)};
        candidates_.push_back(Candidate(current_map_idx_, 0., transform, temp));
      }
    }
  }
  return true;
}

double ScanMatcher::ScoreCandidates(Eigen::Isometry2d& transform) {
  double score{0.};
  int idx{0}, res{0};
  for (auto& candidate : candidates_) {
    const double temp = EvaluateCandidate(candidate.point_cloud);
    candidate.score = temp;
    if (temp > score) {
      score = temp;
      res = idx;
    }
    ++idx;
  }
  transform = candidates_[res].transform;
  return score;
}

DiscretizedPointCloud ScanMatcher::DiscretizePointCloud(
    const PointCloud& pc, const int& map_idx) const {
  DiscretizedPointCloud dpc;
  const Map map{map_stack_[map_idx]};
  std::unordered_set<int> voxel_set;
  for (const auto& point : pc.points) {
    const int x{static_cast<int>(std::round((point.x() - map.origin.x()) / map.resolution))};
    const int y{static_cast<int>(std::round((point.y() - map.origin.y()) / map.resolution))};
    const auto res = voxel_set.insert(x + y * map.shape.x());
    if (!res.second) {
      continue;
    }
    dpc.points.push_back(Eigen::Array2i(x, map.shape.y() - y));
  }
  return dpc;
}

double ScanMatcher::EvaluateCandidate(const DiscretizedPointCloud& pc) const {
  double score{0.};
  const Map map = map_stack_[current_map_idx_];
#ifdef ANIMATE
  cv::Mat image;
  map.data.copyTo(image);
  cv::cvtColor(image, image, CV_GRAY2BGR);
#endif
  for (const auto& point : pc.points) {
    if (point.x() < 0 || point.x() >= map.shape.x()) {
      continue;
    }
    if (point.y() < 0 || point.y() >= map.shape.y()) {
      continue;
    }
    const cv::Point pixel{point.x(), point.y()};
#ifdef ANIMATE
    cv::circle(image, pixel, 1, cv::Scalar(0,255,0), -1);
#endif
    if (map.data.at<uchar>(pixel) >= 100 && map.data.at<uchar>(pixel) <= 150) {
      continue;
    }
    score += ((255.-map.data.at<uchar>(pixel)) / 255.);
  }
  score /= pc.points.size();
#ifdef ANIMATE
  cv::imshow("debug", image);
  cv::waitKey(3);
#endif
  return score;
}

} // namespace
