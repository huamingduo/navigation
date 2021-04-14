#include <chrono>
#include <unordered_set>

#include "scan_matcher.h"

namespace scan_matching {

PointCloud ScanToPointCloud(const Scan& scan) {
  PointCloud point_cloud;
  double angle{scan.angle_properties[0]};
  double prev_range{scan.ranges[0]};
  for (const auto& range : scan.ranges) {
    if (angle > scan.angle_properties[1]) {
      break;
    }
    if (range < scan.range_properties[0] || range > scan.range_properties[1]) {
      angle += scan.angle_properties[2];
      continue;
    }
    if (std::abs(prev_range - range) > 0.3) {
      angle += scan.angle_properties[2];
      prev_range = range;
      continue;
    }
    point_cloud.points.push_back(Eigen::Array2d(range * std::cos(angle), range * std::sin(angle)));
    angle += scan.angle_properties[2];
    prev_range = range;
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

ScanMatcher::ScanMatcher(const Option& option) : option_(option), current_map_idx_(option.map_stack_depth-1) {}

ScanMatcher::ScanMatcher(const Map& map, const Option& option)
    : option_(option), current_map_idx_(option.map_stack_depth-1) {
  // map
  map_stack_.reserve(option_.map_stack_depth);
  map_stack_.push_back(map);
  const cv::Mat kernal = cv::getStructuringElement(cv::MORPH_RECT,
    cv::Size(option.kernal_size, option.kernal_size));
  for (int i=0; i<option_.map_stack_depth-1; ++i) {
    cv::Mat grid;
    map_stack_[i].data.copyTo(grid);
    cv::erode(grid, grid, kernal);
    cv::resize(grid, grid, cv::Size(),
      option_.shrink_scale, option_.shrink_scale, cv::INTER_NEAREST);
    map_stack_.push_back(Map(map_stack_[i].resolution/option_.shrink_scale,
      map_stack_[i].origin, grid));
  }

#ifdef DEBUG
  cv::namedWindow("debug", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
  for (const auto& item : map_stack_) {
    cv::imshow("debug", item.data);
    cv::waitKey(0);
  }
#endif

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

#ifdef DEBUG
  int idx{0};
  for (const auto& item : search_params_) {
    std::cout << "------------- map " << idx << "---------------" << std::endl;
    std::cout << "linear_search_window: " << item.linear_search_window << std::endl;
    std::cout << "angular_search_window: " << item.angular_search_window << std::endl;
    std::cout << "linear_search_step: " << item.linear_search_step << std::endl;
    std::cout << "angular_search_step: " << item.angular_search_step << std::endl;
    ++idx;
  }
#endif
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

bool ScanMatcher::GenerateInitialCandidates(const PointCloud& pc, const Eigen::Isometry2d& transform,
    std::vector<Candidate>& candidates) {
  candidates.clear();
  const int map_idx{option_.map_stack_depth-1};
  const Map map{map_stack_[map_idx]};
  const SearchParam param{search_params_[map_idx]};
  // TODO: boundary check
  for (double angle = -param.angular_search_window; angle <= param.angular_search_window;
      angle += param.angular_search_step) {
    for (double x_offset = -param.linear_search_window; x_offset <= param.linear_search_window;
        x_offset += param.linear_search_step) {
      for (double y_offset = -param.linear_search_window; y_offset <= param.linear_search_window;
          y_offset += param.linear_search_step) {
        Eigen::Isometry2d estimate{transform};
        estimate.translate(Eigen::Vector2d(x_offset, y_offset));
        estimate.rotate(angle);
        const DiscretizedPointCloud temp{DiscretizePointCloud(
          TransformPointCloud(estimate, pc), map_idx)};
        candidates.push_back(Candidate(map_idx, 0., estimate, temp));
      }
    }
  }
  return true;
}

bool ScanMatcher::GenerateCandidates(const PointCloud& pc, const Candidate& candidate,
    std::vector<Candidate>& candidates) {
  if (candidate.map_idx < 1) {
    return false;
  }
  const int map_idx{candidate.map_idx - 1};
  const Map map{map_stack_[map_idx]};
  const SearchParam param{search_params_[map_idx]};
  // TODO: boundary check
  for (double angle = -param.angular_search_window; angle <= param.angular_search_window;
      angle += param.angular_search_step) {
    for (double x_offset = -param.linear_search_window; x_offset <= param.linear_search_window;
        x_offset += param.linear_search_step) {
      for (double y_offset = -param.linear_search_window; y_offset <= param.linear_search_window;
          y_offset += param.linear_search_step) {
        Eigen::Isometry2d estimate{candidate.transform};
        estimate.translate(Eigen::Vector2d(x_offset, y_offset));
        estimate.rotate(angle);
        const DiscretizedPointCloud temp{DiscretizePointCloud(
          TransformPointCloud(estimate, pc), map_idx)};
        candidates.push_back(Candidate(map_idx, 0., estimate, temp));
      }
    }
  }
  return true;
}

bool ScanMatcher::ScoreCandidates(std::vector<Candidate>& candidates) {
  for (auto& candidate : candidates) {
    EvaluateCandidate(candidate);
  }
  std::sort(candidates.begin(), candidates.end());
  return true;
}

bool ScanMatcher::EvaluateCandidate(Candidate& candidate) {
  const Map map{map_stack_[candidate.map_idx]};
  candidate.score = 0.;
  for (const auto& point : candidate.point_cloud.points) {
    if (point.x() < 0 || point.x() >= map.shape.x()) {
      candidate.score += 0.1;
      continue;
    }
    if (point.y() < 0 || point.y() >= map.shape.y()) {
      candidate.score += 0.1;
      continue;
    }
    const cv::Point pixel{point.x(), point.y()};
//    if (map.data.at<uchar>(pixel) >= 110 && map.data.at<uchar>(pixel) <= 140) {
//      continue;
//    }
//    candidate.score += ((255.-map.data.at<uchar>(pixel)) / 255.);
    const auto value = map.data.at<uchar>(pixel);
    constexpr double scale{(0.9-0.1)/255.};
    if (value == 128) {
      candidate.score += 0.1;
    } else if (value <= 127 && value >= 0) {
      candidate.score += (scale*(255-value)+0.1);
    } else if (value <= 255 && value >= 129) {
      candidate.score += (scale*(255-value)+0.1);
    }
  }
  candidate.score /= static_cast<double>(candidate.point_cloud.points.size());
  const Eigen::Matrix2d temp{candidate.transform.rotation()};
  const double angle{std::atan2(temp(1,0), temp(0,0))};
  candidate.score *= std::exp(-std::pow(
    1. * std::hypot(candidate.transform.translation().x(), candidate.transform.translation().y()) +
    1. * std::abs(angle),2));
  return true;
}

bool ScanMatcher::StartRelocalization(const Scan& scan, Eigen::Isometry2d& transform) {
  const auto start{std::chrono::steady_clock::now()};
  auto temp{start};
  const PointCloud init_pc{ScanToPointCloud(scan)};
  std::chrono::duration<double> duration{std::chrono::steady_clock::now()-temp};
  std::cout << "=================================================================" << std::endl;
#ifdef DEBUG
  temp = std::chrono::steady_clock::now();
  std::cout << "time for converting scan message: " << duration.count() << " seconds" << std::endl;
  std::cout << "-----------------------------------------------------------------"
    << std::endl << std::endl;
#endif
  std::vector<Candidate> candidates;
  if (!GenerateInitialCandidates(init_pc, transform, candidates)) {
    std::cerr << "cannot generate initial candidates" << std::endl;
    return false;
  }
#ifdef DEBUG
  duration = std::chrono::steady_clock::now() - temp;
  temp = std::chrono::steady_clock::now();
  std::cout << "time for generating initial candidates: " << duration.count()
    << " seconds" << std::endl;
  std::cout << "-----------------------------------------------------------------"
    << std::endl << std::endl;
#endif
  ScoreCandidates(candidates);
#ifdef DEBUG
  duration = std::chrono::steady_clock::now() - temp;
  temp = std::chrono::steady_clock::now();
  std::cout << "time for scoring initial candidates: " << duration.count()
    << " seconds" << std::endl;
  std::cout << "-----------------------------------------------------------------"
    << std::endl << std::endl;
#endif
  double min_score{option_.min_score};
  int count{0};
  while (!candidates.empty()) {
    Candidate current{candidates.back()};
    candidates.pop_back();
    if (current.score < min_score) {
      continue;
    }
    if (current.map_idx == 0) {
      transform = current.transform;
      min_score = current.score;
      continue;
    }
    if (!GenerateCandidates(init_pc, current, candidates)) {
      continue;
    }
    ScoreCandidates(candidates);
#ifdef DEBUG
    duration = std::chrono::steady_clock::now() - temp;
    temp = std::chrono::steady_clock::now();
    std::cout << "time for iteration " << count << ": " << duration.count()
      << " seconds" << std::endl;
    std::cout << "current candidate map_idx: " << current.map_idx << std::endl;
    std::cout << "current candidate score: " << current.score << std::endl;
    std::cout << "-----------------------------------------------------------------"
      << std::endl << std::endl;
    ++count;
#endif
  }
  duration = std::chrono::steady_clock::now() - start;
  std::cout << "total time used: " << duration.count() << " seconds" << std::endl;
  std::cout << "with score: " << min_score << std::endl;
  std::cout << "=================================================================" << std::endl;
  // if (min_score > option_.min_score) {
    return true;
  // } else {
    // return false;
  // }
}

void ScanMatcher::SetMap(const Map& map) {
  // map
  map_stack_.clear();
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

#ifdef DEBUG
  int idx{0};
  for (const auto& item : map_stack_) {
    cv::imwrite("/home/robot/base_line/map_" + std::to_string(idx) + ".png", item.data);
    ++idx;
  }
#endif
}

} // namespace
