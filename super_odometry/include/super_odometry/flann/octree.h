//
// Created by ubuntu on 2020/7/12.
//

#ifndef OCTREE_H
#define OCTREE_H

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <unordered_map>
#include <vector>

#include "nanoflann.h"

namespace nanoflann
{

namespace traits
{
template <typename PointT, int D>
struct access
{
};

template <typename PointT>
struct access<PointT, 0>
{
  static float get(const PointT& p)
  {
    return p.x;
  }
};

template <typename PointT>
struct access<PointT, 1>
{
  static float get(const PointT& p)
  {
    return p.y;
  }
};

template <typename PointT>
struct access<PointT, 2>
{
  static float get(const PointT& p)
  {
    return p.z;
  }
};
}  // namespace traits

template <int D, typename PointT>
inline float get(const PointT& p)
{
  return traits::access<PointT, D>::get(p);
}

template <typename PointT>
struct L1Distance
{
  static inline float compute(const PointT& p, const PointT& q)
  {
    float diff1 = get<0>(p) - get<0>(q);
    float diff2 = get<1>(p) - get<1>(q);
    float diff3 = get<2>(p) - get<2>(q);

    return std::abs(diff1) + std::abs(diff2) + std::abs(diff3);
  }

  static inline float norm(float x, float y, float z)
  {
    return std::abs(x) + std::abs(y) + std::abs(z);
  }

  static inline float sqr(float r)
  {
    return r;
  }

  static inline float sqrt(float r)
  {
    return r;
  }
};  // struct L1Distance

template <typename PointT>
struct L2Distance
{
  static inline float compute(const PointT& p, const PointT& q)
  {
    float diff1 = get<0>(p) - get<0>(q);
    float diff2 = get<1>(p) - get<1>(q);
    float diff3 = get<2>(p) - get<2>(q);

    return std::pow(diff1, 2) + std::pow(diff2, 2) + std::pow(diff3, 2);
  }

  static inline float norm(float x, float y, float z)
  {
    return std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2);
  }

  static inline float sqr(float r)
  {
    return r * r;
  }

  static inline float sqrt(float r)
  {
    return std::sqrt(r);
  }
};  // struct L2Distance

template <typename PointT>
struct MaxDistance
{
  static inline float compute(const PointT& p, const PointT& q)
  {
    float diff1 = std::abs(get<0>(p) - get<0>(q));
    float diff2 = std::abs(get<1>(p) - get<1>(q));
    float diff3 = std::abs(get<2>(p) - get<2>(q));

    float maximum = diff1;
    if (diff2 > maximum) maximum = diff2;
    if (diff3 > maximum) maximum = diff3;

    return maximum;
  }

  static inline float norm(float x, float y, float z)
  {
    float maximum = std::abs(x);
    if (std::abs(y) > maximum) maximum = std::abs(y);
    if (std::abs(z) > maximum) maximum = std::abs(z);

    return maximum;
  }

  static inline float sqr(float r)
  {
    return std::abs(r);
  }

  static inline float sqrt(float r)
  {
    return std::abs(r);
  }
};  // struct MaxDistance

struct OctreeParams
{
public:
  OctreeParams(size_t bucketSize = 32, bool copyPoints = false, float minExtent = 0.f)
    : bucketSize(bucketSize), copyPoints(copyPoints), minExtent(minExtent)
  {
  }

public:
  size_t bucketSize;
  bool copyPoints;
  float minExtent;
};

/** \brief Index-based Octree implementation offering different queries and insertion/removal of points.
 *
 * The index-based Octree uses a successor relation and a startIndex in each Octant to improve runtime
 * performance for radius queries. The efficient storage of the points by relinking list elements
 * bases on the insight that children of an Octant contain disjoint subsets of points inside the Octant and
 * that we can reorganize the points such that we get an continuous single connect list that we can use to
 * store in each octant the start of this list.
 *
 * Special about the implementation is that it allows to search for neighbors with arbitrary p-norms, which
 * distinguishes it from most other Octree implementations.
 *
 * We decided to implement the Octree using a template for points and containers. The container must have an
 * operator[], which allows to access the points, and a size() member function, which allows to get the size of the
 * container. For the points, we used an access trait to access the coordinates inspired by boost.geometry.
 * The implementation already provides a general access trait, which expects to have public member variables x,y,z.
 *
 * if you use the implementation or ideas from the corresponding paper in your academic work, it would be nice if you
 * cite the corresponding paper:
 *
 *    J. Behley, V. Steinhage, A.B. Cremers. Efficient Radius Neighbor Search in Three-dimensional Point Clouds,
 *    Proc. of the IEEE International Conference on Robotics and Automation (ICRA), 2015.
 *
 * In future, we might add also other neighbor queries and implement the removal and adding of points.
 *
 * \version 0.1-icra
 *
 * \author behley
 */

template <typename PointT, typename ContainerT = std::vector<PointT> >
class Octree
{
public:
  Octree();
  ~Octree();

  Octree(Octree&) = delete;
  Octree& operator=(const Octree& oct) = delete;

  /// \brief initialize octree with all points
  /// \param pts
  /// \param params
  void initialize(const ContainerT& pts, const OctreeParams& params = OctreeParams());

  /// \brief initialize octree only from pts that are inside indices
  /// \param pts
  /// \param indices
  /// \param params
  void initialize(const ContainerT& pts, const std::vector<size_t>& indices,
                  const OctreeParams& params = OctreeParams());

  /// \brief remove all data inside the octree
  void clear();

  template <typename Distance>
  void radiusNeighbors(const PointT& query, float radius, std::vector<size_t>& resultIndices) const;

  template <typename Distance>
  void radiusNeighbors(const PointT& query, float radius, std::vector<size_t>& resultIndices,
                       std::vector<float>& distances) const;

  template <typename Distance>
  void radiusNeighbors(const PointT& query, float radius,
                       std::vector<std::pair<size_t, float> >& resultIndicesAnddists) const;

  template <typename Distance>
  bool knnNeighbors(const PointT& query, const size_t num_closest, size_t* out_indices, float* out_distance_sq) const;

  template <typename Distance>
  size_t findNeighbor(const PointT& query, float minDistance = -1) const;

protected:
  struct Octant
  {
  public:
    Octant();
    ~Octant();

    bool isLeaf;

    // bounding box of the octant needed for overlap and contains tests...
    float x, y, z;  // center
    float extent;   // half of side-length

    size_t start, end;  // start and end in succ_
    size_t size;        // number of points

    Octant* child[8];
  };

protected:
  /// \brief creation of an octant using elements starting at startIdx.
  /// This method reorders the index such that all points are correctly
  /// linked to successors belonging to the same octant.
  ///
  /// \param x        center coordinates of octant
  /// \param y
  /// \param z
  /// \param extent   extent of octant
  /// \param startIdx first index of points inside octant
  /// \param endIdx   last index of points inside octant
  /// \param size     number of points in octant
  /// \return
  Octant* createOctant(float x, float y, float z, float extent, size_t startIdx, size_t endIdx, size_t size);

  template <typename Distance>
  bool findNeighbor(const Octant* octant, const PointT& query, float minDistance, float& maxDistance,
                    size_t& resultIndex) const;

  template <typename Distance>
  void radiusNeighbors(const Octant* octant, const PointT& query, float radius, float sqrtRadius,
                       std::vector<size_t>& resultIndices) const;

  template <typename Distance>
  void radiusNeighbors(const Octant* octant, const PointT& query, float radius, float sqrtRadius,
                       std::vector<size_t>& resultIndices, std::vector<float>& distances) const;

  template <typename Distance>
  void radiusNeighbors(const Octant* octant, const PointT& query, float radius, float sqrtRadius,
                       std::vector<std::pair<size_t, float> >& resultIndicesAndDists) const;

  template <typename Distance>
  bool knnNeighbors(const Octant* octant, const PointT& query, KNNResult<float>& result) const;
  /// \brief  test if search ball S(q,r) overlaps with octant
  /// \tparam Distance
  /// \param query        query point
  /// \param radius       "sqaured" radius
  /// \param sqRaius      pointer to octant
  /// \param o
  /// \return             if search
  template <typename Distance>
  static bool overlaps(const PointT& query, float radius, float sqRaius, const Octant* o);

  /// \brief test if search ball S(q, r) contains octant
  /// \tparam Distance
  /// \param query
  /// \param sqRaius
  /// \param octant
  /// \return
  template <typename Distance>
  static bool contains(const PointT& query, float sqRaius, const Octant* octant);

  /// \brief test if search ball S(q,r) is completely inside octant
  /// \tparam Distance
  /// \param query    query point
  /// \param radius   radius r
  /// \param octant   point to octant
  /// \return
  template <typename Distance>
  static bool inside(const PointT& query, float radius, const Octant* octant);

protected:
  OctreeParams params_;
  Octant* root_;

  const ContainerT* data_;

  std::vector<size_t> successors_;  // single connected list of next point indices...
};

// class Octree

template <typename PointT, typename ContainerT>
Octree<PointT, ContainerT>::Octant::Octant()
  : isLeaf(true), x(0.f), y(0.f), z(0.f), extent(0.f), start(0), end(0), size(0)
{
  memset(&child, 0, 8 * sizeof(Octant*));
}
template <typename PointT, typename ContainerT>
Octree<PointT, ContainerT>::Octant::~Octant()
{
  for (size_t i = 0; i < 8; i++) delete (child[i]);
}

template <typename PointT, typename ContainerT>
Octree<PointT, ContainerT>::Octree() : root_(0), data_(0)
{
}
template <typename PointT, typename ContainerT>
Octree<PointT, ContainerT>::~Octree()
{
  delete (root_);
  if (params_.copyPoints) delete (data_);
}
template <typename PointT, typename ContainerT>
void Octree<PointT, ContainerT>::initialize(const ContainerT& pts, const OctreeParams& params)
{
  clear();

  params_ = params;
  if (params.copyPoints)
    data_ = new ContainerT(pts);
  else
    data_ = &pts;

  const size_t N = pts.size();
  successors_ = std::vector<size_t>(N);

  float min[3], max[3];
  min[0] = get<0>(pts[0]);
  min[1] = get<1>(pts[0]);
  min[2] = get<2>(pts[0]);
  max[0] = min[0];
  max[1] = min[1];
  max[2] = min[2];

  for (size_t i = 0; i < N; i++)
  {
    successors_[i] = i + 1;
    const PointT& p = pts[i];
    if (get<0>(pts[i]) < min[0]) min[0] = get<0>(p);
    if (get<1>(pts[i]) < min[1]) min[1] = get<1>(p);
    if (get<2>(pts[i]) < min[2]) min[2] = get<2>(p);
    if (get<0>(pts[i]) > max[0]) max[0] = get<0>(p);
    if (get<0>(pts[i]) > max[1]) max[1] = get<1>(p);
    if (get<0>(pts[i]) > max[2]) min[2] = get<2>(p);
  }

  float ctr[3] = {min[0], min[1], min[2]};
  float maxextent = 0.5f * (max[0] - min[0]);

  ctr[0] += maxextent;
  for (size_t i = 1; i < 3; ++i)
  {
    float extant = 0.5f * (max[i] - min[i]);
    ctr[i] += extant;
    if (extant > maxextent) maxextent = extant;
  }

  root_ = createOctant(ctr[0], ctr[1], ctr[2], maxextent, 0, N - 1, N);
}
template <typename PointT, typename ContainerT>
void Octree<PointT, ContainerT>::initialize(const ContainerT& pts, const std::vector<size_t>& indices,
                                            const OctreeParams& params)
{
  clear();
  params_ = params;

  if (params_.copyPoints)
    data_ = new ContainerT(pts);
  else
    data_ = &pts;

  const size_t N = pts.size();
  successors_ = std::vector<size_t>(N);
  if (indices.empty()) return;

  size_t lastIdx = indices[0];

  float min[3], max[3];
  min[0] = get<0>(pts[lastIdx]);
  min[1] = get<1>(pts[lastIdx]);
  min[2] = get<2>(pts[lastIdx]);

  max[0] = min[0];
  max[1] = min[1];
  max[2] = min[2];

  for (size_t i = 1; i < indices.size(); i++)
  {
    size_t idx = indices[i];
    successors_[lastIdx] = idx;

    const PointT& p = pts[idx];

    if (get<0>(p) < min[0]) min[0] = get<0>(p);
    if (get<1>(p) < min[1]) min[1] = get<1>(p);
    if (get<2>(p) < min[2]) min[2] = get<2>(p);

    if (get<0>(p) > max[0]) max[0] = get<0>(p);
    if (get<1>(p) > max[1]) max[1] = get<1>(p);
    if (get<2>(p) > max[2]) max[2] = get<2>(p);

    lastIdx = idx;
  }

  float ctr[3] = {min[0], min[1], min[2]};
  float maxextant = 0.5f * (max[0] - min[0]);
  ctr[0] += maxextant;

  for (int i = 1; i < 3; i++)
  {
    float extant = 0.5f * (max[i] - min[i]);
    ctr[i] += extant;
    if (extant > maxextant) maxextant = extant;
  }

  root_ = createOctant(ctr[0], ctr[1], ctr[2], maxextant, indices[0], lastIdx, indices.size());
}
template <typename PointT, typename ContainerT>
void Octree<PointT, ContainerT>::clear()
{
  delete (root_);
  if (params_.copyPoints) delete (data_);
  root_ = nullptr;
  data_ = nullptr;
  successors_.clear();
}
template <typename PointT, typename ContainerT>
template <typename Distance>
void Octree<PointT, ContainerT>::radiusNeighbors(const PointT& query, float radius,
                                                 std::vector<size_t>& resultIndices) const
{
  resultIndices.clear();
  if (root_ == 0) return;

  float sqrRadius = Distance::sqr(radius);
  radiusNeighbors<Distance>(root_, query, radius, sqrRadius, resultIndices);
}

template <typename PointT, typename ContainerT>
template <typename Distance>
void Octree<PointT, ContainerT>::radiusNeighbors(const PointT& query, float radius, std::vector<size_t>& resultIndices,
                                                 std::vector<float>& distances) const
{
  resultIndices.clear();
  distances.clear();
  if (root_ == nullptr) return;

  float sqrRadius = Distance::sqr(radius);
  radiusNeighbors<Distance>(root_, query, radius, sqrRadius, resultIndices, distances);
}

template <typename PointT, typename ContainerT>
template <typename Distance>
void Octree<PointT, ContainerT>::radiusNeighbors(const PointT& query, float radius,
                                                 std::vector<std::pair<size_t, float> >& resultIndicesAnddists) const
{

  resultIndicesAnddists.clear();
  if (root_ == 0) return;

  float sqrRadius = Distance::sqr(radius);
  // TODO:
  radiusNeighbors<Distance>(root_, query, radius, sqrRadius, resultIndicesAnddists);
}

template <typename PointT, typename ContainerT>
template <typename Distance>
bool Octree<PointT, ContainerT>::knnNeighbors(const PointT& query, const size_t num_closest, size_t* out_indices,
                                              float* out_distance_sq) const

{
  if (root_ == nullptr) return false;

  nanoflann::KNNResult<float, size_t> result(num_closest);
  result.init(out_indices, out_distance_sq);
  knnNeighbors<Distance>(root_, query, result);
  return result.full();
}

template <typename PointT, typename ContainerT>
template <typename Distance>
size_t Octree<PointT, ContainerT>::findNeighbor(const PointT& query, float minDistance) const
{
  float maxDistance = std::numeric_limits<float>::infinity();
  size_t resultIndx = std::numeric_limits<size_t>::max();

  if (root_ == nullptr) return resultIndx;

  findNeighbor<Distance>(root_, query, minDistance, maxDistance, resultIndx);

  return resultIndx;
}

template <typename PointT, typename ContainerT>
typename Octree<PointT, ContainerT>::Octant* Octree<PointT, ContainerT>::createOctant(float x, float y, float z,
                                                                                      float extent, size_t startIdx,
                                                                                      size_t endIdx, size_t size)
{
  Octant* octant = new Octant;

  octant->isLeaf = true;
  octant->x = x;
  octant->y = y;
  octant->z = z;
  octant->extent = extent;

  octant->start = startIdx;
  octant->end = endIdx;
  octant->size = size;

  static const float factor[] = {-0.5f, 0.5f};

  if (size > params_.bucketSize && extent > 2 * params_.minExtent)
  {
    octant->isLeaf = false;
    const ContainerT& points = *data_;
    std::vector<size_t> childStarts(8, 0);
    std::vector<size_t> childEnds(8, 0);
    std::vector<size_t> childSizes(8, 0);
    if (size < 100000)
    {
      // re-link disjoint child subsets ...
      size_t idx = startIdx;
      for (size_t i = 0; i < size; ++i)
      {
        const PointT& p = points[idx];
        int mortonCode = 0;
        if (get<0>(p) > x) mortonCode |= 1;
        if (get<1>(p) > y) mortonCode |= 2;
        if (get<2>(p) > z) mortonCode |= 4;

        if (childSizes[mortonCode] == 0)
          childStarts[mortonCode] = idx;
        else
          successors_[childEnds[mortonCode]] = idx;
        childSizes[mortonCode] += 1;

        childEnds[mortonCode] = idx;
        idx = successors_[idx];
      }
    }
    else
    {
      // step1: 将点云数据进行分组,分出四组吧
      std::vector<size_t> group_start;
      std::vector<size_t> group_end;
      std::vector<size_t> group_size;
      constexpr size_t group_num = 4;

      size_t ele_size = (size + group_num - 1) / group_num;
      // 分组
      {
        size_t idx = startIdx;
        for (size_t i = 0; i < 4; i++)
        {
          group_start.push_back(idx);
          size_t j_end = ele_size;
          if (size - ele_size * i < ele_size) j_end = size - ele_size * i;

          group_size.push_back(j_end);

          for (size_t j = 0; j < j_end - 1; j++)
          {
            idx = successors_[idx];
          }
          group_end.push_back(idx);

          idx = successors_[idx];
        }
      }

      // 开始合并结果
      // step2: 每一个group计算多个mortonCode
      {
        std::vector<std::vector<size_t> > mortonCode_start(4);
        std::vector<std::vector<size_t> > mortonCode_end(4);
        std::vector<std::vector<size_t> > mortonCode_size(4);
        for (int i = 0; i < 4; i++)
        {
          mortonCode_start[i].resize(8);
          mortonCode_end[i].resize(8);
          mortonCode_size[i].assign(8, 0);
        }

        auto compute_func = [&](const tbb::blocked_range<int>& range) {
          for (int i = range.begin(); i != range.end(); ++i)
          {
            size_t idx = group_start[i];
            for (int j = 0; j < static_cast<int>(group_size[i]); j++)
            {

              const PointT& p = points[idx];
              int mortonCode = 0;
              if (get<0>(p) > x) mortonCode |= 1;
              if (get<1>(p) > y) mortonCode |= 2;
              if (get<2>(p) > z) mortonCode |= 4;

              if (mortonCode_size[i][mortonCode] == 0)
                mortonCode_start[i][mortonCode] = idx;
              else
                successors_[mortonCode_end[i][mortonCode]] = idx;
              mortonCode_size[i][mortonCode] += 1;
              mortonCode_end[i][mortonCode] = idx;

              idx = successors_[idx];
            }
          }
        };
        tbb::blocked_range<int> range(0, 4);
        tbb::parallel_for(range, compute_func);

        for (int i = 0; i < 8; i++)
        {
          for (int j = 0; j < 4; j++)
          {
            if (mortonCode_size[j][i] == 0) continue;
            if (childSizes[i] == 0)
            {
              childStarts[i] = mortonCode_start[j][i];
              childEnds[i] = mortonCode_end[j][i];
              childSizes[i] = mortonCode_size[j][i];
            }
            else
            {
              successors_[childEnds[i]] = mortonCode_start[j][i];
              childEnds[i] = mortonCode_end[j][i];
              childSizes[i] += mortonCode_size[j][i];
            }
          }
        }
      }
    }
#if 0
    {
      float childExtent = 0.5f * extent;
      bool firsttime = true;
      int lastChildIdx = 0;
      // 并行构建8个子节点
      auto compute_func = [&](const tbb::blocked_range<int>& range) {
        for (int i = range.begin(); i != range.end(); ++i)
        {
          if (childSizes[i] == 0) continue;
          float childX = x + factor[(i & 1) > 0] * extent;
          float childY = y + factor[(i & 2) > 0] * extent;
          float childZ = z + factor[(i & 4) > 0] * extent;

          octant->child[i] =
              createOctant(childX, childY, childZ, childExtent, childStarts[i], childEnds[i], childSizes[i]);
        }
      };

      tbb::blocked_range<int> range(0, 8);
      tbb::parallel_for(range, compute_func);

      for (int i = 0; i < 8; ++i)
      {
        if (childSizes[i] == 0) continue;

        if (firsttime)
          octant->start = octant->child[i]->start;
        else
          successors_[octant->child[lastChildIdx]->end] = octant->child[i]->start;  // 保证一个大octant是连续的

        lastChildIdx = i;
        octant->end = octant->child[i]->end;

        firsttime = false;
      }
    }
#else
    float childExtent = 0.5f * extent;
    bool firsttime = true;
    int lastChildIdx = 0;
    for (int i = 0; i < 8; ++i)
    {
      if (childSizes[i] == 0) continue;

      float childX = x + factor[(i & 1) > 0] * extent;
      float childY = y + factor[(i & 2) > 0] * extent;
      float childZ = z + factor[(i & 4) > 0] * extent;

      octant->child[i] = createOctant(childX, childY, childZ, childExtent, childStarts[i], childEnds[i], childSizes[i]);
      if (firsttime)
        octant->start = octant->child[i]->start;
      else
        successors_[octant->child[lastChildIdx]->end] = octant->child[i]->start;  // 保证一个大octant是连续的

      lastChildIdx = i;
      octant->end = octant->child[i]->end;

      firsttime = false;
    }
#endif
  }

  return octant;
}
template <typename PointT, typename ContainerT>
template <typename Distance>
bool Octree<PointT, ContainerT>::findNeighbor(const Octree::Octant* octant, const PointT& query, float minDistance,
                                              float& maxDistance, size_t& resultIndex) const
{
  const ContainerT& points = *data_;

  // 1. first descend to leaf and check in leafs points
  if (octant->isLeaf)
  {
    size_t idx = octant->start;
    float sqrMaxDistance = Distance::sqr(maxDistance);
    float sqrMinDistance = (minDistance < 0) ? minDistance : Distance::sqr(minDistance);

    for (size_t i = 0; i < octant->size; ++i)
    {
      const PointT& p = points[idx];
      float dist = Distance::compute(query, p);
      if (dist > sqrMinDistance && dist < sqrMaxDistance)
      {
        resultIndex = idx;
        sqrMaxDistance = dist;
      }
      idx = successors_[idx];
    }

    maxDistance = Distance::sqrt(sqrMaxDistance);

    return inside<Distance>(query, maxDistance, octant);
  }

  // determine Morton code for each point...
  int mortonCode = 0;
  if (get<0>(query) > octant->x) mortonCode |= 1;
  if (get<1>(query) > octant->y) mortonCode |= 2;
  if (get<2>(query) > octant->z) mortonCode |= 4;

  // 2. if current best point completely inside, just return.
  if (octant->child[mortonCode] != nullptr)
  {
    if (findNeighbor<Distance>(octant->child[mortonCode], query, minDistance, maxDistance, resultIndex)) return true;
  }

  // 3. check adjacent octants for overlap and check these if neccessary
  float sqrMaxDistance = Distance::sqr(maxDistance);
  for (int c = 0; c < 8; ++c)
  {
    if (c == mortonCode) continue;
    if (octant->child[c] == nullptr) continue;
    if (!overlaps<Distance>(query, maxDistance, sqrMaxDistance, octant->child[c])) continue;
    if (findNeighbor<Distance>(octant->child[c], query, minDistance, maxDistance, resultIndex)) return true;
  }

  // all children have been checked... check if point is inside the current octant...
  return inside<Distance>(query, maxDistance, octant);
}
template <typename PointT, typename ContainerT>
template <typename Distance>
void Octree<PointT, ContainerT>::radiusNeighbors(const Octree::Octant* octant, const PointT& query, float radius,
                                                 float sqrtRadius, std::vector<size_t>& resultIndices) const
{
  const ContainerT& points = *data_;
  // if search ball S(q,r) contains octant, simply add point indices
  if (contains<Distance>(query, sqrtRadius, octant))
  {
    size_t idx = octant->start;
    for (size_t i = 0; i < octant->size; i++)
    {
      resultIndices.push_back(idx);
      idx = successors_[idx];
    }

    return;  // early pruning
  }

  if (octant->isLeaf)
  {
    size_t idx = octant->start;
    for (size_t i = 0; i < octant->size; ++i)
    {
      const PointT& p = points[idx];
      float dist = Distance::compute(query, p);
      if (dist < sqrtRadius) resultIndices.push_back(idx);
      idx = successors_[idx];
    }

    return;
  }

  // check whether child nodes are in range
  for (int c = 0; c < 8; ++c)
  {
    if (octant->child[c] == nullptr) continue;
    if (!overlaps<Distance>(query, radius, sqrtRadius, octant->child[c])) continue;
    radiusNeighbors<Distance>(octant->child[c], query, sqrtRadius, resultIndices);
  }
}
template <typename PointT, typename ContainerT>
template <typename Distance>
void Octree<PointT, ContainerT>::radiusNeighbors(const Octree::Octant* octant, const PointT& query, float radius,
                                                 float sqrtRadius, std::vector<size_t>& resultIndices,
                                                 std::vector<float>& distances) const

{
  const ContainerT& points = *data_;

  // if search ball S(q, r) contains octant, simply add point indices and compute distance

  if (contains<Distance>(query, sqrtRadius, octant))
  {
    size_t idx = octant->start;
    for (size_t i = 0; i < octant->size; i++)
    {
      resultIndices.push_back(idx);
      distances.push_back(Distance::compute(query, points[idx]));
      idx = successors_[idx];
    }

    return;  // early pruning
  }

  if (octant->isLeaf)
  {
    size_t idx = octant->start;
    for (size_t i = 0; i < octant->size; i++)
    {
      const PointT& p = points[idx];
      float dist = Distance::compute(query, p);
      if (dist < sqrtRadius)
      {
        resultIndices.push_back(idx);
        distances.push_back(dist);
      }
      idx = successors_[idx];
    }

    return;  // early pruning
  }

  // check whether child nodes are in range
  for (int c = 0; c < 8; ++c)
  {
    if (octant->child[c] == nullptr) continue;
    if (!overlaps<Distance>(query, radius, sqrtRadius, octant->child[c])) continue;
    radiusNeighbors<Distance>(octant->child[c], query, radius, sqrtRadius, resultIndices, distances);
  }
}

template <typename PointT, typename ContainerT>
template <typename Distance>
void Octree<PointT, ContainerT>::radiusNeighbors(const Octree::Octant* octant, const PointT& query, float radius,
                                                 float sqrtRadius,
                                                 std::vector<std::pair<size_t, float> >& resultIndicesAndDists) const
{
  const ContainerT& points = *data_;

  // if search ball S(q, r) contains octant, simply add point indices and compute distance

  if (contains<Distance>(query, sqrtRadius, octant))
  {
    size_t idx = octant->start;
    for (size_t i = 0; i < octant->size; i++)
    {
      float dist = Distance::compute(query, points[idx]);
      resultIndicesAndDists.emplace_back(idx, dist);
      idx = successors_[idx];
    }

    return;  // early pruning
  }

  if (octant->isLeaf)
  {
    size_t idx = octant->start;
    for (size_t i = 0; i < octant->size; i++)
    {
      const PointT& p = points[idx];
      float dist = Distance::compute(query, p);
      if (dist < sqrtRadius)
      {
        resultIndicesAndDists.emplace_back(idx, dist);
      }
      idx = successors_[idx];
    }

    return;  // early pruning
  }

  // check whether child nodes are in range
  for (int c = 0; c < 8; ++c)
  {
    if (octant->child[c] == nullptr) continue;
    if (!overlaps<Distance>(query, radius, sqrtRadius, octant->child[c])) continue;
    radiusNeighbors<Distance>(octant->child[c], query, radius, sqrtRadius, resultIndicesAndDists);
  }
}

template <typename PointT, typename ContainerT>
template <typename Distance>
bool Octree<PointT, ContainerT>::overlaps(const PointT& query, float radius, float sqRaius, const Octree::Octant* o)
{
  float x = get<0>(query) - o->x;
  float y = get<1>(query) - o->y;
  float z = get<2>(query) - o->z;

  x = std::abs(x);
  y = std::abs(y);
  z = std::abs(z);

  float max_dist = radius + o->extent;

  // completely outside, since query is outside the relevant area
  if (x > max_dist || y > max_dist || z > max_dist) return false;

  // surface check
  int num_less_extant = (x < o->extent) + (y < o->extent) + (z < o->extent);

  if (num_less_extant > 1) return true;

  x = std::max(x - o->extent, 0.f);
  y = std::max(y - o->extent, 0.f);
  z = std::max(z - o->extent, 0.f);

  return Distance::norm(x, y, z) < sqRaius;
}
template <typename PointT, typename ContainerT>
template <typename Distance>
bool Octree<PointT, ContainerT>::contains(const PointT& query, float sqRaius, const Octree::Octant* octant)
{
  float x = get<0>(query) - octant->x;
  float y = get<1>(query) - octant->y;
  float z = get<2>(query) - octant->z;

  x = std::abs(x);
  y = std::abs(y);
  z = std::abs(z);

  x += octant->extent;
  y += octant->extent;
  z += octant->extent;

  return Distance::norm(x, y, z) < sqRaius;
}
template <typename PointT, typename ContainerT>
template <typename Distance>
bool Octree<PointT, ContainerT>::inside(const PointT& query, float radius, const Octree::Octant* octant)
{
  // if the query ball is inside the octant

  float x = get<0>(query) - octant->x;
  float y = get<0>(query) - octant->y;
  float z = get<0>(query) - octant->z;

  x = std::abs(x) + radius;
  y = std::abs(y) + radius;
  z = std::abs(z) + radius;

  if (x > octant->extent) return false;
  if (y > octant->extent) return false;
  if (z > octant->extent) return false;

  return true;
}
template <typename PointT, typename ContainerT>
template <typename Distance>
bool Octree<PointT, ContainerT>::knnNeighbors(const Octree::Octant* octant, const PointT& query,
                                              KNNResult<float>& result) const
{

  const ContainerT& points = *data_;

  if (octant->isLeaf)
  {
    size_t idx = octant->start;

    for (size_t i = 0; i < octant->size; ++i)
    {
      const PointT& p = points[idx];

      float dist = Distance::compute(query, p);

      result.addPoint(dist, idx);

      idx = successors_[idx];  //　迭代到下一个点
    }

    float radius = Distance::sqrt(result.worstDist());

    return inside<Distance>(query, radius, octant);
  }

  int mortonCode = 0;
  if (get<0>(query) > octant->x) mortonCode |= 1;
  if (get<1>(query) > octant->y) mortonCode |= 2;
  if (get<2>(query) > octant->z) mortonCode |= 4;

  if (octant->child[mortonCode] != nullptr)
  {
    if (knnNeighbors<Distance>(octant->child[mortonCode], query, result)) return true;
  }

  for (int i = 0; i < 8; ++i)
  {
    if (i == mortonCode) continue;
    if (octant->child[i] == nullptr) continue;

    float sqrRadius = result.worstDist();
    float radius = Distance::sqrt(sqrRadius);
    if (!overlaps<Distance>(query, radius, sqrRadius, octant->child[i])) continue;
    if (knnNeighbors<Distance>(octant->child[i], query, result)) return true;
  }

  float sqrRadius = result.worstDist();
  float radius = Distance::sqrt(sqrRadius);

  return inside<Distance>(query, radius, octant);
}

}  // namespace nanoflann
#endif  // OCTREE_H
