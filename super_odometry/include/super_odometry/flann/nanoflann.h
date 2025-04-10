//
// Created by ubuntu on 2020/7/9.
//

#ifndef NANOFLANN_HPP
#define NANOFLANN_HPP

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>   // for abs
#include <cstdio>  // for fwrite
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace nanoflann
{

template <typename T, typename = int>
struct has_resize : std::false_type
{
};

template <typename T>
struct has_resize<T, decltype(std::declval<T>().resize(1), 0)> : std::true_type
{
};

template <typename T, typename = int>
struct has_assign : std::false_type
{
};

template <typename T>
struct has_assign<T, decltype(std::declval<T>().assign(1, 0), 0)> : std::true_type
{
};

template <typename Container>
inline typename std::enable_if<has_resize<Container>::value, void>::type resize(Container &c, const size_t nElements)
{
  c.resize(nElements);
}

template <typename Container>
inline typename std::enable_if<!has_resize<Container>::value, void>::type resize(Container &c, const size_t nElements)
{
  if (nElements != c.size()) throw(std::logic_error("Try to change the size of a std::array"));
}

template <typename Container, typename T>
inline typename std::enable_if<has_assign<Container>::value, void>::type assign(Container &c, const size_t nElements,
                                                                                const T &value)
{
  c.assgin(nElements, value);
}

template <typename Container, typename T>
inline typename std::enable_if<!has_assign<Container>::value, void>::type assign(Container &c, const size_t nElements,
                                                                                 const T &value)
{
  for (size_t i = 0; i < nElements; i++) c[i] = value;
}

/// \brief KNN的结果
/// \tparam _DistanceType eg: int, float, double
/// \tparam _IndexType    eg: 索引的类型
/// \tparam _CountType 　　eg:
template <typename _DistanceType, typename _IndexType = size_t, typename _CountType = size_t>
class KNNResult
{
public:
  typedef _DistanceType DistanceType;
  typedef _IndexType IndexType;
  typedef _CountType CountType;

private:
  IndexType *indices;
  DistanceType *dists;
  CountType capacity;
  CountType count;

public:
  inline explicit KNNResult(CountType capacity) : indices(nullptr), dists(nullptr), capacity(capacity), count(0)
  {
  }

  /// \brief 初始化 KNNResult容器
  /// \param indices_: eg: vector<IndexType>的首地址
  /// \param dists_  : vector<DistanceType> 的首地址
  inline void init(IndexType *indices_, DistanceType *dists_)
  {
    indices = indices_;
    dists = dists_;
    count = 0;
    if (capacity) dists[capacity - 1] = std::numeric_limits<DistanceType>::max();
  }

  inline CountType size() const
  {
    return count;
  }

  // 判断k个最近点是否找到了
  inline bool full() const
  {
    return count == capacity;
  }

  /// \brief 添加每一个点
  /// \param dist  [in]查询点到邻近点的距离
  /// \param index [in] 临近点的id
  /// \return [out] 一直是true
  inline bool addPoint(DistanceType dist, IndexType index)
  {
    //    std::cout << "idx: " << index << " dist: " << dist << std::endl;
    CountType i;
    // 移动, count 总是指向最新元素的位置
    for (i = count; i > 0; --i)
    {
      if (dists[i - 1] > dist)
      {
        if (i < capacity)
        {
          dists[i] = dists[i - 1];
          indices[i] = indices[i - 1];
        }
      }
      else
        break;
    }

    //　将点插入到合适的位置
    if (i < capacity)
    {
      dists[i] = dist;
      indices[i] = index;
    }

    // 如果最近邻点少于capacity, count++
    if (count < capacity) count++;

    return true;
  }  // function addPoint

  /// \brief 返回最坏距离
  /// \return
  inline DistanceType worstDist() const
  {
    return dists[capacity - 1];
  }
};  // class KNNResult

struct IndexDis_Sorter
{
  /// \brief 将最近邻点按照邻近距离进行排序
  /// \tparam PairType
  /// \param p1
  /// \param p2
  /// \return
  template <typename PairType>
  inline bool operator()(const PairType &p1, const PairType &p2)
  {
    return p1.second < p2.second;
  }
};

template <typename _DistanceTpye, typename _IndexType = size_t>
class RadiusResultSet
{
public:
  typedef _DistanceTpye DistanceType;
  typedef _IndexType IndexType;

public:
  const DistanceType radius;

  std::vector<std::pair<IndexType, DistanceType> > &m_indices_dists;

  inline RadiusResultSet(DistanceType radius_, std::vector<std::pair<IndexType, DistanceType> > &indices_dists)
    : radius(radius_), m_indices_dists(indices_dists)
  {
    init();
  }

public:
  inline void init()
  {
    clear();
  }
  inline void clear()
  {
    m_indices_dists.clear();
  }

  inline size_t size() const
  {
    return m_indices_dists.size();
  }

  inline bool full() const
  {
    return true;
  }

  inline bool addPoint(DistanceType dist, IndexType index)
  {
    if (dist < radius) m_indices_dists.push_back(std::make_pair(index, dist));

    return true;
  }

  inline DistanceType worstDist() const
  {
    return radius;
  }

  std::pair<IndexType, DistanceType> worst_item() const
  {
    if (m_indices_dists.empty())
      throw std::runtime_error(
        "Cannot invoke RadiusResultSet::worst_item() on "
        "an empty list of result");

    auto it = std::max_element(m_indices_dists.begin(), m_indices_dists.end(), IndexDis_Sorter());

    return *it;
  }
};  // class RediusResuktSet

/// KD-Tree 构建的需要的参数
struct KDTreeSingleIndexAdaptorParams
{

  /// \brief
  /// \param _leaf_max_size , 叶节点的大小
  KDTreeSingleIndexAdaptorParams(size_t _leaf_max_size = 10) : leaf_max_size(_leaf_max_size)
  {
  }
  size_t leaf_max_size;
};

/// KD-Tree 搜索过程需要的参数
struct SearchParams
{

  SearchParams(int checks_IGNORED_ = 32, float eps_ = 0, bool sorted_ = true)
    : checks(checks_IGNORED_), eps(eps_), sorted(sorted_)
  {
  }
  int checks;
  float eps;
  bool sorted;
};

/// 内存管理
template <typename T>
inline T *allocate(size_t count = 1)
{
  T *mem = static_cast<T *>(::malloc(sizeof(T) * count));
  return mem;
}

const size_t WORDSIZE = 16;

const size_t BLOCKSIZE = 8192;

class PooledAllocator
{
  size_t remaining;  /// Number of bytes left in current block of storage
  void *base;        /// Pointer to base of currrent block of storage
  void *loc;         /// Current location in block to next a

  void initial_init()
  {
    remaining = 0;
    base = nullptr;
    usedMemory = 0;
    wastedMemory = 0;
  }

public:
  size_t usedMemory;
  size_t wastedMemory;

  PooledAllocator()
  {
    initial_init();
  }

  ~PooledAllocator()
  {
    free_all();
  }

  void free_all()
  {
    while (base != nullptr)
    {
      void *prev = *(static_cast<void **>(base));  // get pointer to prev block
      ::free(base);
      base = prev;
    }
    initial_init();
  }

  void *malloc(const size_t req_size)
  {
    const size_t size = (req_size + WORDSIZE - 1) & ~(WORDSIZE - 1);

    if (size > remaining)
    {
      wastedMemory += remaining;

      const size_t blocksize =
        (size + sizeof(void *) + WORDSIZE - 1 > BLOCKSIZE) ? size + sizeof(void *) + (WORDSIZE - 1) : BLOCKSIZE;

      void *m = ::malloc(blocksize);
      if (!m)
      {
        std::cerr << "Failed to allocate memory. \n" << std::endl;

        return nullptr;
      }

      static_cast<void **>(m)[0] = base;
      base = m;

      size_t shift = 0;

      remaining = blocksize - sizeof(void *) - shift;
      loc = (static_cast<char *>(m) + sizeof(void *) + shift);
    }

    void *rloc = loc;
    loc = static_cast<char *>(loc) + size;
    remaining -= size;

    usedMemory += size;

    return rloc;
  }

  template <typename T>
  T *allocate(const size_t count = 1)
  {
    T *mem = static_cast<T *>(this->malloc(sizeof(T) * count));

    return mem;
  }
};

template <int DIM, typename T>
struct array_or_vector_selector
{
  using container_t = std::array<T, DIM>;
};

template <typename T>
struct array_or_vector_selector<-1, T>
{
  using container_t = std::vector<T>;
};

/// \brief Kd-tree 的基类
/// \tparam Derived        : Kd-tree 的子类
/// \tparam Distance       : L1 norm 或者 L2 norm
/// \tparam DatasetAdaptor : 数据集
/// \tparam DIM            :
/// \tparam IndexType      :
template <class Derived, typename Distance, class DatasetAdaptor, int DIM = -1, typename IndexType = size_t>
class KDTreeBaseClass
{
public:
  typedef typename Distance::ElementType ElementType;
  typedef typename Distance::DistanceType DistanceType;

  // 声明节点类型
  struct Node
  {
    union {
      // 叶节点
      struct leaf
      {
        IndexType left, right;  // Indices of points in leaf node
      } lr;
      struct nonleaf
      {
        int div_axis;                  // Dimension used for subdivision
        DistanceType divlow, divhigh;  // The values used for subdivision
      } sub;
    } node_type;

    Node *child1 = nullptr;
    Node *child2 = nullptr;  // Child nodes (both = nullptr mean its a leaf node)
  };

  typedef Node *NodePtr;

  struct Interval
  {
    ElementType low, high;
  };

  // vector indices
  std::vector<IndexType> vind;
  NodePtr root_node;

  size_t m_leaf_max_size;
  size_t m_size;                 // Number of current points in the dataset
  size_t m_size_at_index_build;  // Number of points in the dataset when the
  // index was built
  int dim;  // Dimensionality of each data point

  typedef typename array_or_vector_selector<DIM, Interval>::container_t BoundingBox;

  typedef typename array_or_vector_selector<DIM, DistanceType>::container_t distance_vector_t;

  /// The KD-tree used to find neighbors
  BoundingBox root_bbox;

  PooledAllocator pool;

  void freeIndex(Derived &obj)
  {
    obj.pool.free_all();
    obj.root_node = nullptr;
    obj.m_size_at_index_build = 0;
  }

  size_t size(const Derived &obj) const
  {
    return obj.m_size;
  }

  size_t veclen(const Derived &obj)
  {
    return static_cast<size_t>(DIM > 0 ? DIM : obj.dim);
  }

  /// \brief
  /// \param obj       子类
  /// \param idx       查询点的id
  /// \param component 查询点维度的id
  /// \return
  inline ElementType dataset_get(const Derived &obj, size_t idx, int component) const
  {
    return obj.dataset.kdtree_get_pt(idx, component);
  }

  size_t usedMemory(Derived &obj)
  {
    return obj.pool.usedMemory + obj.pool.wastedMemory + obj.dataset.kdtree_get_point_count() * sizeof(IndexType);
  }

  /// \brief 计算一个范围内点云的axis
  /// \param obj        [in]  子类
  /// \param ind        [in]  点的index的起始地址
  /// \param count      [in]  点的大小
  /// \param axis       [in]  沿着哪个轴进行统计最大值和最小值
  /// \param min_elem   [out] 统计最小值
  /// \param max_elem   [out] 统计最大值
  void computeMinMax(const Derived &obj, IndexType *ind, IndexType count, int axis, ElementType &min_elem,
                     ElementType &max_elem)
  {
    min_elem = dataset_get(obj, ind[0], axis);
    max_elem = dataset_get(obj, ind[0], axis);
    for (IndexType i = 1; i < count; ++i)
    {
      ElementType val = dataset_get(obj, ind[i], axis);
      if (val < min_elem) min_elem = val;
      if (val > max_elem) max_elem = val;
    }
  }

  /// \brief Create a tree node that subdivides the list of vecs from
  /// vind[first] to vind[last]. The rounine is called recursively on each
  /// sublist. \param obj \param left  index of the first vector \param right
  /// index of the second vector \param bbox \return
  NodePtr divideTree(Derived &obj, const IndexType left, const IndexType right, BoundingBox &bbox)
  {
    NodePtr node = obj.pool.template allocate<Node>();  // allocate memory

    if ((right - left) < static_cast<IndexType>(obj.m_leaf_max_size))
    {

      node->child1 = node->child2 = nullptr;  // mask as leaf node
      node->node_type.lr.left = left;
      node->node_type.lr.right = right;

      for (int i = 0; i < (DIM > 0 ? DIM : obj.dim); ++i)
      {
        bbox[i].low = bbox[i].high = dataset_get(obj, obj.vind[left], i);
      }

      for (IndexType k = left; k < right; ++k)
      {
        for (int i = 0; i < (DIM > 0 ? DIM : obj.dim); ++i)
        {
          if (bbox[i].low > dataset_get(obj, obj.vind[k], i)) bbox[i].low = dataset_get(obj, obj.vind[k], i);

          if (bbox[i].high < dataset_get(obj, obj.vind[k], i)) bbox[i].high = dataset_get(obj, obj.vind[k], i);
        }
      }
    }
    else
    {

      IndexType idx;
      int cut_axis;
      DistanceType cutval;
      middleSplit_(obj, &(obj.vind[0]) + left, right - left, idx, cut_axis, cutval, bbox);

      node->node_type.sub.div_axis = cut_axis;

      BoundingBox left_bbox(bbox);
      left_bbox[cut_axis].high = cutval;
      node->child1 = divideTree(obj, left, left + idx, left_bbox);

      BoundingBox right_bbox(bbox);
      right_bbox[cut_axis].low = cutval;
      node->child2 = divideTree(obj, left + idx, right, right_bbox);

      node->node_type.sub.divlow = left_bbox[cut_axis].high;
      node->node_type.sub.divhigh = right_bbox[cut_axis].low;

      // TODO: 这个好像没啥用吧
      for (int i = 0; i < (DIM > 0 ? DIM : obj.dim); ++i)
      {
        bbox[i].low = std::min(left_bbox[i].low, right_bbox[i].low);
        bbox[i].high = std::max(left_bbox[i].high, right_bbox[i].high);
      }
    }

    return node;
  }  // function divideTree

  /// \brief
  /// \param obj        [in]  obj      : 子类
  /// \param ind        [in]  ind      : 需要分割点的indices的起始地址
  /// \param count      [in]  count    : 需要分割的点数
  /// \param index      [out] index    : [左右分割空间的分界线]
  /// \param cut_axis   [out] cut_axis : 沿着哪个轴进行切分数据
  /// \param cut_val    [out] cut_val  :　cut_axis 的值
  /// \param bbox       bbox     :
  void middleSplit_(Derived &obj, IndexType *ind, IndexType count, IndexType &index, int &cut_axis,
                    DistanceType &cut_val, const BoundingBox &bbox)
  {

    const DistanceType EPS = static_cast<DistanceType>(0.00001);
    ElementType max_span = bbox[0].high - bbox[0].low;

    // step1: 计算的点云分布,统计点云的分布范围
    for (int i = 0; i < (DIM > 0 ? DIM : obj.dim); ++i)
    {
      ElementType span = bbox[i].high - bbox[i].low;
      if (span > max_span)
      {
        max_span = span;
      }
    }

    ElementType max_spread = -1;
    cut_axis = 0;
    for (int i = 0; i < (DIM > 0 ? DIM : obj.dim); ++i)
    {
      ElementType span = bbox[i].high - bbox[i].low;
      if (span > (1 - EPS) * max_span)
      {
        ElementType min_elem, max_elem;
        computeMinMax(obj, ind, count, i, min_elem, max_elem);
        ElementType spread = max_elem - min_elem;
        if (spread > max_spread)
        {
          cut_axis = i;
          max_spread = spread;
        }
      }
    }

    DistanceType split_val = (bbox[cut_axis].low + bbox[cut_axis].high) / 2;

    ElementType min_elem, max_elem;
    computeMinMax(obj, ind, count, cut_axis, min_elem, max_elem);

    if (split_val < min_elem)
      cut_val = min_elem;
    else if (split_val > max_elem)
      cut_val = max_elem;
    else
      cut_val = split_val;

    IndexType lim1, lim2;

    planeSplit(obj, ind, count, cut_axis, cut_val, lim1, lim2);

    // TODO: 尽可能的均分数据点
    if (lim1 > count / 2)
      index = lim1;
    else if (lim2 < count / 2)
      index = lim2;
    else
      index = count / 2;
  }  // function middleSplit_

  void planeSplit(Derived &obj, IndexType *ind, const IndexType count, int cut_axis, DistanceType &cut_val,
                  IndexType &lim1, IndexType &lim2)
  {
    IndexType left = 0;
    IndexType right = count - 1;

    for (;;)
    {
      while (left <= right && dataset_get(obj, ind[left], cut_axis) < cut_val) ++left;
      while (right && left <= right && dataset_get(obj, ind[right], cut_axis) >= cut_val) --right;

      if (left > right || !right) break;
      std::swap(ind[left], ind[right]);
      ++left;
      --right;
    }

    lim1 = left;
    right = count - 1;

    for (;;)
    {
      while (left <= right && dataset_get(obj, ind[left], cut_axis) <= cut_val) ++left;
      while (right && left <= right && dataset_get(obj, ind[right], cut_axis) > cut_val) --right;

      if (left > right || !right) break;

      std::swap(ind[left], ind[right]);
      ++left;
      --right;
    }
    lim2 = left;
  }

  /// \brief
  /// \param obj    : [in] 子类
  /// \param vec    : [in] 询问点
  /// \param dists  : [out]询问点到最近boundingbox个的距离
  /// \return       :
  DistanceType computeInitialDistances(const Derived &obj, const ElementType *vec, distance_vector_t &dists) const
  {
    DistanceType distsq = DistanceType();
    for (int i = 0; i < (DIM > 0 ? DIM : obj.dim); ++i)
    {

      if (vec[i] < obj.root_bbox[i].low)
      {
        dists[i] = obj.distance.accum_dist(vec[i], obj.root_bbox[i].low, i);
        distsq += dists[i];
      }
      if (vec[i] > obj.root_bbox[i].high)
      {
        dists[i] = obj.distance.accum_dist(vec[i], obj.root_bbox[i].high, i);
        distsq += dists[i];
      }
    }
    return distsq;
  }
};

/// \brief distance metric
/// \tparam T eg float double, uint8, int8 int32, int64, uint64
/// \tparam Datasource
/// \tparam _DistanceType
template <typename T, class Datasource, typename _DistanceType = T>
struct L2_Adaptor
{
  typedef T ElementType;
  typedef _DistanceType DistanceType;

  const Datasource &datasource;

public:
  L2_Adaptor(const Datasource &datasource) : datasource(datasource)
  {
  }

  ///
  /// \param a :query point
  /// \param b_idx : neighborest dataset point
  /// \param size  : 向量的维度
  /// \param worst_dist :最坏距离
  /// \return 当最近点与询问点之间的距离大于worst_dist
  inline DistanceType evalMetric(const T *a, const size_t b_idx, size_t size, DistanceType worst_dist = -1) const
  {

    DistanceType result = DistanceType();

    const T *last = a + size;
    const T *lastgroup = last - 3;
    size_t d = 0;

    // b_idx,
    while (a < lastgroup)
    {
      const DistanceType diff0 = a[0] - datasource.kdtree_get_point_count(b_idx, d++);
      const DistanceType diff1 = a[1] - datasource.kdtree_get_point_count(b_idx, d++);
      const DistanceType diff2 = a[2] - datasource.kdtree_get_point_count(b_idx, d++);
      const DistanceType diff3 = a[3] - datasource.kdtree_get_point_count(b_idx, d++);

      result += diff0 * diff0 + diff1 * diff1 + diff2 * diff2 + diff3 * diff3;
      a += 4;

      if ((worst_dist > 0) && (result > worst_dist))
      {
        return result;
      }
    }

    // Process last 0-3 componets. Not need for stan
    while (a < last)
    {
      const DistanceType diff0 = *a++ - datasource.kdtree_get_pt(b_idx, d++);
      result += diff0 * diff0;
    }

    return result;
  }

  // 计算两个点之间的距离
  template <typename U, typename V>
  inline DistanceType accum_dist(const U a, const V b, const size_t) const
  {
    return (a - b) * (a - b);
  }
};  // struct L2_Adaptor

template <typename T, typename DataSource, typename _DistanceType = T>
struct L2_Simple_Adaptor
{
  typedef T ElementType;
  typedef _DistanceType DistanceType;

  const DataSource &data_source;

  L2_Simple_Adaptor(const DataSource &_data_source) : data_source(_data_source)
  {
  }

  inline DistanceType evalMetric(const T *a, const size_t b_idx, size_t size) const
  {
    DistanceType result = DistanceType();
    for (size_t i = 0; i < size; i++)
    {

      const DistanceType diff = a[i] - data_source.kdtree_get_pt(b_idx, i);
      result += diff * diff;
    }

    return result;
  }

  template <typename U, typename V>
  inline DistanceType accum_dist(const U a, const V b, const size_t) const
  {
    return (a - b) * (a - b);
  }
};

/// \brief KD-tree 的封装
/// \tparam Distance : L1, L2 norm
/// \tparam DatasetAdapter : 数据集
/// \tparam DIM : 向量的维度
/// \tparam IndexType : 索引
template <typename Distance, class DatasetAdapter, int DIM = -1, typename IndexType = size_t>
class KDTreeSingIndexAaptor : public KDTreeBaseClass<KDTreeSingIndexAaptor<Distance, DatasetAdapter, DIM, IndexType>,
                                                     Distance, DatasetAdapter, DIM, IndexType>
{
public:
  typedef
  typename nanoflann::KDTreeBaseClass<nanoflann::KDTreeSingIndexAaptor<Distance, DatasetAdapter, DIM, IndexType>,
                                      Distance, DatasetAdapter, DIM, IndexType>
    BaseClassRef;

  typedef typename BaseClassRef::ElementType ElementType;
  typedef typename BaseClassRef::DistanceType DistanceType;

  typedef typename BaseClassRef::Node Node;
  typedef Node *NodePtr;

  typedef typename BaseClassRef::Interval Interval;
  typedef typename BaseClassRef::BoundingBox BoundingBox;
  typedef typename BaseClassRef::distance_vector_t distance_vector_t;

public:
  KDTreeSingIndexAaptor(const int dimensionality, const DatasetAdapter &inputData,
                        const KDTreeSingleIndexAdaptorParams &params = KDTreeSingleIndexAdaptorParams())
    : dataset(inputData),    // 数据集
      index_params(params),  // 输入的配置参数
      distance(inputData)    // 距离度量函数
  {

    // 初始化根节点
    BaseClassRef::root_node = nullptr;
    // 初始化数据集的大小
    BaseClassRef::m_size = dataset.kdtree_get_point_count();
    BaseClassRef::m_size_at_index_build = BaseClassRef::m_size;
    // 初始化数据集的维度
    BaseClassRef::dim = dimensionality;
    if (DIM > 0) BaseClassRef::dim = DIM;
    // 初始化叶节点的大小
    BaseClassRef::m_leaf_max_size = params.leaf_max_size;

    std::cout << "BaseClassRef::m_size " << BaseClassRef::m_size << "\nBaseClassRef::m_size_at_index_build "
              << BaseClassRef::m_size_at_index_build << "\nBaseClassRef::dim " << BaseClassRef::dim
              << "\nBaseClassRef::m_leaf_max_size " << BaseClassRef::m_leaf_max_size << std::endl;

    // Create a permutable array of indices to the input vectors
    init_vind();
  }

  KDTreeSingIndexAaptor(const KDTreeSingIndexAaptor<Distance, DatasetAdapter, DIM, IndexType> &) = delete;

  // 构建kdTree
  void buildIndex()
  {
    BaseClassRef::m_size = dataset.kdtree_get_point_count();
    BaseClassRef::m_size_at_index_build = BaseClassRef::m_size;
    init_vind();
    this->freeIndex(*this);
    if (BaseClassRef::m_size == 0) return;

    computeBoundingBox(BaseClassRef::root_bbox);

    BaseClassRef::root_node = this->divideTree(*this, 0, BaseClassRef::m_size, BaseClassRef::root_bbox);
  }
  //　查询点
  template <typename RESULTSET>
  bool findNeighbors(RESULTSET &result, const ElementType *vec, const SearchParams &searchParams) const
  {

    if (this->size(*this) == 0) return false;
    if (!BaseClassRef::root_node)
      throw std::runtime_error("[nanoflann] findNeighbors() called before building the index");
    float epsError = 1 + searchParams.eps;

    //声明到boundingbox的距离
    distance_vector_t dists;

    auto zero = static_cast<decltype(result.worstDist())>(0);
    assign(dists, (DIM > 0 ? DIM : BaseClassRef::dim), zero);

    //计算当前节点boundingbox的距离
    DistanceType distsq = this->computeInitialDistances(*this, vec, dists);

    searchLevel(result, vec, BaseClassRef::root_node, distsq, dists, epsError);

    return result.full();
  }

  /// \brief knn search
  /// \param [in] query_point
  /// \param [in] num_closest
  /// \param [out]   out_indices
  /// \param [out]   out_distance_sq
  /// \return
  size_t knnSearch(const ElementType *query_point, const size_t num_closest, IndexType *out_indices,
                   DistanceType *out_distance_sq, const int /*nChecks_IGNOREG*/ = 10) const
  {
    nanoflann::KNNResult<DistanceType, IndexType> result(num_closest);
    result.init(out_indices, out_distance_sq);
    this->findNeighbors(result, query_point, nanoflann::SearchParams());
  }

  size_t radiusSearch(const ElementType *query_point, const DistanceType &radius,
                      std::vector<std::pair<IndexType, DistanceType> > &IndiceDists,
                      const SearchParams &searchParams) const
  {
    RadiusResultSet<DistanceType, IndexType> resultSet(radius, IndiceDists);

    this->findNeighbors(resultSet, query_point, searchParams);
    const size_t nFound = resultSet.size();

    if (searchParams.sorted) std::sort(IndiceDists.begin(), IndiceDists.end(), IndexDis_Sorter());

    return nFound;
  }

  /// \brief 在每个叶节点搜索最近邻点
  /// \tparam RESULTSET
  /// \param result     : [out]最近邻点的集合
  /// \param vec        : [in] 查询点
  /// \param node       : [in] 在哪个节点进行查询
  /// \param mindistsq  : [out]查询点到boundingbox的距离
  /// \param dists      : [out]查询点到到boundingbox的距离
  /// \param epsError   :
  /// \return
  template <class RESULTSET>
  bool searchLevel(RESULTSET &result, const ElementType *vec, const NodePtr node, DistanceType mindistsq,
                   distance_vector_t &dists, const float epsError) const
  {

    // condition1: 如果遍历到了叶节点
    if (node->child1 == nullptr && node->child2 == nullptr)
    {
      DistanceType worst_dist = result.worstDist();

      for (IndexType i = node->node_type.lr.left; i < node->node_type.lr.right; ++i)
      {
        const IndexType index = BaseClassRef::vind[i];
        //　计算vec和dataset[index]
        DistanceType dist = distance.evalMetric(vec, index, (DIM > 0 ? DIM : BaseClassRef::dim));

        if (dist < worst_dist)
        {
          // TODO: 这个不会发生
          if (!result.addPoint(dist, BaseClassRef::vind[i]))
          {
            return false;
          }
        }
      }
      return true;
    }
    // conditional2: 遍历非叶节点

    // 获得沿着哪个轴切分空间
    int idx = node->node_type.sub.div_axis;
    // 获得 val
    ElementType val = vec[idx];

    DistanceType diff1 = val - node->node_type.sub.divlow;
    DistanceType diff2 = val - node->node_type.sub.divhigh;

    NodePtr bestChild;
    NodePtr otherChild;
    DistanceType cut_dist;

    // 判断查询点在右子树，还是在左子树
    if (diff1 + diff2 < 0)
    {
      bestChild = node->child1;
      otherChild = node->child2;
      cut_dist = distance.accum_dist(val, node->node_type.sub.divhigh, idx);
    }
    else
    {

      bestChild = node->child2;
      otherChild = node->child1;
      cut_dist = distance.accum_dist(val, node->node_type.sub.divlow, idx);
    }

    // Call recursively to search next level down
    if (!searchLevel(result, vec, bestChild, mindistsq, dists, epsError))
    {
      return false;
    }

    DistanceType dst = dists[idx];
    mindistsq = mindistsq + cut_dist - dst;  // 更新查询点到边界框的距离
    dists[idx] = cut_dist;

    // 如果3d点到边界框的
    if (mindistsq * epsError <= result.worstDist())
    {
      if (!searchLevel(result, vec, otherChild, mindistsq, dists, epsError))
      {
        return false;
      }
    }

    dists[idx] = dst;
    return true;
  }

public:
  /// \brief 初始化indices
  void init_vind()
  {
    BaseClassRef::m_size = dataset.kdtree_get_point_count();
    if (BaseClassRef::vind.size() != BaseClassRef::m_size) BaseClassRef::vind.resize(BaseClassRef::m_size);
    for (size_t i = 0; i < BaseClassRef::m_size; i++) BaseClassRef::vind[i] = i;
  }

  // 计算所有点的BoundingBox
  void computeBoundingBox(BoundingBox &bbox)
  {
    resize(bbox, (DIM > 0 ? DIM : BaseClassRef::dim));
    if (dataset.kdtree_get_bbox(bbox))
    {
    }
    else
    {
      const size_t N = dataset.kdtree_get_point_count();
      if (!N)
        throw std::runtime_error(
          "[nanoflann] computeBoundingBox() called but "
          "no data points found.");
      for (int i = 0; i < (DIM > 0 ? DIM : BaseClassRef::dim); ++i)
      {
        bbox[i].low = bbox[i].high = this->dataset_get(*this, 0, i);
      }

      for (size_t k = 1; k < N; ++k)
      {
        for (int i = 0; i < (DIM > 0 ? DIM : BaseClassRef::dim); ++i)
        {

          if (this->dataset_get(*this, k, i) < bbox[i].low) bbox[i].low = this->dataset_get(*this, k, i);

          if (this->dataset_get(*this, k, i) > bbox[i].high) bbox[i].high = this->dataset_get(*this, k, i);
        }
      }

      //      for (int i = 0; i < (DIM > 0 ? DIM : BaseClassRef::dim); ++i)
      //      {
      //        std::cout << "bbox[i].low: " << bbox[i].low << " bbox[i].high: " << bbox[i].high << std::endl;
      //      }
    }
  }  // function computeBoundingBox

public:
  const DatasetAdapter &dataset;  // The source of our data

  const KDTreeSingleIndexAdaptorParams index_params;

  Distance distance;
};

}  // namespace nanoflann

#endif  // NANOFLANN_HPP
