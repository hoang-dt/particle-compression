#ifndef HEAP_H
#define HEAP_H

#include <algorithm>
#include <functional>
#include <map>
#include <vector>

template <
  typename T,                            // data type
  typename P,                            // priority type
  class    K = std::less<T>,
  class    C = std::less<P>,             // comparator for priorities
  class    M = std::map<T, unsigned, K>  // maps type T to unsigned integer
>
class DynamicHeap {
public:
  DynamicHeap(size_t count = 0);
  ~DynamicHeap() {}
  void insert(T data, P priority);
  void update(T data, P priority);
  bool top(T& data);
  bool top(T& data, P& priority);
  bool pop();
  bool extract(T& data);
  bool extract(T& data, P& priority);
  bool erase(T data);
  bool find(T data) const;
  bool find(T data, P& priority) const;
  bool empty() const { return heap.empty(); }
  size_t size() const { return heap.size(); }
public:
  struct HeapEntry {
    HeapEntry(P p, T d) : priority(p), data(d) {}
    P priority;
    T data;
  };
  std::vector<HeapEntry> heap;
  M index;
  C lower;
  void ascend(unsigned i);
  void descend(unsigned i);
  void swap(unsigned i, unsigned j);
  bool ordered(unsigned i, unsigned j) const
  {
    return !lower(heap[i].priority, heap[j].priority);
  }
  unsigned parent(unsigned i) const { return (i - 1) / 2; }
  unsigned left(unsigned i) const { return 2 * i + 1; }
  unsigned right(unsigned i) const { return 2 * i + 2; }
};

template < typename T, typename P, class K, class C, class M >
DynamicHeap<T, P, K, C, M>::DynamicHeap(size_t count)
{
  heap.reserve(count);
}

template < typename T, typename P, class K, class C, class M >
void
DynamicHeap<T, P, K, C, M>::insert(T data, P priority)
{
  if (index.find(data) != index.end())
    update(data, priority);
  else {
    unsigned i = (unsigned int)heap.size();
    heap.push_back(HeapEntry(priority, data));
    ascend(i);
  }
}

template < typename T, typename P, class K, class C, class M >
void
DynamicHeap<T, P, K, C, M>::update(T data, P priority)
{
  unsigned i = index[data];
  heap[i].priority = priority;
  ascend(i);
  descend(i);
}

template < typename T, typename P, class K, class C, class M >
bool
DynamicHeap<T, P, K, C, M>::top(T& data)
{
  if (!heap.empty()) {
    data = heap[0].data;
    return true;
  }
  else
    return false;
}

template < typename T, typename P, class K, class C, class M >
bool
DynamicHeap<T, P, K, C, M>::top(T& data, P& priority)
{
  if (!heap.empty()) {
    data = heap[0].data;
    priority = heap[0].priority;
    return true;
  }
  else
    return false;
}

template < typename T, typename P, class K, class C, class M >
bool
DynamicHeap<T, P, K, C, M>::pop()
{
  if (!heap.empty()) {
    T data = heap[0].data;
    swap(0, (unsigned int)heap.size() - 1);
    index.erase(data);
    heap.pop_back();
    if (!heap.empty())
      descend(0);
    return true;
  }
  else
    return false;
}

template < typename T, typename P, class K, class C, class M >
bool
DynamicHeap<T, P, K, C, M>::extract(T& data)
{
  if (!heap.empty()) {
    data = heap[0].data;
    return pop();
  }
  else
    return false;
}

template < typename T, typename P, class K, class C, class M >
bool
DynamicHeap<T, P, K, C, M>::extract(T& data, P& priority)
{
  if (!heap.empty()) {
    data = heap[0].data;
    priority = heap[0].priority;
    return pop();
  }
  else
    return false;
}

template < typename T, typename P, class K, class C, class M >
bool
DynamicHeap<T, P, K, C, M>::erase(T data)
{
  if (index.find(data) == index.end())
    return false;
  unsigned i = index[data];
  swap(i, (unsigned int)heap.size() - 1);
  index.erase(data);
  heap.pop_back();
  if (i < heap.size()) {
    ascend(i);
    descend(i);
  }
  return true;
}

template < typename T, typename P, class K, class C, class M >
bool
DynamicHeap<T, P, K, C, M>::find(T data) const
{
  return index.find(data) != index.end();
}

template < typename T, typename P, class K, class C, class M >
bool
DynamicHeap<T, P, K, C, M>::find(T data, P& priority) const
{
  typename M::const_iterator p;
  if ((p = index.find(data)) == index.end())
    return false;
  unsigned i = p->second;
  priority = heap[i].priority;
  return true;
}

template < typename T, typename P, class K, class C, class M >
void
DynamicHeap<T, P, K, C, M>::ascend(unsigned i)
{
  for (unsigned j; i && !ordered(j = parent(i), i); i = j)
    swap(i, j);
  index[heap[i].data] = i;
}

template < typename T, typename P, class K, class C, class M >
void
DynamicHeap<T, P, K, C, M>::descend(unsigned i)
{
  for (unsigned j, k;
       (j = ((k =  left(i)) < heap.size() && !ordered(i, k) ? k : i),
        j = ((k = right(i)) < heap.size() && !ordered(j, k) ? k : j)) != i;
       i = j)
    swap(i, j);
  index[heap[i].data] = i;
}

template < typename T, typename P, class K, class C, class M >
void
DynamicHeap<T, P, K, C, M>::swap(unsigned i, unsigned j)
{
  std::swap(heap[i], heap[j]);
  index[heap[i].data] = i;
}

#endif

