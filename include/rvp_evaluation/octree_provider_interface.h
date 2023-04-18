#pragma once

#include <octomap_vpp/RoiOcTree.h>
#include <boost/thread/mutex.hpp>

namespace rvp_evaluation
{

class MutexBase
{
public:
  virtual void lock() = 0;
  virtual bool try_lock() = 0;
  virtual void unlock() = 0;
};

class MutexDummy : public MutexBase
{
public:
  MutexDummy() {}

  void lock() {}
  bool try_lock() { return true; }
  void unlock() {}
};

template <typename T>
class MutexPtr : public MutexBase
{
protected:
  std::shared_ptr<T> mtx_;

public:
  MutexPtr() : mtx_(new T) {}
  MutexPtr(std::shared_ptr<T> mtx) : mtx_(mtx) {}

  void lock() { mtx_->lock(); }
  bool try_lock() { return mtx_->try_lock(); }
  void unlock() { mtx_->unlock(); }
};

template <typename T>
class MutexRef : public MutexBase
{
private:
  T own_mtx_;

protected:
  T &mtx_;

public:
  MutexRef() : mtx_(own_mtx_) {}
  MutexRef(T &mtx) : mtx_(mtx) {}

  void lock() { mtx_.lock(); }
  bool try_lock() { return mtx_.try_lock(); }
  void unlock() { mtx_.unlock(); }
};

class OctreeProviderInterface
{
public:
  virtual std::shared_ptr<octomap_vpp::RoiOcTree> getPlanningTree() = 0;
  virtual double getTreeResolution() = 0;
  virtual MutexBase& getTreeMutex() = 0;
};

} // namespace rvp_evaluation
