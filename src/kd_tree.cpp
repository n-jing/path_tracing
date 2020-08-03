#include "../inc/kd_tree.h"
#include <numeric>
#include <iostream>
#include "../inc/object.h"
#include "../inc/macros.inc"
#include "../inc/beam.h"
#include "../thirdparty/inc/hit_bounding_box.h"

using namespace Eigen;
using namespace std;

Node::Node()
{
  idx_ = -1;
  left_ = nullptr;
  right_ = nullptr;
  minB_[0] = numeric_limits<double>::max();
  minB_[1] = numeric_limits<double>::max();
  minB_[2] = numeric_limits<double>::max();

  maxB_[0] = -numeric_limits<double>::max();
  maxB_[1] = -numeric_limits<double>::max();
  maxB_[2] = -numeric_limits<double>::max();
}

Node::~Node()
{
  
}

size_t Node::GetId() const
{
  return idx_;
}

int Node::set_node(size_t idx)
{
  idx_ = idx;

  return 0;
}

int Node::set_child(Node *const l, Node *const r)
{
  left_ = l;
  right_ = r;

  return 0;
}

Node *Node::GetLeft() const
{
  return left_;
}

Node *Node::GetRight() const
{
  return right_;
}

const double *Node::GetMinB() const
{
  return &minB_[0];
}

const double *Node::GetMaxB() const
{
  return &maxB_[0];
}

int Node::set_bounding_box(const vector<Object*> &obj_ptr_vec, const vector<size_t> &idx)
{
  assert(idx.size());
  Vector3d obj_min;
  Vector3d obj_max;
  for (auto obj_idx : idx)
  {
    const Object &obj = *obj_ptr_vec[obj_idx];
    obj.GetBoundingBox(obj_min, obj_max);
    for (size_t itr = 0; itr < 3; ++itr)
    {
      if (obj_min[itr] < minB_[itr])
        minB_[itr] = obj_min[itr];
      if (obj_max[itr] > maxB_[itr])
        maxB_[itr] = obj_max[itr];
    }
  }

  return 0;
}

jing_m::KdTree::KdTree(const vector<Object*> &obj_ptr_vec)
{
  if (obj_ptr_vec.empty())
  {
    root_ = nullptr;
  } 
  else
  {
    vector<size_t> obj_idx(obj_ptr_vec.size());
    iota(obj_idx.begin(), obj_idx.end(), 0);

    vector<Vector3d> obj_center(obj_ptr_vec.size());
    vector<Vector3d> obj_length(obj_ptr_vec.size());
    for (size_t itr = 0; itr < obj_ptr_vec.size(); ++itr)
    {
      const Object &obj = *obj_ptr_vec[itr];
      obj_center[itr] = obj.GetCenter();
      obj_length[itr] = obj.GetBoxLength();
    }
    size_t axis = 0;
    root_ = build(obj_ptr_vec, obj_center, obj_length, obj_idx, axis);
  }
}

Node* jing_m::KdTree::build(const vector<Object*> &obj_ptr_vec, const vector<Vector3d> &obj_center, const vector<Vector3d> &obj_length, vector<size_t> &idx, size_t axis)
{
  assert(!idx.empty());
  if (idx.size() == 1)
  {
    Node *node = new Node;
    node->set_node(idx[0]);
    node->set_bounding_box(obj_ptr_vec, idx);

    return node;
  }
  else
  {
    // auto ret = max_element(idx.begin(), idx.end(),
    //                        [&](size_t lidx, size_t ridx) -> bool
    //                        {
    //                          return obj_length[lidx][axis] < obj_length[ridx][axis];
    //                        });
    // const size_t max_idx = distance(idx.begin(), ret);
    // assert(max_idx < idx.size());
    // node->set_node(idx[max_idx]);
    // idx.erase(ret);
    Node *node = new Node;
    node->set_bounding_box(obj_ptr_vec, idx);
    const size_t mid = idx.size() / 2;
    nth_element(idx.begin(), idx.begin() + mid, idx.end(),
                [&](size_t lidx, size_t ridx) -> bool
                {
                  return obj_center[lidx][axis] < obj_center[ridx][axis];
                });
    vector<size_t> lchild(idx.begin(), idx.begin() + mid);
    vector<size_t> rchild(idx.begin() + mid, idx.end());
    Node *l = build(obj_ptr_vec, obj_center, obj_length, lchild, (axis + 1) % 3);
    Node *r = build(obj_ptr_vec, obj_center, obj_length, rchild, (axis + 1) % 3);
    node->set_child(r, l);

    return node;
  }
}

jing_m::KdTree::~KdTree()
{
  clear(root_);
}

int jing_m::KdTree::clear(Node *node)
{
  if (node == nullptr)
  {
    return 0;
  }
  else
  {
    clear(node->GetLeft());
    clear(node->GetRight());
  }
  
  delete node;
  node = nullptr;

  return 0;
}

int jing_m::KdTree::GetNearObj(const vector<Object*> &obj_ptr_vec, const Beam &b, size_t &idx, double &dst) const
{
  GetKdTreeIt(obj_ptr_vec, root_, b.GetOrigin(), b.GetDir(), idx, dst);

  return 0;
}

int jing_m::KdTree::GetKdTreeIt(const vector<Object*> &obj_ptr_vec, Node *const node, const Vector3d &o, const Vector3d &d, size_t &idx, double &dst) const
{
  if (node == nullptr)
    return 0;

  const double *node_minB = node->GetMinB();
  const double *node_maxB = node->GetMaxB();

  bool ret = HitBoundingBox(const_cast<double*>(node_minB), const_cast<double*>(node_maxB), const_cast<double*>(&o[0]), const_cast<double*>(&d[0]));
  if (ret && node->GetId() != size_t(-1))
  {
    Vector3d minB;
    Vector3d maxB;
    const Object &obj = *obj_ptr_vec[node->GetId()];
    // obj.GetBoundingBox(minB, maxB);
    // bool tri = HitBoundingBox(const_cast<double*>(&minB[0]), const_cast<double*>(&maxB[0]), const_cast<double*>(&o[0]), const_cast<double*>(&d[0]));
    // if (tri)
    {
      double t = obj.GetIntersect(o, d);
      if (t < dst)
      {
        idx = node->GetId();
        dst = t;
      }
    }
  }

  if (ret)
  {
    GetKdTreeIt(obj_ptr_vec, node->GetLeft(), o, d, idx, dst);
    GetKdTreeIt(obj_ptr_vec, node->GetRight(), o, d, idx, dst);
  }

  return 0;
}
