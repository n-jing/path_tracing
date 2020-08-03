#ifndef KD_TREE_JJ_H
#define KD_TREE_JJ_H

#include <vector>
#include <Eigen/Core>

class Object;
class Beam;

class Node
{
public:
  Node();
  ~Node();
  int set_node(size_t idx);
  int set_child(Node *const l, Node *const r);
  int set_bounding_box(const std::vector<Object*> &obj_ptr_vec, const std::vector<size_t> &idx);
  size_t GetId() const;
  Node *GetLeft() const;
  Node *GetRight() const;
  const double *GetMinB() const;
  const double *GetMaxB() const;
  
private:
  size_t idx_;
  Node *left_;
  Node *right_;
  Eigen::Vector3d minB_;
  Eigen::Vector3d maxB_;
};

namespace jing_m
{
  class KdTree
  {
  public:
    KdTree(const std::vector<Object*> &obj_ptr_vec);
    ~KdTree();

    int GetNearObj(const std::vector<Object*> &obj_ptr_vec, const Beam &b, size_t &idx, double &dst) const;
  private:
    Node* build(const std::vector<Object*> &obj_ptr_vec, const std::vector<Eigen::Vector3d> &obj_center, const std::vector<Eigen::Vector3d> &obj_length, std::vector<size_t> &idx, size_t axis);
    int GetKdTreeIt(const std::vector<Object*> &obj_ptr_vec, Node *const node, const Eigen::Vector3d &o, const Eigen::Vector3d &d, size_t &idx, double &dst) const;

    int clear(Node *node);
    Node *root_;
  };
}

#endif // KD_TREE_JJ_H
