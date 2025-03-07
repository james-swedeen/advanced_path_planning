/**
 * @File: node.hpp
 * @Date: May 2020
 * @Author: James Swedeen
 *
 * @brief
 * Used to hold information about one point in the RRT tree.
 **/

#ifndef RRT_SEARCH_TREE_NODE_HPP
#define RRT_SEARCH_TREE_NODE_HPP

/* C++ Headers */
#include<cstdint>
#include<memory>
#include<functional>
#include<list>
#include<vector>
#include<string>
#include<cmath>

/* Eigen Headers */
#include<Eigen/Dense>

namespace rrt
{
namespace tree
{
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
class Node;

/**
 * @DIM
 * The number of dimensions each point will have.
 *
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options.
 **/
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class Node
{
public:
  /**
   * @Default Constructor
   *
   * @brief
   * This constructor zeros everything out. Not to be used without
   * modifying the node after construction.
   **/
  inline Node() noexcept;
  /**
   * @Copy Constructor
   **/
  Node(const Node&) = delete;
  /**
   * @Move Constructor
   **/
  Node(Node&&) = delete;
  /**
   * @Minimal Constructor
   *
   * @brief
   * A minimalistic constructor used to make nodes independently of a RRT tree.
   *
   * @parameters
   * point: The state vector that this node is associated with
   * index: The index in the KD tree that this node is associated with
   * local_cost: The cost of the node
   **/
  inline Node(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>& point,
              const size_t                                            index,
              const SCALAR                                            local_cost) noexcept;
  /**
   * @Full Constructor
   *
   * @brief
   * Used to fully initialize this node.
   *
   * @parameters
   * edge: The edge that connects this node to it's parent
   * fillet: The fillet that connects this node's edge to the edge of this node's parent's edge
   * index: The index in the KD tree that this node is associated with
   * parent: The node that comes in the tree one step toured the root node
   * children: The nodes that come after this one in the tree
   * cost: The cost of the node
   * local_cost: The cost that this node alone contributes
   **/
  inline Node(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>& edge,
              const size_t                                            index,
                    Node*                                             parent,
              const std::list<Node*>&                                 children,
              const SCALAR                                            cost,
              const SCALAR                                            local_cost) noexcept;

  inline Node(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>& edge,
              const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>& fillet,
              const size_t                                            index,
                    Node*                                             parent,
              const std::list<Node*>&                                 children,
              const SCALAR                                            cost,
              const SCALAR                                            local_cost) noexcept;
  /**
   * @Deconstructor
   **/
  inline ~Node() noexcept = default;
  /**
   * @Copy Assignment Operator
   **/
  Node& operator=(const Node&) = delete;
  /**
   * @Move Assignment Operator
   **/
  Node& operator=(Node&&) = delete;
  /**
   * @isRoot
   *
   * @brief
   * Denotes weather or not this node is the root node. Functionally
   * equivalent to ~hasParent.
   *
   * @return
   * True if this node is the root node and false otherwise.
   **/
  inline bool isRoot() const noexcept;
  /**
   * @isEnd
   *
   * @brief
   * Denotes weather or not this node is at the end of an arm
   * the RRT tree. Functionally equivalent to having no children.
   *
   * @return
   * True if this node has no children and false otherwise.
   **/
  inline bool isEnd() const noexcept;
  /**
   * @numberOfChildren
   *
   * @brief
   * Used to see how many children this node has.
   *
   * @return
   * The number of children this node has.
   **/
  inline size_t numberOfChildren() const noexcept;
  /**
   * @hasParent
   *
   * @brief
   * Used to see if this node has a parent.
   *
   * @brief
   * True if this node has a parent and false otherwise.
   **/
  inline bool hasParent() const noexcept;
  /**
   * @addChild
   *
   * @brief
   * Used to add a child to this node.
   *
   * @parameters
   * child: The child to be added
   **/
  inline void addChild(Node* child);
  /**
   * @removeChild
   *
   * @brief
   * Used to remove children from this node.
   *
   * @parameters
   * child: The child to be removed
   **/
  inline void removeChild(const Node* child);
  /**
   * @size
   *
   * @brief
   * The length of the edge that connects this node to its parent.
   *
   * @return
   * The length of the edge that connects this node to its parent.
   **/
  inline Eigen::Index size() const noexcept;
  /**
   * @set
   *
   * @brief
   * Used to modify internally held parameters.
   *
   * @parameters
   * edge: The edge that connects this node to it's parent
   * fillet: The fillet that connects this node's edge to the edge of this node's parent's edge
   * index: The index in the KD tree that this node is associated with
   * parent: The node that comes in the tree one step toured the root node
   * children: The nodes that come after this one in the tree
   * cost: The cost of the node
   * local_cost: The cost that this node alone contributes
   *
   * @return
   * The new value.
   **/
  inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>&
    setEdge(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>& edge) noexcept;
  inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>&
    setFillet(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>& fillet) noexcept;
  inline size_t            setIndex(    const size_t            index)      noexcept;
  inline Node*             setParent(         Node*             parent)     noexcept;
  inline std::list<Node*>& setChildren( const std::list<Node*>& children)   noexcept;
  inline SCALAR            setCost(     const SCALAR            cost)       noexcept;
  inline SCALAR            setLocalCost(const SCALAR            local_cost) noexcept;
  /**
   * @get
   *
   * @brief
   * Used to access internally held variables.
   *
   * @return
   * The asked for variable.
   **/
  inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>& getEdge()     noexcept;
  inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>& getFillet()   noexcept;
  inline Node*                                             getParent()   noexcept;
  inline std::list<Node*>&                                 getChildren() noexcept;
  /**
   * @cget
   *
   * @brief
   * Used to access internally held variables in a const way.
   *
   * @return
   * The asked for variable.
   **/
  inline       Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>              cgetPoint()           const noexcept;
  inline       Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>              cgetFilletEnd()       const noexcept;
  inline       Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>> cgetEdge()            const noexcept;
  inline       Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>> cgetFillet()          const noexcept;
  inline       size_t                                                             cgetIndex()           const noexcept;
  inline const Node*                                                              cgetParent()          const noexcept;
  inline const std::list<Node*>&                                                  cgetChildren()        const noexcept;
  inline       std::vector<size_t>                                                cgetChildrenIndexes() const noexcept;
  inline       SCALAR                                                             cgetCost()            const noexcept;
  inline       SCALAR                                                             cgetLocalCost()       const noexcept;
private:
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> edge;
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> fillet;
  size_t                                           index;
  Node*                                            parent;
  std::list<Node*>                                 children;
  SCALAR                                           cost;
  SCALAR                                           local_cost;
};

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
Node<DIM,SCALAR,OPTIONS>::Node() noexcept
 : index(0),
   parent(nullptr),
   cost(0),
   local_cost(0)
{}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Node<DIM,SCALAR,OPTIONS>::Node(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>& point,
                                      const size_t                                            index,
                                      const SCALAR                                            local_cost) noexcept
 : edge(point),
   fillet(point.template bottomRows<1>()),
   index(index),
   parent(nullptr),
   cost(local_cost),
   local_cost(local_cost)
{}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Node<DIM,SCALAR,OPTIONS>::Node(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>& edge,
                                      const size_t                                            index,
                                            Node*                                             parent,
                                      const std::list<Node*>&                                 children,
                                      const SCALAR                                            cost,
                                      const SCALAR                                            local_cost) noexcept
 : edge(edge),
   index(index),
   parent(parent),
   children(children),
   cost(cost),
   local_cost(local_cost)
{}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Node<DIM,SCALAR,OPTIONS>::Node(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>& edge,
                                      const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>& fillet,
                                      const size_t                                            index,
                                            Node*                                             parent,
                                      const std::list<Node*>&                                 children,
                                      const SCALAR                                            cost,
                                      const SCALAR                                            local_cost) noexcept
 : edge(edge),
   fillet(fillet),
   index(index),
   parent(parent),
   children(children),
   cost(cost),
   local_cost(local_cost)
{}


template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool Node<DIM,SCALAR,OPTIONS>::isRoot() const noexcept
{
  return !this->hasParent();
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool Node<DIM,SCALAR,OPTIONS>::isEnd() const noexcept
{
  return this->cgetChildren().empty();
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline size_t Node<DIM,SCALAR,OPTIONS>::numberOfChildren() const noexcept
{
  return this->cgetChildren().size();
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool Node<DIM,SCALAR,OPTIONS>::hasParent() const noexcept
{
  return this->cgetParent() != nullptr;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void Node<DIM,SCALAR,OPTIONS>::addChild(Node* child)
{
#ifndef NDEBUG
  for(auto child_it = this->cgetChildren().cbegin(); child_it != this->cgetChildren().cend(); child_it++)
  {
    if(*child_it == child)
    {
      throw std::runtime_error("Warning, trying to add a duplicate child");
    }
  }
#endif
  this->getChildren().emplace_front(child);
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void Node<DIM,SCALAR,OPTIONS>::removeChild(const Node* child)
{
#ifndef NDEBUG
  const size_t pre_size = this->numberOfChildren();
#endif
  const auto children_end = this->getChildren().end();
  for(auto child_it = this->getChildren().begin(); child_it != children_end; ++child_it)
  {
    if(child == *child_it)
    {
      this->getChildren().erase(child_it);
      break;
    }
  }
#ifndef NDEBUG
  if(this->numberOfChildren() != (pre_size - 1))
  {
    throw std::runtime_error(std::string("Warning inconsistent number of children of been removed. ") +
      "Before removal this node had " + std::to_string(pre_size) + " children and now it has " + std::to_string(this->numberOfChildren()));
  }
#endif
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Index Node<DIM,SCALAR,OPTIONS>::size() const noexcept
{
  return this->cgetEdge().rows();
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>& Node<DIM,SCALAR,OPTIONS>::
  setEdge(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>& other_edge) noexcept
{
  return (this->edge = other_edge);
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>& Node<DIM,SCALAR,OPTIONS>::
  setFillet(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>& fillet) noexcept
{
  return (this->fillet = fillet);
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline size_t Node<DIM,SCALAR,OPTIONS>::setIndex(const size_t index) noexcept
{
  return (this->index = index);
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Node<DIM,SCALAR,OPTIONS>* Node<DIM,SCALAR,OPTIONS>::setParent(Node* parent) noexcept
{
  return (this->parent = parent);
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline std::list<Node<DIM,SCALAR,OPTIONS>*>& Node<DIM,SCALAR,OPTIONS>::
  setChildren(const std::list<Node*>& children) noexcept
{
  return (this->getChildren() = children);
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR Node<DIM,SCALAR,OPTIONS>::setCost(const SCALAR cost) noexcept
{
  return (this->cost = cost);
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR Node<DIM,SCALAR,OPTIONS>::setLocalCost(const SCALAR local_cost) noexcept
{
  return (this->local_cost = local_cost);
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>& Node<DIM,SCALAR,OPTIONS>::getEdge() noexcept
{
  return this->edge;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>& Node<DIM,SCALAR,OPTIONS>::getFillet() noexcept
{
  return this->fillet;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Node<DIM,SCALAR,OPTIONS>* Node<DIM,SCALAR,OPTIONS>::getParent() noexcept
{
  return this->parent;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline std::list<Node<DIM,SCALAR,OPTIONS>*>& Node<DIM,SCALAR,OPTIONS>::getChildren() noexcept
{
  return this->children;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>> Node<DIM,SCALAR,OPTIONS>::cgetPoint() const noexcept
{
  return this->cgetEdge().template bottomRows<1>();
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>
  Node<DIM,SCALAR,OPTIONS>::cgetFilletEnd() const noexcept
{
  return this->cgetFillet().template bottomRows<1>();
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>
  Node<DIM,SCALAR,OPTIONS>::cgetEdge() const noexcept
{
  return this->edge;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>
  Node<DIM,SCALAR,OPTIONS>::cgetFillet() const noexcept
{
  return this->fillet;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline size_t Node<DIM,SCALAR,OPTIONS>::cgetIndex() const noexcept
{
  return this->index;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const Node<DIM,SCALAR,OPTIONS>* Node<DIM,SCALAR,OPTIONS>::cgetParent() const noexcept
{
  return this->parent;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const std::list<Node<DIM,SCALAR,OPTIONS>*>& Node<DIM,SCALAR,OPTIONS>::cgetChildren() const noexcept
{
  return this->children;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline std::vector<size_t> Node<DIM,SCALAR,OPTIONS>::cgetChildrenIndexes() const noexcept
{
  std::vector<size_t> output(this->numberOfChildren());

  const auto end_it = this->cgetChildren().cend();
  size_t index = 0;
  for(auto child_it = this->cgetChildren().cbegin(); child_it != end_it; ++child_it)
  {
    output[index++] = (*child_it)->cgetIndex();
  }

  return output;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR Node<DIM,SCALAR,OPTIONS>::cgetCost() const noexcept
{
  return this->cost;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR Node<DIM,SCALAR,OPTIONS>::cgetLocalCost() const noexcept
{
  return this->local_cost;
}
} // namespace tree
} // namespace rrt

#endif
/* node.hpp */
