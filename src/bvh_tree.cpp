#include "bvh_tree.h"

template<typename BV>
BvhTree<BV>::BvhTree(){
    root_node = nullptr;
    n_leaves = 0;
    bu_threshold = 2;
}

template<typename BV>
BvhTree<BV>::~BvhTree(){
    clear();
}

template<typename BV>
void BvhTree<BV>::clear(){
    if(root_node){
        recurseDeleteNode(root_node);
    }
    n_leaves = 0;

}

template<typename BV>
size_t BvhTree<BV>::size() const{
    return n_leaves;
}

template<typename BV>
void BvhTree<BV>::recurseDeleteNode(NodeType* node){
    if(!node->isLeaf()){
        recurseDeleteNode(node->children[0]);
        recurseDeleteNode(node->children[1]);
    }

    if(node == root_node) root_node = nullptr;
    delete(node);
}

template<typename BV>
void BvhTree<BV>::deleteNode(NodeType* node){
    delete node;
}

template<typename BV>
typename BvhTree<BV>::NodeType* BvhTree<BV>::getRoot() const{
    return root_node;
}

template<typename BV>
typename BvhTree<BV>::NodeType*& BvhTree<BV>::getRoot(){
    return root_node;
}


template<typename BV>
void BvhTree<BV>::init(std::vector<NodeType*>& leaves){
    clear();
    root_node = topdown(leaves.begin(), leaves.end());
    n_leaves = leaves.size();
}

//==============================================================================
template<typename BV>
bool nodeBaseLess(NodeBase<BV>* a, NodeBase<BV>* b, int d)
{
  if(a->bv.center()[d] < b->bv.center()[d]) return true;
  return false;
}

template<typename BV>
typename BvhTree<BV>::NodeType* BvhTree<BV>::topdown(const NodeVecIterator lbeg, const NodeVecIterator lend)
{
    int num_leaves = lend - lbeg;
    if(num_leaves > 1){
        if(num_leaves > bu_threshold){
            BV vol = (*lbeg)->bv;
            for(NodeVecIterator it = lbeg + 1; it < lend; ++it)
                vol += (*it)->bv;

            int best_axis = 0;
            S extent[3] = {vol.width(), vol.height(), vol.depth()};
            if(extent[1] > extent[0]) best_axis = 1;
            if(extent[2] > extent[best_axis]) best_axis = 2;

            // compute median
            NodeVecIterator lcenter = lbeg + num_leaves / 2;
            std::nth_element(lbeg, lcenter, lend, std::bind(&nodeBaseLess<BV>, std::placeholders::_1, std::placeholders::_2, std::ref(best_axis))); //对节点范围进行部分排序，并选择一个中间点作为切割点

            NodeType* node = createNode(nullptr, vol, nullptr);
            node->children[0] = topdown(lbeg, lcenter);
            node->children[1] = topdown(lcenter, lend);
            node->children[0]->parent = node;
            node->children[1]->parent = node;
            return node;
        }
        else{
            bottomup(lbeg, lend);
            return *lbeg;
        }
    }
    return *lbeg;
}


template<typename BV>
void BvhTree<BV>::bottomup(const NodeVecIterator lbeg, const NodeVecIterator lend)
{

  NodeVecIterator lcur_end = lend;
  while(lbeg < lcur_end - 1)
  {
    NodeVecIterator min_it1, min_it2;
    S min_size = std::numeric_limits<S>::max();
    for(NodeVecIterator it1 = lbeg; it1 < lcur_end; ++it1)
    {
      for(NodeVecIterator it2 = it1 + 1; it2 < lcur_end; ++it2)
      {
        S cur_size = ((*it1)->bv + (*it2)->bv).size();
        if(cur_size < min_size)
        {
          min_size = cur_size;
          min_it1 = it1;
          min_it2 = it2;
        }
      }
    }

    NodeType* n[2] = {*min_it1, *min_it2};
    NodeType* p = createNode(nullptr, n[0]->bv + n[1]->bv, nullptr);
    p->children[0] = n[0];
    p->children[1] = n[1];
    n[0]->parent = p;
    n[1]->parent = p;
    *min_it1 = p;
    NodeType* tmp = *min_it2;
    lcur_end--;
    *min_it2 = *lcur_end;
    *lcur_end = tmp;
  }
}


template<typename BV>
typename BvhTree<BV>::NodeType* BvhTree<BV>::createNode(NodeType* parent, const BV& bv, CollisionObject<double>* data){
    NodeType* node = new NodeType(); // 分配内存并初始化 node
    node->parent = parent;
    node->data = data;
    node->children[1] = 0;
    node->bv = bv;
    return node;
}

template<typename BV>
void BvhTree<BV>::print(NodeType* root, int depth){
    for(int i = 0; i < depth; ++i)
        std::cout << "   ";
    std::cout << " (" << root->bv.min_[0] << ", " << root->bv.min_[1] << ", " << root->bv.min_[2] << "; " << root->bv.max_[0] << ", " << root->bv.max_[1] << ", " << root->bv.max_[2] << ")" << std::endl;
    if(root->isLeaf())
    {
    }
    else
    {
        print(root->children[0], depth+1);
        print(root->children[1], depth+1);
    }
}


template class BvhTree<AABB<double>>;