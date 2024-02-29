// Fig. 19.22: Tree.h
// Tree class-template definition.
#ifndef TREE_H
#define TREE_H

#include <iostream>
#include "TreeNode.h"

// Tree class-template definition
template<typename NODETYPE> class Tree {
public:
   // insert node in Tree
   void insertNode(const NODETYPE& value) {
      insertNodeHelper(&rootPtr, value); 
   } 

   // begin preorder traversal of Tree
   void preOrderTraversal() const {
      preOrderHelper(rootPtr); 
   } 

   // begin inorder traversal of Tree
   void inOrderTraversal() const {
      inOrderHelper(rootPtr); 
   } 

   // begin postorder traversal of Tree
   void postOrderTraversal() const {
      postOrderHelper(rootPtr); 
   }
   
   /**
    * @brief Searches for a specific value in the tree
    * 
    * Iterates over the tree to search for a wanted value, if the 
    * value is found then it returns the position in the tree, if not
    * then returns a null pointer
    * 
    * @param const NODETYPE& valueToSearch
    * @return TreeNode<NODETYPE>* Pointer to the node containing the 
    * searched value, or nullptr if not found.
   */
   TreeNode<NODETYPE>* search(const NODETYPE& valueToSearch)
   {
      return searchHelper(rootPtr, valueToSearch);
   }

   /**
    * @brief Outputs the elements of the tree
    * 
    * Prints the whole tree in a piramid like structure using the 
    * helper function outputTreeHelper() to iterate over the nodes
    * of the tree and the different levels of the tree.
    * 
    * @return void
   */
   void outputTree()
   {
      outputTreeHelper(rootPtr, 0);
      std::cout << std::endl; // Move to the next line after outputting the tree
   }

private:

   TreeNode<NODETYPE>* rootPtr{nullptr};

   /**
    * @brief Checks if current node is the value wanted
    * 
    * Iterates over the tree and checks if the node has the value wanted,
    * if it does, returns on the pointer to that location, if not then 
    * returns a null pointer.
    * 
    * @param TreeNode<NODETYPE>* ptr
    * @param const NODETYPE& valueToSearch
    * @return TreeNode<NODETYPE>* Pointer to the node containing the 
    * searched value, or nullptr if not found.
   */
   TreeNode<NODETYPE>* searchHelper(TreeNode<NODETYPE>* ptr, const NODETYPE& valueToSearch) const 
   {
      // If pointer null or value is the one wanted then return it
      if (ptr == nullptr || valueToSearch == ptr->data) 
      {
         return ptr;
      } 
   
      if (valueToSearch < ptr->data) 
      {
         return searchHelper(ptr->leftPtr, valueToSearch);
      } 
      else 
      {
         return searchHelper(ptr->rightPtr, valueToSearch);
      }
   }

   /**
    * @brief Prints the current level of the tree recursivetly
    * 
    * Prints a level of the tree by printing the right branch of the tree 
    * of a higher level, later it prints the left branch of the tree of a 
    * higher level. 
    * 
    * @param TreeNode<NODETYPE>* ptr
    * @param int level
    * @return void
   */
   void outputTreeHelper(TreeNode<NODETYPE>* ptr, int level) const
   {
      // If node is empty then ignore
      if(ptr == nullptr) 
      {
         return;
      }

      outputTreeHelper(ptr->rightPtr, level + 1); // traverse right subtree

      // Indent based on the level
      for (int i = 0; i < level; ++i) 
      {
         std::cout << "    ";
      }

      std::cout << ptr->data << std::endl; // process node

      outputTreeHelper(ptr->leftPtr, level + 1); // traverse left subtree
   }

   // utility function called by insertNode; receives a pointer
   // to a pointer so that the function can modify pointer's value
   void insertNodeHelper(
      TreeNode<NODETYPE>** ptr, const NODETYPE& value) {
      // subtree is empty; create new TreeNode containing value
      if (*ptr == nullptr) {
         *ptr = new TreeNode<NODETYPE>(value);
      }
      else { // subtree is not empty
             // data to insert is less than data in current node
         if (value < (*ptr)->data) {
            insertNodeHelper(&((*ptr)->leftPtr), value);
         }
         else {
            // data to insert is greater than data in current node
            if (value > (*ptr)->data) { 
               insertNodeHelper(&((*ptr)->rightPtr), value);
            }
            else { // duplicate data value ignored
               std::cout << value << " dup" << std::endl;
            }
         } 
      } 
   } 

   // utility function to perform preorder traversal of Tree
   void preOrderHelper(TreeNode<NODETYPE>* ptr) const {
      if (ptr != nullptr) {
         std::cout << ptr->data << ' '; // process node               
         preOrderHelper(ptr->leftPtr); // traverse left subtree  
         preOrderHelper(ptr->rightPtr); // traverse right subtree
      } 
   } 

   // utility function to perform inorder traversal of Tree
   void inOrderHelper(TreeNode<NODETYPE>* ptr) const {
      if (ptr != nullptr) {
         inOrderHelper(ptr->leftPtr); // traverse left subtree  
         std::cout << ptr->data << ' '; // process node              
         inOrderHelper(ptr->rightPtr); // traverse right subtree
      } 
   } 

   // utility function to perform postorder traversal of Tree
   void postOrderHelper(TreeNode<NODETYPE>* ptr) const {
      if (ptr != nullptr) { 
         postOrderHelper(ptr->leftPtr); // traverse left subtree  
         postOrderHelper(ptr->rightPtr); // traverse right subtree
         std::cout << ptr->data << ' '; // process node                
      } 
   } 
}; 

#endif


/**************************************************************************
 * (C) Copyright 1992-2017 by Deitel & Associates, Inc. and               *
 * Pearson Education, Inc. All Rights Reserved.                           *
 *                                                                        *
 * DISCLAIMER: The authors and publisher of this book have used their     *
 * best efforts in preparing the book. These efforts include the          *
 * development, research, and testing of the theories and programs        *
 * to determine their effectiveness. The authors and publisher make       *
 * no warranty of any kind, expressed or implied, with regard to these    *
 * programs or to the documentation contained in these books. The authors *
 * and publisher shall not be liable in any event for incidental or       *
 * consequential damages in connection with, or arising out of, the       *
 * furnishing, performance, or use of these programs.                     *
 **************************************************************************/
