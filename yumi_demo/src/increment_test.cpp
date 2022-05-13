#include <iostream>
#include <vector>
#include <fstream>
#include <cstring>
#include <math.h>
#include <algorithm>

using namespace std;

struct BVHNode{
    int id;
    struct BVHNode* parent;
    struct BVHNode* lchild;
    struct BVHNode* rchild;

    BVHNode() {};
    BVHNode(int idx) {id = idx;}
};

BVHNode *bvh2;
BVHNode *bvh1;

vector<BVHNode> bvh1list;
vector<BVHNode> bvh2list;

struct SeparationNode{
    int node1;
    int node2;
};

vector<SeparationNode> SeparationList;

struct CollisionCheckNode{
    SeparationNode node;
    bool isoverlap;
};

vector<CollisionCheckNode> CollisionCheckList;

int UpdatingCnt = 0;

void buildSeparationList(BVHNode* &node1, BVHNode* &node2, int depth);
void maintainingSeparationList();
void movedown();
void moveup();
void SeparationListPrint();

void CCLInit();
void BVHbuild(BVHNode* &node, int idx, int depth, int k);
bool isoverlap(int a, int b);
int getparentID(int BVHid, int id);
bool compare(BVHNode& a, BVHNode& b);
void preOrder(vector<BVHNode>& list, BVHNode* node);

void CCLInit()
{
    char buffer[10];
    ifstream infile("/home/monica/yumi/src/yumi_demo/scripts/CollisionCheckList.txt");
    if (!infile.is_open()) {
        cerr << "Error opening file";
        exit(1);
    }
    while (!infile.eof()) {
        infile.getline(buffer, 10);
        char *tokenPtr = strtok(buffer, " ");
        CollisionCheckNode tmp;
        tmp.node.node1 = atoi(tokenPtr);
        tokenPtr = strtok(NULL, " ");
        tmp.node.node2 = atoi(tokenPtr);
        tokenPtr = strtok(NULL, " ");
        tmp.isoverlap = atoi(tokenPtr)?true:false;
//        cout << tmp.node.node1 << " " << tmp.node.node2 << " " << boolalpha << tmp.isoverlap << "\n";
        CollisionCheckList.push_back(tmp);
    }
    infile.close();
}

void BVHbuild(BVHNode* &node, int idx, int depth, int k) {
    node = new BVHNode;
    node->id = idx;
    if (depth < k) {
        BVHbuild(node->lchild, idx * 2, depth + 1, k);
        BVHbuild(node->rchild, idx * 2 + 1, depth + 1, k);
        node->lchild->parent = node;
        node->rchild->parent = node;
    }
    else {
        node->lchild = NULL;
        node->rchild = NULL;
    }
}

void BVHInit(int k, int l) {
    BVHbuild(bvh1, 1, 1, k);
    BVHbuild(bvh2, 1, 1, l);
    preOrder(bvh1list, bvh1);
    preOrder(bvh2list, bvh2);
    sort(bvh1list.begin(), bvh1list.begin() + bvh1list.size(), compare);
    sort(bvh2list.begin(), bvh2list.begin() + bvh2list.size(), compare);
    cout << "BVHInit OK!\n";
}

bool isoverlap(int a, int b)
{
    for (auto &t: CollisionCheckList) {
        if (t.node.node1 == a && t.node.node2 == b)
            return t.isoverlap;
        else continue;
    }
    cerr << "Error founding corresponding CollisionCheckNode\n";
    exit(1);
}

bool compare(BVHNode& a, BVHNode& b)
{
    return a.id<b.id?true:false;
}

void preOrder(vector<BVHNode>& list, BVHNode* node)
{
    if (node) {
        list.push_back(*node);
        preOrder(list, node->lchild);
        preOrder(list,node->rchild);
    }
}

int getparentID(int BVHid, int id)
{
    // list[i-1] id: i
    if (BVHid == 1)
        return bvh1list[id-1].parent->id;
    else if (BVHid == 2)
        return bvh2list[id-1].parent->id;
    else {
        cerr << "Error getting parent ID\n";
        exit(1);
    }
}

void callback()
{
    if (SeparationList.empty() || UpdatingCnt > 1) {
        buildSeparationList(bvh1, bvh2, 1);
    } else {
        maintainingSeparationList();
    }
}

void buildSeparationList(BVHNode* &node1, BVHNode* &node2, int depth)
{
    if (isoverlap(node1->id, node2->id)) {
        SeparationNode tmp;
        tmp.node1 = node1->id;
        tmp.node2 = node2->id;

        if (SeparationList.empty() || (SeparationList.at(SeparationList.size()-1).node1 == node1->id
                                       && SeparationList.at(SeparationList.size()-1).node2 == node2->id)) {
            SeparationList.pop_back();
            SeparationList.push_back(tmp);
        }
        else {
            SeparationList.push_back(tmp);
        }

        if (depth % 2 == 1 && bvh1list.at(node1->id - 1).lchild != NULL) {
            // descend on the left & check whether internal node
            buildSeparationList(node1->lchild, node2, depth + 1);
            buildSeparationList(node1->rchild, node2, depth + 1);
        }
        else {
            if (bvh2list.at(node2->id - 1).lchild != NULL) {
                // descend on the right & check whether internal node
                buildSeparationList(node1, node2->lchild, depth + 1);
                buildSeparationList(node1, node2->rchild, depth + 1);
            }
            else
            {
                // the branch traversed to the bottom
            }
        }
    }
}

void SeparationListPrint()
{
    for (int i = 0; i < SeparationList.size(); i++)
        cout << SeparationList[i].node1 << " " << SeparationList[i].node2 << "\n";
}

void maintainingSeparationList()
{

}

void movedown()
{

}

void moveup()
{

}

int main()
{
    CCLInit();
    BVHInit(3, 3);
    buildSeparationList(bvh1, bvh2, 1);
    SeparationListPrint();
    return 0;
}

