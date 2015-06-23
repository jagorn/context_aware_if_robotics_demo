#ifndef A_STAR_IMPLEMENTATION
#define A_STAR_IMPLEMENTATION

#include <map>
#include <set>
#include <vector>

#include "../utils/point.h"
#include "../utils/utils.h"

//#define EUCLEDIAN_DISTANCE

class AstarHandler
{
public:
    class Node
    {
    public:
        int id;
        Utils::Point2f pos;

        float g_score;
        float f_score;

        Node(int _id, Utils::Point2f _p): id(_id), pos(_p){}
        Node(const Node& n)
        {
            id = n.id;
            pos = n.pos;
            g_score = n.g_score;
            f_score = n.f_score;
        }
        struct cmp
        {
            bool operator()(Node left, Node right) const
            {
                if(left.id != right.id)return left.id < right.id;
                else return left.f_score <= right.f_score;
            }


        };
        Node clone(){return *this;}

    };

    AstarHandler(){}
    AstarHandler(const std::vector<Node*>& _g, const std::map<int,std::vector<int> >& _c):
        graph(_g), connections(_c)
    {
        for(unsigned int i=0; i<graph.size(); ++i)
        {
            std::vector<float> tmp_init(graph.size(), 0.f);
            context_costs.push_back(tmp_init);
        }
    }
    ~AstarHandler()
    {
        clear();
    }
    void clear();
    void reset();
    int getClosetNodeId(cv::Point2f p);
    float heuristicCostEstimate(Node* ni, Node* ne);
    void removeFromSet(int id, std::set<Node,Node::cmp>* _set);
    bool isInSet(int id, std::set<Node,Node::cmp>* _set);
    int getLowestFscoreNode();
    void retracePath(int c_id, int s_id, std::vector<int>* _path);
    void setContextCosts(std::vector<std::vector<float> >* _ncc);
    // s_id: start node id, e_id: end node id, _path: path pointer where the path will be stored
    bool findPath(int s_id, int e_id, std::vector<int>* _path);

private:
    std::vector<Node*> graph;
    std::map<int, std::vector<int> > connections;
    std::vector<std::vector<float> > context_costs;


    std::set<Node,Node::cmp> closed_set;
    std::set<Node,Node::cmp> open_set;
    std::map<int, Node*> history;
};

#endif
