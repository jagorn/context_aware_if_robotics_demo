#ifndef _TOP_GRAPH_GENERATOR_
#define _TOP_GRAPH_GENERATOR_

#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "./utils/utils_proximity.h"
#include "./utils/a_star.h"

//#define _TOP_GRAPH_MAIN_
//#define VIS_NODE_ID
//#define GENERATE_EDGES_LP

#define INTRA_NODE_DISTANCE 5
#define NODE_DISTANCE 10
#define EDGE_MAX_DIST 15

class TopGraph
{

public:
    class Area
    {
    public:
        std::string id;
        std::string label;
        cv::Point2f centroid;
        float weight;
        std::vector<float> params;

        Area(std::string _i, std::string _l, cv::Point2f _c, float _w, std::vector<float> _p):
            id(_i), label(_l), centroid(_c), weight(_w), params(_p){}
        Area(const Area& a)
        {
            id = a.id;
            label = a.label;
            centroid = a.centroid;
            weight = a.weight;
            params = a.params;
        }
        ~Area(){}
    };

    class Node
    {
    public:

        class Edge
        {
        public:
            int end_point_id;
            float cost;

            Edge(int ep_id, float _cost = 0): end_point_id(ep_id), cost(_cost){}
            ~Edge(){}
        };

        int id;
        cv::Point2f pos;
        std::vector<Node::Edge*> edges;

        Node(int _id, cv::Point2f _pos): id(_id), pos(_pos){}
        ~Node(){}

        void setEdges(std::vector<TopGraph::Node::Edge*> _edges) { edges = _edges; }
    };

    TopGraph(cv::Mat img, std::string _output_file_name = " ");
    ~TopGraph(){}

    void updateContextCosts(std::vector<float>* costs_weights);
    void updateContextAreas(std::vector<Area>* _areas);
    void computePath(cv::Point3f start_node, cv::Point3f end_node, std::vector<cv::Point3f>* _path);
    void vis(cv::Mat vis);

    void generateFileLP();

    inline unsigned int getNodeNum(){ return graph.size(); }

private:
    cv::Mat map;
    std::string output_file_name;
    std::vector<Node*> graph;
    std::vector<Area> areas;

    AstarHandler* astar_handler;

    void generateDenseGraph();


};


#endif
