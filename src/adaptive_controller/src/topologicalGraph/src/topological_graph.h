#ifndef _TOP_GRAPH_GENERATOR_
#define _TOP_GRAPH_GENERATOR_

#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "utils/utils_proximity.h"
#include "utils/a_star.h"

#define _TOP_GRAPH_MAIN_

#define NODE_DISTANCE 10
#define INTRA_NODE_DISTANCE 20
#define EDGE_MAX_DIST 45

class TopGraph
{
public:
    TopGraph(cv::Mat img, std::string _output_file_name = " ");
    ~TopGraph(){}

    void computePath(cv::Point3f start_node, cv::Point3f end_node, std::vector<cv::Point3f>* _path);
    void vis();

    void generateFileLP();

private:

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

    cv::Mat map;
    std::string output_file_name;
    std::vector<Node*> graph;

    AstarHandler* astar_handler;

    void generateDenseGraph();
};


#endif
