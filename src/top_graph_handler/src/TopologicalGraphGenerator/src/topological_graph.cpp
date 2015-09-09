
#include "topological_graph.h"


TopGraph::TopGraph(cv::Mat img, std::string _output_file_name)
{
    map = img;
    output_file_name = _output_file_name;
    generateDenseGraph();
}

void TopGraph::vis(cv::Mat vis)
{
//    cv::cvtColor(map, vis, CV_GRAY2RGB);

    for(unsigned int a=0; a<areas.size(); ++a)
    {
        if(areas.at(a).label=="circle")
            Utils::drawCircleArea(vis, areas.at(a).id, areas.at(a).centroid,
                                  areas.at(a).params.at(0), areas.at(a).weight);
        else if(areas.at(a).label=="rect")
            Utils::drawRectArea(vis, areas.at(a).id, areas.at(a).centroid,
                                areas.at(a).params.at(0), areas.at(a).params.at(1), areas.at(a).weight);

    }

    for(unsigned int i=0; i< graph.size(); ++i)
    {
        for(unsigned int e=0; e<graph.at(i)->edges.size(); ++e)
            cv::line(vis, graph.at(i)->pos, graph.at(graph.at(i)->edges.at(e)->end_point_id)->pos, CV_RGB(0,180,0), 1);

        cv::circle( vis, graph.at(i)->pos, 3, CV_RGB(0,80,0), CV_FILLED );
    }
}

void TopGraph::generateFileLP()
{
    std::ofstream out_file(output_file_name.c_str());
    if (out_file.is_open() )
    {
        out_file << "% Nodes --------------------------- \n";
        for(unsigned n=0; n< graph.size(); ++n)
        {
            out_file << "node("
                        + Utils::to_string(graph.at(n)->id) + ","
                        + Utils::to_string(graph.at(n)->pos.x) + ","
                        + Utils::to_string(graph.at(n)->pos.y) +").\n";
        }

#ifdef GENERATE_EDGES_LP
        out_file << "\n % Edges --------------------------- \n";
        for(unsigned n=0; n< graph.size(); ++n)
        {
            for(unsigned int e=0; e<graph.at(n)->edges.size(); ++e)
            {
                out_file << "edge("
                            + Utils::to_string(graph.at(n)->id) + ","
                            + Utils::to_string(graph.at(n)->edges.at(e)->end_point_id) + ","
                            + Utils::to_string((int)graph.at(n)->edges.at(e)->cost) + ").\n";
            }
        }
#endif

        out_file.close();
    }
    else Utils::println("[TopGraph::generateFileLP] File < "+ output_file_name + "> not found!",Utils::Red);
}

void TopGraph::generateDenseGraph()
{
    cv::Mat binary_map;
    cv::threshold( map, binary_map, 200, 255, CV_THRESH_BINARY);

    /// generate nodes
    for(int i=0; i< binary_map.rows; i=i+NODE_DISTANCE)
    {
        for(int j=0; j< binary_map.cols; j=j+NODE_DISTANCE)
        {
            if( (int)binary_map.at<uchar>(i, j) == 255 )
            {
                if((int)binary_map.at<uchar>(i-INTRA_NODE_DISTANCE, j) == 0) continue;
                if((int)binary_map.at<uchar>(i, j-INTRA_NODE_DISTANCE) == 0) continue;
                if((int)binary_map.at<uchar>(i+INTRA_NODE_DISTANCE, j) == 0) continue;
                if((int)binary_map.at<uchar>(i, j+INTRA_NODE_DISTANCE) == 0) continue;

                graph.push_back( new Node(graph.size(), cv::Point2f(j,i)) );

#ifdef VIS_NODE_ID
                Utils::number(map, graph.size()-1, cv::Point2f(j,i));
#endif
            }
        }
    }

    /// connect nodes
    std::vector<AstarHandler::Node*> astar_rep;
    std::map<int, std::vector<int> > astar_connections;

    for(unsigned int n=0; n<graph.size(); ++n)
    {
        astar_rep.push_back( new AstarHandler::Node(n, Utils::cv2co_point(graph.at(n)->pos)) );
        std::vector<int> tmp_edges;
        std::vector<Node::Edge*> tmp_node_edges;

        for(unsigned int c=0; c<graph.size(); ++c)
        {
            if(n==c) continue;
            if( norm(graph.at(n)->pos-graph.at(c)->pos ) <= EDGE_MAX_DIST)
            {
                tmp_edges.push_back(c);
                tmp_node_edges.push_back(new Node::Edge(c, norm(graph.at(n)->pos-graph.at(c)->pos )));
            }
        }

        graph.at(n)->setEdges(tmp_node_edges);
        astar_connections.insert(std::make_pair<int,std::vector<int> >(n, tmp_edges));
    }

    astar_handler = new AstarHandler(astar_rep, astar_connections);
}

void TopGraph::updateContextCosts(std::vector<float>* costs_weights)
{
    astar_handler->setContextMultipliers(costs_weights);
}

void TopGraph::updateContextAreas(std::vector<Area>* _areas)
{
    areas.clear();
    for(unsigned int a=0; a<_areas->size(); ++a)
        areas.push_back(Area(_areas->at(a).id, _areas->at(a).label,
                             _areas->at(a).centroid, _areas->at(a).weight, _areas->at(a).params));


    std::vector<float> additional_costs(graph.size(), 0.f);
    for(unsigned int a=0; a<areas.size(); ++a)
    {
        for(unsigned int n=0; n<graph.size(); ++n)
        {
            if(areas.at(a).label == "rect")
            {
                // for rects weight are just equals for all the nodes inside the rect
                if( std::fabs(graph.at(n)->pos.x - areas.at(a).centroid.x) < areas.at(a).params.at(0)/2 &&
                        std::fabs(graph.at(n)->pos.y - areas.at(a).centroid.y) < areas.at(a).params.at(1)/2 )
                    additional_costs.at(n) += areas.at(a).weight;
            }

            if(areas.at(a).label == "circle")
            {
                // for circle weights are inversely proportional to the distance from the centre
                float normalized_distance = norm(graph.at(n)->pos - areas.at(a).centroid) / areas.at(a).params.at(0);
                if(normalized_distance < 1) {
                    additional_costs.at(n) += (areas.at(a).weight * normalized_distance);
                }
            }
        }
    }

    updateContextCosts(&additional_costs);
}

void TopGraph::computePath(cv::Point3f start_node, cv::Point3f end_node, std::vector<cv::Point3f>* _path)
{
    std::vector<int> pathId;
    astar_handler->findPath(
                astar_handler->getClosetNodeId(cv::Point2f(start_node.x, start_node.y)),
                astar_handler->getClosetNodeId(cv::Point2f(end_node.x, end_node.y)),
                &pathId );

    for(unsigned int n=0; n<pathId.size(); ++n)
        _path->push_back(cv::Point3f(graph.at(pathId.at(n))->pos.x, graph.at(pathId.at(n))->pos.y, 0.f)); //TODO next node orientation

    return;
}

#ifdef _TOP_GRAPH_MAIN_

int main(int argc, char** argv)
{
    cv::Mat src;
    std::string out_source = " ";

    if(argv[1])
    {
        std::string file = argv[2];
        std::ifstream f(file.c_str());

        if(!f.good())
        {
            f.close();
            std::cerr << "File " << file << " does not exist!" << std::endl;
            exit(1);
        }
        f.close();

        src = cv::imread(file, 0);
    }

    if(argv[3]) out_source = argv[4];



    TopGraph tg(src, out_source);

    std::vector<std::vector<float> > test_costs;
    for(unsigned int i=0; i<tg.getNodeNum(); ++i)
    {
        std::vector<float> tmp_test_costs(tg.getNodeNum(), 0.f);
        for(unsigned int j=0; j<tmp_test_costs.size(); ++j)
        {
            if(j>=105 && j<=112) tmp_test_costs.at(j) = 1000;
            if(j>=133 && j<=142) tmp_test_costs.at(j) = 1000;
        }
        test_costs.push_back(tmp_test_costs);
    }
    tg.updateContextCosts(&test_costs);

    tg.generateFileLP();
    cv::Mat vis;
    tg.vis(vis);
    cv::waitKey(0);


    return 0;
}

#endif



