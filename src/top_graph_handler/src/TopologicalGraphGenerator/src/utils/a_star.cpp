#include "a_star.h"

void AstarHandler::clear()
{
    for(unsigned int i=0; i<graph.size(); ++i)
        delete graph.at(i);

    std::map<int,Node*>::iterator eraser = history.begin();
    for(; eraser!=history.end(); ++eraser)
        delete eraser->second;

    graph.clear();
    connections.clear();
    closed_set.clear();
    open_set.clear();
    history.clear();
}

void AstarHandler::reset()
{
    std::map<int,Node*>::iterator eraser = history.begin();
    for(; eraser!=history.end(); ++eraser)
        delete eraser->second;

    closed_set.clear();
    open_set.clear();
    history.clear();
}

int AstarHandler::getClosetNodeId(cv::Point2f p)
{
    int _i = graph.size()-1;
    for(unsigned int n=0; n<graph.size(); ++n)
    {
        if( norm(Utils::co2cv_point(graph.at(n)->pos) - p) < norm(Utils::co2cv_point(graph.at(_i)->pos) - p) )
            _i = n;
    }

    return _i;
}

void AstarHandler::removeFromSet(int id, std::set<Node,Node::cmp>* _set)
{
    std::set<Node,Node::cmp>::iterator remover = _set->begin();
    for(;remover != _set->end(); ++remover)
    {
        if( remover->id == id )
        {
            _set->erase(remover);
            break;
        }
    }
}

bool AstarHandler::isInSet(int id, std::set<Node,Node::cmp>* _set)
{
    std::set<Node,Node::cmp>::iterator remover = _set->begin();
    for(;remover != _set->end(); ++remover)
    {
        if( remover->id == id ) return true;
    }
    return false;
}

float AstarHandler::heuristicCostEstimate(Node* ni, Node* ne)
{

#ifdef EUCLEDIAN_DISTANCE
    // eucledian distance
    return Utils::norm2D(ni->pos, ne->pos);
#else
    return Utils::norm2D(ni->pos, ne->pos) + context_costs.at(ni->id);
#endif
}

int AstarHandler::getLowestFscoreNode()
{
    float min_fscore = FLT_MAX;
    int min_i = -1;
    std::set<Node,Node::cmp>::iterator analyser = open_set.begin();
    for(; analyser!=open_set.end(); ++analyser)
    {
        if((*analyser).f_score<min_fscore)
        {
            min_fscore = (*analyser).f_score;
            min_i = (*analyser).id;
        }
    }

    return min_i;
}

void AstarHandler::retracePath(int c_id, int s_id, std::vector<int>* _path)
{
    // init
    int c_node = c_id;
    _path->push_back(c_id);

    // retrace
    while(c_node != s_id)
    {
        _path->push_back(history[c_node]->id);
        c_node = history[c_node]->id;
    }

     std::reverse(_path->begin(), _path->end());
}

void AstarHandler::setContextCosts(std::vector<float> *_ncc)
{
    for(unsigned int i=0; i<_ncc->size(); ++i)
            context_costs.at(i) = _ncc->at(i);
}

bool AstarHandler::findPath(int s_id, int e_id, std::vector<int>* _path)
{
    // init
    reset();
    open_set.insert(*graph.at(s_id));

    graph.at(s_id)->g_score = 0;
    graph.at(s_id)->f_score = graph.at(s_id)->g_score + heuristicCostEstimate(graph.at(s_id), graph.at(e_id));

    while(!open_set.empty())
    {

        Node current( *graph.at(getLowestFscoreNode()) );

        // if the path is complete
        if(current.id == e_id)
        {
            retracePath(current.id, s_id, _path);
            return true;
        }
        else
        {
            // erase current node from the open set
            removeFromSet(current.id, &open_set);

            // add current node to the closed set
            closed_set.insert(current);

            // evaluate neigheborood of the current node
            for(unsigned int n=0; n<connections[current.id].size(); ++n)
            {
                // if the neighbors of the current node has been closed already
                if(isInSet(connections[current.id].at(n), &closed_set)) continue;

                float tmp_g_score = current.g_score +
                        Utils::norm2D(current.pos, graph.at(connections[current.id].at(n))->pos);

                // expansion step
                if( !isInSet(connections[current.id].at(n), &open_set) ||
                        tmp_g_score < graph.at(connections[current.id].at(n))->g_score)
                {
                    std::map<int, Node*>::iterator remover = history.find(connections[current.id].at(n));
                    if(remover != history.end()) history.erase(remover);

                    history.insert( std::make_pair<int,Node*>(connections[current.id].at(n), new Node(current.id,current.pos)) );
                    graph.at(connections[current.id].at(n))->g_score = tmp_g_score;
                    graph.at(connections[current.id].at(n))->f_score = tmp_g_score +
                            heuristicCostEstimate(graph.at(connections[current.id].at(n)), graph.at(e_id));

                    // if neighbor is not in the open set then add it
                    if( !isInSet(connections[current.id].at(n), &open_set))
                        open_set.insert(*graph.at(connections[current.id].at(n)));
                }
            }
        }
    }
    // return failure
    return false;
}
