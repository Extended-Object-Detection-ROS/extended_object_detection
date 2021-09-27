#ifndef COMPLEX_OBJECT_GRAPH_H_
#define COMPLEX_OBJECT_GRAPH_H_

#include "SimpleObject.h"
#include "Relationship.h"
#include "igraph.h"

namespace eod{
    
    class Graph{
        
    public:
        Graph();
        
        int add_vectice(std::string object_name, int object_type, int obj_num = 0);
        int add_edge(std::string relation_name, int relation_type, int o1, int o2);
                
        igraph_vector_int_t get_vertices_colors();
        igraph_vector_int_t get_edges_colors();
        
    private:
        // graph representation
        igraph_t graph;
        
        // colors, needed for VF2                
        std::vector<int> vertices_colors;
        std::vector<int> edges_colors;
        
        int vertices_len;
        int edges_len;
                
    };
    
    class ComplexObjectGraph{
    public:
        ComplexObjectGraph();
                
        std::string name;
        int ID;                
        
        Graph graph;
        
        void add_object(std::string name, SimpleObject* so);
        void add_relation(std::string o1_name, std::string o2_name, RelationShip* rs);
        
        std::vector<ExtendedObjectInfo> Identify(const cv::Mat& frame, const cv::Mat& depth, int seq);
                        
    private:
        std::map<std::string, int> ObjectsToGraphsVerticesIds;
        std::map<std::string, SimpleObject*> ObjectsToSimpleObjects;
        
    };
    
}

#endif //COMPLEX_OBJECT_GRAPH_H_
