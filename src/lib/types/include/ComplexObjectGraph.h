#ifndef COMPLEX_OBJECT_GRAPH_H_
#define COMPLEX_OBJECT_GRAPH_H_

#include "SimpleObject.h"
#include "Relationship.h"
#include "igraph.h"

namespace eod{
    
    class Graph{
        
    public:
        Graph();
        
        int add_vectice(std::string object_name, int object_type, int obj_num = 0, double dc = 1, double weight = 1);
        int add_edge(std::string relation_name, int relation_type, int o1, int o2, bool fake = false, double weight = 1);                        
        
        std::vector<std::pair<std::vector<int>, double>> get_subisomorphisms(Graph * sub_graph);
        
        std::string get_vertice_params(int id, int* object_type, int* obj_num);
        
        inline int get_edges_len(){
            return edges_len;
        }
        
    private:
        // graph representation
        igraph_t graph;
        
        // colors, needed for VF2                
        std::vector<int> vertices_colors;
        std::vector<int> edges_colors;
        
        igraph_vector_int_t get_vertices_colors();
        igraph_vector_int_t get_edges_colors();
        
        int vertices_len;
        int edges_len;
        
        int accuracy;                
                
    };
    
    class ComplexObjectGraph{
    public:
        ComplexObjectGraph();
                
        std::string name;
        int ID;                
        IdentifyMode identify_mode;  
        
        Graph graph;
        double degree_of_confidence;
        
        double Probability;
        int plot_offset;
        
        void add_object(std::string name, SimpleObject* so, int num = 0, double weight = 1);
        void add_relation(std::string o1_name, std::string o2_name, RelationShip* rs, double weight = 1);
        
        std::vector<ExtendedObjectInfo> Identify(const cv::Mat& frame, const cv::Mat& depth, int seq);
        
        std::vector<ExtendedObjectInfo> IdentifyHard(const cv::Mat& frame, const cv::Mat& depth, int seq);
        std::vector<ExtendedObjectInfo> IdentifySoft(const cv::Mat& frame, const cv::Mat& depth, int seq);
        
        void drawOne(const cv::Mat& frameTD, int no, cv::Scalar color, int tickness);
        void drawAll(const cv::Mat& frameTD, cv::Scalar color, int tickness);
        
        std::vector<ExtendedObjectInfo> complex_objects;
                        
    private:
        std::map<std::string, int> ObjectsToGraphsVerticesIds;
        std::map<std::string, SimpleObject*> ObjectsToSimpleObjects;
        
        std::map<std::string, RelationShip*> NamesToRelations;
        std::map<std::string, std::pair<std::string, std::string>> NamesToObjects;
        
        
    };
    
}

#endif //COMPLEX_OBJECT_GRAPH_H_
