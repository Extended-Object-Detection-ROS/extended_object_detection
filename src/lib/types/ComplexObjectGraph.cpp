#ifdef USE_IGRAPH
#include "ComplexObjectGraph.h"

namespace eod{
    
    Graph::Graph(){
        vertices_len = 0;
        edges_len = 0;
        igraph_empty(&graph, 0, IGRAPH_UNDIRECTED);
    }
    
    int Graph::add_vectice(std::string object_name, int object_type, int obj_num){
                
        igraph_add_vertices( &graph, 1, NULL ); 
        SETVAS(&graph, "obj_name", vertices_len, object_name.c_str());
        SETVAN(&graph, "obj_type", vertices_len, object_type);
        SETVAN(&graph, "obj_num", vertices_len, obj_num);
        vertices_colors.push_back(object_type);
        
        vertices_len++;
        return vertices_len;
    }
    
    int Graph::add_edge(std::string relation_name, int relation_type, int o1, int o2){
        
        igraph_add_edge(&graph, o1, o2);
        SETEAS(&graph, "rel_name", edges_len, relation_name.c_str());
        SETEAN(&graph, "rel_type", edges_len, relation_type);
        
        edges_colors.push_back(relation_type);
        
        edges_len++;
        return edges_len;
    }
    
    igraph_vector_int_t Graph::get_vertices_colors(){
        igraph_vector_int_t vc;
        igraph_vector_int_init(&vc, vertices_len);
        
        for( int i = 0 ; i < vertices_len ; i++ )
            VECTOR(vc)[i] = vertices_colors[i];
        return vc;
    }
    
    igraph_vector_int_t Graph::get_edges_colors(){
        igraph_vector_int_t ec;
        igraph_vector_int_init(&ec, edges_len);
        
        for( int i = 0 ; i < edges_len ; i++ )
            VECTOR(ec)[i] = edges_colors[i];
        return ec;
    }
            
    //----------------------
    // COMPLEX
    //----------------------
    
    ComplexObjectGraph::ComplexObjectGraph(){
        graph = Graph();
    }
    
    void ComplexObjectGraph::add_object(std::string name, SimpleObject* so){
        int vid = graph.add_vectice(name, so->ID, 0);
        ObjectsToGraphsVerticesIds.insert(std::pair<std::string, int>(name, vid));
        ObjectsToSimpleObjects.insert(std::pair<std::string, SimpleObject*>(name, so));
    }
    
    void ComplexObjectGraph::add_relation(std::string o1_name, std::string o2_name, RelationShip* rs){
        graph.add_edge(rs->Name, rs->ID, ObjectsToGraphsVerticesIds[o1_name], ObjectsToGraphsVerticesIds[o2_name] );
    }
    
    std::vector<ExtendedObjectInfo> ComplexObjectGraph::Identify(const cv::Mat& frame, const cv::Mat& depth, int seq ){
        
        std::vector <ExtendedObjectInfo> result;
        
        for( auto const& so : ObjectsToSimpleObjects){
            so.second->Identify(frame, depth, seq);
        }
        
        return result;
    }
}

#endif //USE_IGRAPH
