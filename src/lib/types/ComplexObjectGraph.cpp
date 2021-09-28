#ifdef USE_IGRAPH
#include "ComplexObjectGraph.h"
#include "drawing_utils.h"

namespace eod{
    
    Graph::Graph(){
        vertices_len = 0;
        edges_len = 0;
        igraph_empty(&graph, 0, IGRAPH_UNDIRECTED);
    }        
    
    int Graph::add_vectice(std::string object_name, int object_type, int obj_num){
        // check this already 
        if( vertices_len > 0 ){            
            igraph_vector_t types;
            igraph_vector_t nums;            
            igraph_vector_init (&types, 0);
            igraph_vector_init (&nums, 0);            
            VANV(&graph, "obj_type", &types);
            VANV(&graph, "obj_num", &nums);
                        
            //long int ind_type, ind_num;
            // TODO if check that it exists will be faster
            for( int i = 0 ; i < vertices_len ; i++){
                if( object_type == VECTOR(types)[i] && obj_num == VECTOR(nums)[i] && object_name == VAS(&graph, "obj_name", i) ){
                    printf("Vertice %i already added\n", i);
                    return i; //NOTE true?
                }
            }            
        }
                
        igraph_add_vertices( &graph, 1, NULL ); 
        SETVAS(&graph, "obj_name", vertices_len, object_name.c_str());
        SETVAN(&graph, "obj_type", vertices_len, object_type);
        SETVAN(&graph, "obj_num", vertices_len, obj_num);
        vertices_colors.push_back(object_type);
        
        printf("Added %i vertice\n", vertices_len);
        vertices_len++;        
        return vertices_len-1;
    }
    
    int Graph::add_edge(std::string relation_name, int relation_type, int o1, int o2){
        printf("Trying to added edge between vertices %i and %i\n", o1, o2);
        
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
        printf("Edges len %i\n",edges_len);
        return ec;
    }
    
    std::vector<std::vector<int>> Graph::get_subisomorphisms(Graph * sub_graph){
        
        igraph_vector_ptr_t maps;
        igraph_vector_ptr_init(&maps, 0);
        
        igraph_vector_int_t vert1 = this->get_vertices_colors();
        igraph_vector_int_t vert2 = sub_graph->get_vertices_colors();
        igraph_vector_int_t edg1 = this->get_edges_colors();
        igraph_vector_int_t edg2 = sub_graph->get_edges_colors();

        igraph_get_subisomorphisms_vf2(&graph, &(sub_graph->graph), &vert1, &vert2, &edg1, &edg2, &maps, 0, 0, 0);
        
        std::vector<std::vector<int>> vect_maps;
        
        int n_subis = igraph_vector_ptr_size(&maps);
        for(size_t i = 0; i < n_subis; i++ ){
            igraph_vector_t *temp = (igraph_vector_t*) VECTOR(maps)[i];
            std::vector<int> map;
            
            int n_map = igraph_vector_size(temp);
            for(size_t j = 0; j < n_map; j++){
                // j of graph2 --> VECTOR(*temp)[j] of graph1;
                map.push_back(VECTOR(*temp)[j]);
            }
            vect_maps.push_back(map);      
            igraph_vector_destroy(temp);
            igraph_free(temp);
        }
        igraph_vector_ptr_destroy(&maps);
        return vect_maps;                
    }
    
    std::string Graph::get_vertice_params(int id, int* object_type, int* obj_num){
        *object_type = VAN(&graph, "obj_type", id);
        *obj_num = VAN(&graph, "obj_num", id);
        return std::string(VAS(&graph, "obj_name", id));        
    }
            
    //----------------------
    // COMPLEX
    //----------------------
    
    ComplexObjectGraph::ComplexObjectGraph(){
        graph = Graph();
    }
    
    void ComplexObjectGraph::add_object(std::string name, SimpleObject* so, int num){
        int vid = graph.add_vectice(name, so->ID, num);
        ObjectsToGraphsVerticesIds.insert(std::pair<std::string, int>(name, vid));
        ObjectsToSimpleObjects.insert(std::pair<std::string, SimpleObject*>(name, so));
    }
    
    void ComplexObjectGraph::add_relation(std::string o1_name, std::string o2_name, RelationShip* rs){
        std::string auto_name = std::to_string(graph.get_edges_len());
        
        NamesToRelations.insert(std::pair<std::string, RelationShip*>(auto_name, rs));
        
        NamesToObjects.insert(std::pair<std::string, std::pair<std::string, std::string>>(auto_name,std::pair<std::string, std::string>(o1_name, o2_name)));        
        
        graph.add_edge(auto_name, rs->ID, ObjectsToGraphsVerticesIds[o1_name], ObjectsToGraphsVerticesIds[o2_name] );
    }
    
    std::vector<ExtendedObjectInfo> ComplexObjectGraph::Identify(const cv::Mat& frame, const cv::Mat& depth, int seq ){
        printf("NEW GRAPH\n");
        std::vector <ExtendedObjectInfo> result;
        
        // FORM A GRAPH
        Graph current_view_graph;
        for(auto const& nto : NamesToObjects){
            printf("Rel: %s %s\n", nto.first.c_str(), NamesToRelations[nto.first]->Name.c_str());
            std::vector<ExtendedObjectInfo> obj1 = ObjectsToSimpleObjects[nto.second.first]->Identify(frame, depth, seq);
            std::vector<ExtendedObjectInfo> obj2 = ObjectsToSimpleObjects[nto.second.second]->Identify(frame, depth, seq);
            
            for( size_t i = 0 ; i < obj1.size() ; i++ ){
                int ind1 = current_view_graph.add_vectice(nto.second.first, ObjectsToSimpleObjects[nto.second.first]->ID, i);
                printf("%i %i -> %i\n",ObjectsToSimpleObjects[nto.second.first]->ID, i, ind1);
                
                for( size_t j = 0 ; j < obj2.size(); j++){
                    
                    int ind2 = current_view_graph.add_vectice(nto.second.second, ObjectsToSimpleObjects[nto.second.second]->ID, j);
                    printf("%i %i -> %i\n",ObjectsToSimpleObjects[nto.second.second]->ID, j, ind2);
                    
                    if( NamesToRelations[nto.first]->checkRelation(frame, &obj1[i], &obj2[j]) ){
                        current_view_graph.add_edge(NamesToRelations[nto.first]->Name, NamesToRelations[nto.first]->ID, ind1, ind2);
                    }
                }
            }                        
        }
        
        // DO VF2
        std::vector<std::vector<int>> maps = current_view_graph.get_subisomorphisms(&graph);                
        
        // maps:
        // j - vert id graph, maps[_][j] - vert id current_view_graph
        
        // RETRIEVE DATA
        
        for( size_t i = 0 ; i < maps.size() ; i++ ){
            printf("Merged\n");
            int obj_type, obj_num;
            std::string object_name = current_view_graph.get_vertice_params(maps[i][0], &obj_type, &obj_num);
            
            ExtendedObjectInfo merged = ObjectsToSimpleObjects[object_name]->objects[obj_num];
            printf("\t%s %i\n", object_name.c_str(), obj_num);
            for( int j = 1 ; j < maps[i].size(); j++){
                object_name = current_view_graph.get_vertice_params(maps[i][j], &obj_type, &obj_num);
                merged = merged | ObjectsToSimpleObjects[object_name]->objects[obj_num];
                printf("\t%s %i\n", object_name.c_str(), obj_num);
            }
            result.push_back(merged);
        }
        
        complex_objects = result;
        return result;
    }
    
    void ComplexObjectGraph::drawOne(const cv::Mat& frameTd, int no, cv::Scalar color, int tickness){
        if( no < complex_objects.size() ){
            complex_objects[no].draw(frameTd, color);
            
            std::string objectInfo = std::to_string(ID)+": "+name;
            cv::Point prevBr = drawFilledRectangleWithText(frameTd, cv::Point(complex_objects[no].x,complex_objects[no].y -12)  , objectInfo, color);     
        }
    }
    
    void ComplexObjectGraph::drawAll(const cv::Mat& frameTd, cv::Scalar color, int tickness){
        for( size_t j = 0 ; j < complex_objects.size(); j++ ){
            drawOne(frameTd,j,color,tickness);
        }
    }
}

#endif //USE_IGRAPH
