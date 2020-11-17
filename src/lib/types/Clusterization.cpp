#include "Clusterization.h"
#include "geometry_utils.h"

using namespace std;
using namespace cv;

namespace eod{

    ClusterizationType getClusterizationTypeFromName(string name){
        transform(name.begin(), name.end(), name.begin(),[](unsigned char c){ return tolower(c); });        
        if( name == "forel" )
            return FOREL_C;
        return UNK_C;
    }
    
    // ------------------
    // FOREL
    // ------------------
    
    ClusterForel::ClusterForel(double R_, double eps_){
        type = FOREL_C;
        R = R_;
        eps = eps_;
    }

    void ClusterForel::init(vector<ExtendedObjectInfo> elements_){
        elements_unclustered.clear();
        clusters.clear();
        elements.swap(elements_);        
        for( size_t i = 0 ; i < elements.size() ; i++){
            elements_unclustered.push_back(i);
        }
        srand(time(NULL));
    }

    int ClusterForel::run(){
        while(elements_unclustered.size() > 0){
            int point = get_random_object();

            vector<int> neigbors = generate_same_objects(point);
            double len2cent1 =0;
            double len2cent2 =0;
            int center_point = center_of_objects(neigbors, &len2cent1);
            while( center_point != point){
                point = center_point;
                neigbors = generate_same_objects(point);
                center_point = center_of_objects(neigbors, &len2cent2);
                if( (len2cent2 - len2cent1) < eps ) break;
                len2cent1 = len2cent2;
            }
            clusters.push_back(neigbors);
            del_from_unclustered(neigbors);

        }
        return clusters.size();
    }

    int ClusterForel::get_random_object(){
#ifdef PRINT_OUTPUT
        printf("get_random_object\n");
#endif
        int ind = rand() % elements_unclustered.size();
        return elements_unclustered.at(ind);
    }

    vector<int> ClusterForel::generate_same_objects(int center){
#ifdef PRINT_OUTPUT
        printf("generate_same_objects\n");
#endif
        vector<int> res;
        for( size_t i = 0 ; i < elements_unclustered.size() ; i++){
            //double r = getRange(elements.at(center), elements.at(elements_unclustered.at(i)));
            double r = rect_distance(elements.at(center).getRect(), elements.at(elements_unclustered.at(i)).getRect());
            if( R >= r ){
                res.push_back(elements_unclustered.at(i) );
            }
        }
        return res;
    }

    int ClusterForel::center_of_objects(vector<int> el, double* len){
#ifdef PRINT_OUTPUT
        printf("center_of_objects\n");
#endif
        if(el.size() <= 0) return -1;

        double massCenter_X, massCenter_Y;
        massCenter_X = massCenter_Y = 0;
        for(size_t i = 0 ; i < el.size(); i++){
            massCenter_X += elements.at(el[i]).x;
            massCenter_Y += elements.at(el[i]).y;
        }
        massCenter_X /= el.size();
        massCenter_Y /= el.size();


        *len = 1.5 * R;
        int center = el.at(0);
        ExtendedObjectInfo fakeObj(massCenter_X,massCenter_Y,0,0);
        for(size_t i = 0 ; i < el.size(); i++){
            //double r = getRange(fakeObj,elements.at(el[i]));
            double r = rect_distance(fakeObj.getRect(),elements.at(el[i]).getRect());
            if( r < *len ){
                *len = r;
                center = el.at(i);
            }
        }
        return center;
    }

    void ClusterForel::del_from_unclustered(vector<int> el){
#ifdef PRINT_OUTPUT
        printf("del_from_unclustered\n");
#endif
        vector<int> newUnclustered;
        for(size_t i = 0 ; i < elements_unclustered.size() ; i++){
            if( std::find(el.begin(), el.end(), elements_unclustered.at(i)) == el.end() )
                newUnclustered.push_back(elements_unclustered.at(i));
        }
        elements_unclustered.swap(newUnclustered);
    }

    vector<ExtendedObjectInfo> ClusterForel::get_clustrs(){
        vector<ExtendedObjectInfo> result;
        for( size_t i = 0 ; i < clusters.size() ; i++){
            ExtendedObjectInfo tmp;
            for( size_t j = 0; j < clusters.at(i).size() ; j++ ){
                if(j==0) tmp = elements.at(clusters.at(i).at(j));
                else tmp = tmp | elements.at(clusters.at(i).at(j));
            }
            result.push_back(tmp);
        }
        return result;
    }
    
    vector<ExtendedObjectInfo> ClusterForel::cluster(vector<ExtendedObjectInfo>& input){        
        init(input);
        run();
        return get_clustrs();
    }

}
