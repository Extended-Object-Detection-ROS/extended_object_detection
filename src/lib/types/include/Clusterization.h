#ifndef _CLUSTERIZATION_H_
#define _CLUSTERIZATION_H_

#include "ExtendedObjectInfo.h"

namespace eod{
    
    enum ClusterizationType{
        UNK_C,
        FOREL_C,
    };
        
    ClusterizationType getClusterizationTypeFromName(std::string name);
    
    // clusterization interface
    class ClusterizationMethod{
    public:
        ClusterizationMethod(){};
        
        virtual std::vector<ExtendedObjectInfo> cluster(std::vector<ExtendedObjectInfo>& input) = 0;
        
        ClusterizationType type;
    private:
        
    };
    
    // FOREL
    class ClusterForel : public ClusterizationMethod{
    public:
        ClusterForel(double R_, double eps_);
        
        void init(std::vector<ExtendedObjectInfo> elements_);
        
        std::vector<ExtendedObjectInfo> cluster(std::vector<ExtendedObjectInfo>& input);

    private:
        double R;
        double eps;
        std::vector<ExtendedObjectInfo> elements;
        std::vector<int> elements_unclustered;

        int get_random_object();
        std::vector<int> generate_same_objects(int center);
        int center_of_objects(std::vector<int>, double* len);
        void del_from_unclustered(std::vector<int> );
        
        int run();
        std::vector<ExtendedObjectInfo> get_clustrs();
        std::vector<std::vector<int> > clusters;

    };
}

#endif// _CLUSTERIZATION_H_
