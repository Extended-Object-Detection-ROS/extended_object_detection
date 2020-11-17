#ifdef USE_TF
#include "TensorFlowDetector.h"
#include <regex>

#include "tensorflow/core/framework/tensor.h"
#include "tensorflow/core/lib/core/errors.h"
#include "tensorflow/core/lib/core/stringpiece.h"
#include "tensorflow/core/platform/env.h"
#include "tensorflow/core/platform/logging.h"
#include "tensorflow/core/platform/types.h"

using namespace cv;
using namespace std;
using tensorflow::Flag;
using tensorflow::Tensor;
using tensorflow::Status;
using tensorflow::string;
using tensorflow::int32;

namespace eod{ 
    
    Status loadGraph(const string &graph_file_name, unique_ptr<tensorflow::Session> *session);
    Status readLabelsMapFile(const string &fileName, map<int, string> &labelsMap);
    Status readTensorFromMat(const Mat &mat, Tensor &outTensor);
    
    // -------------------------
    // Global TF Detector
    // -------------------------
    
    //GlobalTFDetector* GTFD = NULL;
    
    vector< GlobalTFDetector* > GTFDS;
    
    
    GlobalTFDetector::GlobalTFDetector(){
    }
    
    GlobalTFDetector::GlobalTFDetector(string graphPath_, string labelsPath_){
        
        graphPath = graphPath_;
        labelsPath = labelsPath_;
        
        inited = false;
        // load graph
        Status loadGraphStatus = loadGraph(graphPath_, &session);
        if (!loadGraphStatus.ok()) {                
            printf("Error reading graph model %s!\n",graphPath_.c_str());
            return;
        }
        // load labels
        Status readLabelsMapStatus = readLabelsMapFile(labelsPath_, labelsMap);
        if (!readLabelsMapStatus.ok()) {         
            printf("Error reading labels %s!\n",labelsPath_.c_str());
            return;                
        }
        
        // Set input & output nodes names
        inputLayer = "image_tensor:0";
        outputLayer = {"detection_boxes:0", "detection_scores:0", "detection_classes:0", "num_detections:0"};
        
        inited = true;        
    }
    
    vector<ExtendedObjectInfo> GlobalTFDetector::detect(const Mat& frame, int seq, int id_obj, double prob_score, double Weight){                        
        vector<ExtendedObjectInfo> ans;        
        if( !inited ) 
            return ans;
        if( seq == 0 || seq != prev_seq ){            
            //printf("Detect at new time %i %i \n",seq, prev_seq);
            if( seq != 0 )
                prev_seq = seq;
            // detect       
            
            tensorflow::TensorShape shape = tensorflow::TensorShape();
            shape.AddDim(1);
            shape.AddDim((int64)frame.size().height);
            shape.AddDim((int64)frame.size().width);
            shape.AddDim(3);
            cvtColor(frame, frame, COLOR_BGR2RGB); 
            // Convert mat to tensor
            tensor = Tensor(tensorflow::DT_FLOAT, shape);
            
            Status readTensorStatus = readTensorFromMat(frame, tensor);            
            if (!readTensorStatus.ok()) {                
                return ans;
            }          
            
            // Run the graph on tensor
            outputs.clear();
            Status runStatus = session->Run({{inputLayer, tensor}}, outputLayer, {}, &outputs);
            if (!runStatus.ok()) {
                return ans;
            }                        
            
        }    
        // Extract results from the outputs vector
        tensorflow::TTypes<float>::Flat scores = outputs[1].flat<float>();
        tensorflow::TTypes<float>::Flat classes = outputs[2].flat<float>();
        tensorflow::TTypes<float>::Flat numDetections = outputs[3].flat<float>();
        tensorflow::TTypes<float, 3>::Tensor boxes = outputs[0].flat_outer_dims<float,3>();
        
        // collect objects with id_obj
        int width = frame.size().width;
        int height = frame.size().height;
                
        for( size_t i = 0 ; i < scores.size() ; i++){        
            if( scores(i) > prob_score && classes(i) == id_obj ){
                ExtendedObjectInfo box = ExtendedObjectInfo(Rect(Point(width*boxes(0, i, 1), height*boxes(0, i, 0)),Point(width*boxes(0, i, 3), height*boxes(0, i, 2))));  
                box.setScoreWeight(scores(i), Weight);
                ans.push_back(box);
            }
        }        
        return ans;        
    }
    
    // -------------------------
    // -------------------------
    // TF attribute  
    // -------------------------
    // -------------------------
    TensorFlowAttribute::TensorFlowAttribute(){
        Type = TF_A;
        GTFD = NULL;
        inited = false;
    }
    
    TensorFlowAttribute::TensorFlowAttribute(string graphPath, string labelsPath, int id_obj){
        Type = TF_A;        
        inited = false;              
        GTFD = NULL;
        
        this->id_obj = id_obj;
        for( size_t i = 0 ; i < GTFDS.size() ; i++ ){
            if( graphPath == GTFDS[i]->graphPath && labelsPath == GTFDS[i]->labelsPath )
                GTFD = GTFDS[i];
        }
        if( !GTFD ){
            GTFD = new GlobalTFDetector(graphPath, labelsPath);            
            GTFDS.push_back(GTFD);
        }
    }
    
    vector<ExtendedObjectInfo> TensorFlowAttribute::Detect2(const Mat& image, int seq){                                                
        return GTFD->detect(image, seq, id_obj, Probability, Weight);
    }
    
    bool TensorFlowAttribute::Check2(const Mat& image, ExtendedObjectInfo& rect){
        return false;
    }
    
    void TensorFlowAttribute::Extract2(const cv::Mat& image, ExtendedObjectInfo& rect){
    }
    
    // -------------------------
    // -------------------------
    // tf support functions
    // -------------------------
    // -------------------------
  
    /** Read a model graph definition (xxx.pb) from disk, and creates a session object you can use to run it.
    */
    Status loadGraph(const string &graph_file_name, unique_ptr<tensorflow::Session> *session) {
        tensorflow::GraphDef graph_def;
        Status load_graph_status =
                ReadBinaryProto(tensorflow::Env::Default(), graph_file_name, &graph_def);
                
        if (!load_graph_status.ok()) {
            printf("Tensorflow failed to load compute graph.\n");
            return tensorflow::errors::NotFound("Failed to load compute graph at '",
                                                graph_file_name, "'");
        }        
        session->reset(tensorflow::NewSession(tensorflow::SessionOptions()));
        Status session_create_status = (*session)->Create(graph_def);
        if (!session_create_status.ok()) {
            printf("Tensorflow error create session.\n");
            return session_create_status;
        }
        return Status::OK();
    }

    /** Read a labels map file (xxx.pbtxt) from disk to translate class numbers into human-readable labels.
    */
    Status readLabelsMapFile(const string &fileName, map<int, string> &labelsMap) {

        // Read file into a string
        ifstream t(fileName);
        if (t.bad())
            return tensorflow::errors::NotFound("Failed to load labels map at '", fileName, "'");
        stringstream buffer;
        buffer << t.rdbuf();
        string fileString = buffer.str();

        // Search entry patterns of type 'item { ... }' and parse each of them
        smatch matcherEntry;
        smatch matcherId;
        smatch matcherName;
        const regex reEntry("item \\{([\\S\\s]*?)\\}");
        const regex reId(" [0-9]+");
        const regex reDispName("\"[a-z\\s]+\"");
        
        string entry;

        auto stringBegin = sregex_iterator(fileString.begin(), fileString.end(), reEntry);
        auto stringEnd = sregex_iterator();

        int id;
        string name;
        for (sregex_iterator i = stringBegin; i != stringEnd; i++) {
            matcherEntry = *i;
            entry = matcherEntry.str();
            regex_search(entry, matcherId, reId);
            if (!matcherId.empty())
                id = stoi(matcherId[0].str());
            else
                continue;
            regex_search(entry, matcherName, reDispName);
            if (!matcherName.empty())
                name = matcherName[0].str().substr(1, matcherName[0].str().length() - 2);
            else
                name = "unknown";
                //continue;
            labelsMap.insert(pair<int, string>(id, name));
        }
        return Status::OK();
    }

    /** Convert Mat image into tensor of shape (1, height, width, d) where last three dims are equal to the original dims.
    */
    Status readTensorFromMat(const Mat &mat, Tensor &outTensor) {

        auto root = tensorflow::Scope::NewRootScope();
        using namespace ::tensorflow::ops;

        // Trick from https://github.com/tensorflow/tensorflow/issues/8033
        float *p = outTensor.flat<float>().data();
        Mat fakeMat(mat.rows, mat.cols, CV_32FC3, p);
        mat.convertTo(fakeMat, CV_32FC3);

        auto input_tensor = Placeholder(root.WithOpName("input"), tensorflow::DT_FLOAT);
        vector<pair<string, tensorflow::Tensor>> inputs = {{"input", outTensor}};
        auto uint8Caster = Cast(root.WithOpName("uint8_Cast"), outTensor, tensorflow::DT_UINT8);

        // This runs the GraphDef network definition that we've just constructed, and
        // returns the results in the output outTensor.
        tensorflow::GraphDef graph;
        TF_RETURN_IF_ERROR(root.ToGraphDef(&graph));

        vector<Tensor> outTensors;
        unique_ptr<tensorflow::Session> session(tensorflow::NewSession(tensorflow::SessionOptions()));

        TF_RETURN_IF_ERROR(session->Create(graph));
        TF_RETURN_IF_ERROR(session->Run({inputs}, {"uint8_Cast"}, {}, &outTensors));

        outTensor = outTensors.at(0);
        return Status::OK();
    }
  
    
}
#endif // TF
