# ifndef PARALLEL_NODE_H
# define PARALLEL_NODE_H

# include <thread>
# include <vector>

# include "TimeLogger.h"
# include "DataExchange.h"
# include "CriticalBuffer.h"
# include "CameraProperties.h"



typedef void (*CALLBACK_FUNCTION_POINTER)(void* input_data, void* user_data);


struct CallBack
{
    CALLBACK_FUNCTION_POINTER   function_ptr;
    void*                       data_ptr;
    
    CallBack() : function_ptr(0), data_ptr(0) {}

    bool Invoke(void* input)
    {
        if(!function_ptr) return false;

        function_ptr(input, data_ptr);
        return true;
    }
};


class ParallelNode
{

protected: 

    cv::Mat m_matCurrentData;

    PAL::CameraProperties m_oCameraProperties;

    unsigned int m_oCamFlag;

    std::string m_strName;

    ///All the ParallelNode objects, that should be finished, before this ParallelNode starts execution
    std::vector<ParallelNode*> m_vpInputNodes;

    ///All the critical outputs, generated by this ParallelNode
    std::vector<CriticalBuffer> m_vCriticalBuffers; 

    ///flag to indicate if this ParallelNode should exit (value is true when the app is closed)
    bool m_bExited;

    ///flag to indicate if this ParallelNode is running or not (value is true when the pipeline's mode is not using this node)
    bool m_bPaused;

    ///flag to indicate if the camera properties are changed (like resolution/fov etc)
    bool m_bStateChange;

    int m_iNodeFlags;

    DataLock m_oPauseLock;

    DataLock m_oFlushLock;

    MinMax m_oIterationLog;

    std::thread m_oThreadID;

    //In case if user has asked for any notifications - whenever the input is ready
    CallBack m_cbInputReady;
    
    //In case if user has asked for any notifications - whenever the ouput is ready
    CallBack m_cbOutputReady;

public:
    
    ParallelNode(); 

    virtual void AddInputNode(ParallelNode* node);

    virtual void Start();

    virtual void Run();

    virtual void Stop();
    
    virtual void* Iterate(void* input); 

    virtual void* Consume();

    virtual void Pause();

    virtual void Resume(void* arg);

    virtual void Init(void* arg, std::string name);
    
    virtual void Destroy(); 
    
    virtual PAL::Acknowledgement ChangeInternalState(PAL::CameraProperties *prop, unsigned int* flags);

    virtual void Detach();

    virtual void Reset() {}

    virtual ~ParallelNode() { Destroy(); }

    //This callback will be invoked whenever the input data is ready
    virtual void SetInputCallback(CALLBACK_FUNCTION_POINTER pFunc, void* pData);

    //This callback will be invoked whenever the output data is ready
    virtual void SetOutputCallback(CALLBACK_FUNCTION_POINTER pFunc, void* pData);

    virtual void CreateCriticalBuffers(void* arg);

    virtual void NotifyOutputNodes(void* output);

    virtual void* WaitForInputNodes();

    virtual void InvalidateCriticalBuffers();

    virtual void Flush();

    virtual PAL::Acknowledgement ChangeCameraProperties(PAL::CameraProperties* prop, unsigned int* flags);
    
};

# endif //PARALLEL_NODE_H
