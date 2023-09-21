#ifndef MYAPI_H
#define MYAPI_H

namespace ly{
    //ros系统内读取参数服务器参数
    template<typename T>
    T getParam(const std::string& name,const T& defaultValue)//This name must be namespace+parameter_name
    {
        T v;
        if(ros::param::get(name,v))//get parameter by name depend on ROS.
        {
            ROS_INFO_STREAM("Found parameter: "<<name<<",\tvalue: "<<v);
            return v;
        }
        else 
            ROS_WARN_STREAM("Cannot find value for parameter: "<<name<<",\tassigning default: "<<defaultValue);
        return defaultValue;//if the parameter haven't been set,it's value will return defaultValue.
    }
}


#endif