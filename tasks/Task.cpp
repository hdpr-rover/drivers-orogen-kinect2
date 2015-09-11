/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <base/Logging.hpp>
#include <GL/freeglut.h>
#include <GLFW/glfw3.h>
#include <opencv2/opencv.hpp>

using namespace kinect2;
using namespace libfreenect2;

Task::Task(std::string const& name)
    : TaskBase(name)
{
 glfwInit();
 driver = new Freenect2();
 color_frame = 0;
 depth_frame = 0;
 ir_frame = 0;
 pipeline = new CpuPacketPipeline();
 visualizeDepth_frame = 0;
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
 glfwInit();
 driver = new Freenect2();
 pipeline = new CpuPacketPipeline();
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    /*
    int argc=0;
    char **argv=0;
    glutInit(&argc,argv);   
    glutInitDisplayMode(GLUT_RGBA);
    glutCreateWindow("test");
    */

    int device_count = driver->enumerateDevices();
    if(device_count < 1){
        LOG_FATAL_S << "Could not open Kinect"; 
        return false; 
    }
    std::string serial = driver->getDefaultDeviceSerialNumber();
//    device = driver->openDefaultDevice();
    device = driver->openDevice(serial,pipeline); //TODO handle more than 1 device
    if(!device){
        LOG_FATAL_S << "Could not open Kinect"; 
        return false;
    }else{
        LOG_DEBUG_S << "Device opened successfull";
    }
    device->setIrAndDepthFrameListener(this);
    device->setColorFrameListener(this);
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    device->start();
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
    device->stop();
    device->close();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
    delete device;
}
        
bool Task::onNewFrame(Frame::Type type, Frame *frame){
    size_t &width = frame->width;
    size_t &height = frame->height;
    size_t bpp = frame->bytes_per_pixel;
    unsigned char* data = frame->data;

    if(type == Frame::Color){
        //The kinect guy desicded to use BGRX (with alpha channel and alpha always zero so we handle alpha as non-existend in the following)
        if(bpp == 4 || bpp == 3){
            size_t bpp_internal = 3;

            if(!color_frame){
                color_frame = new base::samples::frame::Frame(width,height,8,base::samples::frame::MODE_RGB);
            }

            color_frame->time = base::Time::now();
            char data_new[width*height*bpp_internal];
            for(size_t x=0;x<width;x++){
                for(size_t y=0;y<height;y++){
                    data_new[0 + x*bpp_internal + y*bpp_internal*width] =  data[2 + (width-x)*bpp + y*bpp*width];
                    data_new[1 + x*bpp_internal + y*bpp_internal*width] =  data[1 + (width-x)*bpp + y*bpp*width];
                    data_new[2 + x*bpp_internal + y*bpp_internal*width] =  data[0 + (width-x)*bpp + y*bpp*width];
                }
            }
            color_frame->setImage(data_new,width*height*bpp_internal);
            color_frame_p.reset(color_frame);
            _color_frame.write(color_frame_p);
        }else{
            LOG_ERROR_S << "Color Image: Unssuported bits per pixel size of " << bpp;
            delete frame;
            return false;
        }
    }else if(type == Frame::Ir){
        if(bpp == 4){
            size_t bpp_grayscale = 2;
            if(!ir_frame){
                ir_frame = new base::samples::frame::Frame(width,height,16,base::samples::frame::MODE_GRAYSCALE);
            }
            char data_new[width*height*bpp];
            float *dataFloat = (float*)data;
	    uint16_t data16[width*height];
	    char dataChar[width*height*bpp_grayscale];
            
	    for(size_t x=0;x<width;x++){
                for(size_t y=0;y<height;y++){
                    data16[x + y*width] = (uint16_t)(dataFloat[x + y*width]);
                       // To mirror image : (width-x)*bpp_grayscale
                    dataChar[0 + x*bpp_grayscale + y*width*bpp_grayscale] = (char)(data16[x + y*width] & 0x00ff);
                    dataChar[1 + x*bpp_grayscale + y*width*bpp_grayscale] = (char)((data16[x + y*width] & 0xff00) >> 8);
                 }
             }

            ir_frame->setImage(dataChar,width*height*bpp_grayscale);
            ir_frame->time = base::Time::now();
            ir_frame_p.reset(ir_frame);
            _ir_frame.write(ir_frame_p);
        }else{
            LOG_ERROR_S << "IR Image: Unsupported bits per pixel size of " << bpp;
            delete frame;
            return false;
        }
    }else if(type == Frame::Depth){
        if(bpp == 4){
            if(!depth_frame){
                depth_frame = new base::samples::DistanceImage(width,height); 
                depth_frame->data.resize(width*height);
                Freenect2Device::IrCameraParams p = device->getIrCameraParams();
                depth_frame->setIntrinsic(p.fx,p.fy,p.cx,p.cy);
            }
             // added
           if(!visualizeDepth_frame){
                visualizeDepth_frame = new base::samples::frame::Frame(width, height, 16, base::samples::frame::MODE_GRAYSCALE);
           }
               // end

             	depth_frame->time = base::Time::now();
            	 float *depth_data = (float*)data;
 
               // added
           	float *dataFloat = (float*)data;
           	size_t bpp_grayscale = 2;
           	uint16_t data16[width*height];
           	char dataChar[width*height*bpp_grayscale];
               // end

	    for(size_t x=0;x<width;x++){
                 for(size_t y=0;y<height;y++){
                    depth_frame->data[x+y*width] = depth_data[(width-x)+y*width]*10.0; // randomly scaled for visualization
                        // added
                    data16[x + y*width] = (uint16_t)(dataFloat[x + y*width]);
                        // To mirror image : (width-x)*bpp_grayscale
                    dataChar[0 + x*bpp_grayscale + y*width*bpp_grayscale] = (char)(data16[x + y*width] & 0x00ff);
                    dataChar[1 + x*bpp_grayscale + y*width*bpp_grayscale] = (char)((data16[x + y*width] & 0xff00) >> 8);
                        // end
                 }
             }


            depth_frame_p.reset(depth_frame);
            _depth_frame.write(depth_frame_p);

               // added
            visualizeDepth_frame->setImage(dataChar,width*height*bpp_grayscale);
            visualizeDepth_frame->time = base::Time::now();
            visualizeDepth_frame_p.reset(visualizeDepth_frame);
            _visualizeDepth_frame.write(visualizeDepth_frame_p);
               // end

        }else{
            LOG_ERROR_S << "Depth Image: Unsupported bits per pixel size of " << bpp;
            delete frame;
            return false;
        }
    }else{
        LOG_ERROR_S << "Unsupported Frame format";
        delete frame;
        return false;
    }
    delete frame;
    return true;
}
