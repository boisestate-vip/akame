
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "SYDataDefine.h"
#include "SYObserverInterface.h"
#include "SYSDKInterface.h"

using std::placeholders::_1;

class SynexensNode : public rclcpp::Node {
private:

   class FrameObserver : public Synexens::ISYFrameObserver {

      private:

         SynexensNode * parent;

      public:

         FrameObserver(SynexensNode * parent) : parent(parent) {}


         virtual void OnFrameNotify(unsigned int deviceID, Synexens::SYFrameData * frameData = nullptr) {
            ProcessFrameData(deviceID, frameData);
         }

         void ProcessFrameData(unsigned int deviceID, Synexens::SYFrameData * frameData) {
            /* get device info */
            auto type = parent->streamTypes.find(deviceID);
            auto is_open = parent->streamOns.find(deviceID);

            Synexens::SYErrorCode errorCode;

            /* ensure stream is up and data is useable */
            if (type == parent->streamTypes.end() || 
                (is_open == parent->streamOns.end() || is_open->second != true)) {
                return;
            }

            if (type->second == Synexens::SYSTREAMTYPE_RGBD) {

                std::map<Synexens::SYFrameType, int> mapIndex;
                std::map<Synexens::SYFrameType, int> mapPos;

                int nPos = 0;
                for (int i = 0; i < frameData->m_nFrameCount; i++)
                {
                    mapIndex.insert(std::pair<Synexens::SYFrameType, int>
                          (frameData->m_pFrameInfo[i].m_frameType, i));
                    mapPos.insert(std::pair<Synexens::SYFrameType, int>
                          (frameData->m_pFrameInfo[i].m_frameType, nPos));
                    nPos += frameData->m_pFrameInfo[i].m_nFrameHeight * 
                    frameData->m_pFrameInfo[i].m_nFrameWidth * sizeof(short);
                }

                auto itDepthIndex = mapIndex.find(Synexens::SYFRAMETYPE_DEPTH);
                auto itRGBIndex = mapIndex.find(Synexens::SYFRAMETYPE_RGB);
                int nRGBDWidth = frameData->m_pFrameInfo[itRGBIndex->second].m_nFrameWidth;
                int nRGBDHeight = frameData->m_pFrameInfo[itRGBIndex->second].m_nFrameHeight;
                unsigned short* pRGBDDepth = new unsigned short[nRGBDWidth * nRGBDHeight];
                memset(pRGBDDepth, 0, sizeof(unsigned short) * nRGBDWidth * nRGBDHeight);
                unsigned char* pRGBDRGB = new unsigned char[nRGBDWidth * nRGBDHeight * 3];
                memset(pRGBDRGB, 0, sizeof(unsigned char) * nRGBDWidth * nRGBDHeight * 3);
                Synexens::SYErrorCode errorCode = Synexens::GetRGBD(deviceID, 
                    frameData->m_pFrameInfo[itDepthIndex->second].m_nFrameWidth, 
                    frameData->m_pFrameInfo[itDepthIndex->second].m_nFrameHeight,  
                    (unsigned short*)frameData->m_pData + mapPos[Synexens::SYFRAMETYPE_DEPTH],
                    frameData->m_pFrameInfo[itRGBIndex->second].m_nFrameWidth, 
                    frameData->m_pFrameInfo[itRGBIndex->second].m_nFrameHeight, 
                    (unsigned char*)frameData->m_pData + mapPos[Synexens::SYFRAMETYPE_RGB],
                    nRGBDWidth, 
                    nRGBDHeight,
                    pRGBDDepth, 
                    pRGBDRGB);

                if (errorCode == Synexens::SYERRORCODE_SUCCESS)
                {
                    // TODO: handle rgb
                }

                delete[] pRGBDDepth;
                delete[] pRGBDRGB;
            }
            else {
              int nPos = 0;
              for (int i = 0; i < frameData->m_nFrameCount; i++) 
              {
                  switch (frameData->m_pFrameInfo[i].m_frameType)
                  {
                     case Synexens::SYFRAMETYPE_RAW:
                     {
                         // RawData pointer starting position =  frameData->m_pData + nPos  
                         // RawData pointer end position = frameData->m_pData + frameData->m_pFrameInfo[i].m_nFrameHeight * frameData->m_pFrameInfo[i].m_nFrameWidth * sizeof(short)

                         // handle your own logic ...
                         // TODO

                         nPos += frameData->m_pFrameInfo[i].m_nFrameHeight * frameData->m_pFrameInfo[i].m_nFrameWidth * sizeof(short);
                         break;
                     }
                     case Synexens::SYFRAMETYPE_DEPTH:
                     {
                         // DepthData pointer starting position =  frameData->m_pData + nPos  
                         // DepthData pointer end position = frameData->m_pData + frameData->m_pFrameInfo[i].m_nFrameHeight * frameData->m_pFrameInfo[i].m_nFrameWidth * sizeof(short)
                         int nPoints = frameData->m_pFrameInfo[i].m_nFrameHeight * frameData->m_pFrameInfo[i].m_nFrameWidth;
                         int frameSize = nPoints;

                         Synexens::SYPointCloudData * points = new Synexens::SYPointCloudData[nPoints];

                         errorCode = Synexens::GetDepthPointCloud(deviceID,frameData->m_pFrameInfo[i].m_nFrameWidth,frameData->m_pFrameInfo[i].m_nFrameHeight,(const unsigned short *)(frameData->m_pData) + nPos,points);

                         /* collect our point cloud data, package it as a ros message, and send it off */
                         sensor_msgs::msg::PointCloud2 msg;
                         msg.header.frame_id = parent->get_parameter("frame_id").as_string();

                         /* fill x, y, z values */
                         sensor_msgs::msg::PointField field;

                         field.name = "x";
                         field.offset = 0;
                         field.datatype = sensor_msgs::msg::PointField::FLOAT32;
                         field.count = 1;
                         msg.fields.push_back(field);

                         field.name = "y";
                         field.offset = 4;
                         field.datatype = sensor_msgs::msg::PointField::FLOAT32;
                         field.count = 1;
                         msg.fields.push_back(field);

                         field.name = "z";
                         field.offset = 8;
                         field.datatype = sensor_msgs::msg::PointField::FLOAT32;
                         field.count = 1;
                         msg.fields.push_back(field);

                         msg.is_bigendian = false;
                         msg.point_step = sizeof(Synexens::SYPointCloudData);
                         msg.row_step = nPoints;

                         msg.height = frameData->m_pFrameInfo[i].m_nFrameHeight;
                         msg.width = frameData->m_pFrameInfo[i].m_nFrameWidth;

                         for (int i = 0; i < nPoints; ++i) {
                            points[i].m_fltX /= 1000.0;
                            points[i].m_fltY /= 1000.0;
                            points[i].m_fltZ /= 1000.0;
                         }

                         msg.data.reserve(nPoints*sizeof(Synexens::SYPointCloudData));
                         //msg.data.resize(nPoints*sizeof(Synexens::SYPointCloudData));
                         uint8_t * to_uint = (uint8_t *)points;
                         for (int i = 0; i < nPoints*sizeof(Synexens::SYPointCloudData); ++i) {
                            msg.data.push_back(to_uint[i]);
                         }
                         //memcpy(msg.data.data(),points,nPoints*sizeof(Synexens::SYPointCloudData));

                         /* ? */
                         msg.is_dense = true;

                         msg.header.stamp = parent->get_clock()->now();
                         parent->cloud_out->publish(msg);

                         nPos += frameSize;

                         delete[] points;
                         break;
                     }
                     case Synexens::SYFRAMETYPE_IR:
                     {
                         // IrData pointer starting position =  frameData->m_pData + nPos  
                         // IrData pointer end position = frameData->m_pData + frameData->m_pFrameInfo[i].m_nFrameHeight * frameData->m_pFrameInfo[i].m_nFrameWidth * sizeof(short)

                         // handle your own logic ...
                         // TODO

                         nPos += frameData->m_pFrameInfo[i].m_nFrameHeight * frameData->m_pFrameInfo[i].m_nFrameWidth * sizeof(short);
                         break;
                     }
                     case Synexens::SYFRAMETYPE_RGB:
                     {
                         // RGBData pointer starting position =  frameData->m_pData + nPos  
                         // RGBData pointer end position = frameData->m_pData + frameData->m_pFrameInfo[i].m_nFrameHeight * frameData->m_pFrameInfo[i].m_nFrameWidth * sizeof(short)

                         // handle your own logic ...
                         // TODO

                         nPos += frameData->m_pFrameInfo[i].m_nFrameHeight * frameData->m_pFrameInfo[i].m_nFrameWidth * sizeof(short);
                         break;
                     }
                     case Synexens::SYFRAMETYPE_NULL:
                     {
                         break;
                     }
                     case Synexens::SYFRAMETYPE_AI:
                     {
                         break;
                     }
                  }
               }
            }
         }
   };

protected:

   rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_out;

   Synexens::SYErrorCode errorCode;
   Synexens::SYStreamType streamType;
   Synexens::SYResolution resolutionType, color_resolutionType;

   std::map<int,Synexens::SYStreamType> streamTypes;
   std::map<int,bool> streamOns;

   int deviceCount;
   Synexens::SYDeviceInfo * deviceInfo;

   FrameObserver frameObserver;

   std::thread collection;

public:

   SynexensNode() : Node("synexens_lidar"), frameObserver(this) {

      /* the time interval to wait on in-between polling
       * for data frames from the sensor.                */
      this->declare_parameter("poll_delay",10);

      /* the topic to publish point cloud data on */
      this->declare_parameter("cloud_out","points");

      /* the frame id to publish the point cloud data from */
      this->declare_parameter("frame_id","synexens_link");

      /* the stream type to set the sensor to */
      /* options are [ depth, depth_ir, raw, rgb, rgbd, rgbdai ] */
      this->declare_parameter("stream_type","depth");

      /* the resolution to try for. Options are:
       *    [ 320_240, 640_480, 960_540, 1920_1080, 1600_1200 ] */
      this->declare_parameter("resolution","320_240");

      /* the color resolution to try for. Options are:
       *    [ 320_240, 640_480, 960_540, 1920_1080, 1600_1200 ] */
      this->declare_parameter("color_resolution","320_240");

      if (this->get_parameter("stream_type").as_string() == "depth")
         streamType = Synexens::SYSTREAMTYPE_DEPTH;
      else if (this->get_parameter("stream_type").as_string() == "raw")
         streamType = Synexens::SYSTREAMTYPE_RAW;
      else if (this->get_parameter("stream_type").as_string() == "rgb")
         streamType = Synexens::SYSTREAMTYPE_RGB;
      else if (this->get_parameter("stream_type").as_string() == "rgbd")
         streamType = Synexens::SYSTREAMTYPE_RGBD;
      else if (this->get_parameter("stream_type").as_string() == "rgbdai")
         streamType = Synexens::SYSTREAMTYPE_RGBDAI;
      else if (this->get_parameter("stream_type").as_string() == "depth_ir")
         streamType = Synexens::SYSTREAMTYPE_DEPTHIR;
      else {
         RCLCPP_ERROR(this->get_logger(),"Invalid stream type selected. Please select one of [ depth, depth_ir, raw, rgb, rgbd, rgbdai ]");
      }

      if (this->get_parameter("resolution").as_string() == "320_240")
         resolutionType = Synexens::SYRESOLUTION_320_240;
      else if (this->get_parameter("resolution").as_string() == "640_480")
         resolutionType = Synexens::SYRESOLUTION_640_480;
      else if (this->get_parameter("resolution").as_string() == "960_540")
         resolutionType = Synexens::SYRESOLUTION_960_540;
      else if (this->get_parameter("resolution").as_string() == "1920_1080")
         resolutionType = Synexens::SYRESOLUTION_1920_1080;
      else if (this->get_parameter("resolution").as_string() == "1600_1200")
         resolutionType = Synexens::SYRESOLUTION_1600_1200;
      else {
         RCLCPP_ERROR(this->get_logger(),"Invalid resolution selected. Please select one of [ 320_240, 640_480, 960_540, 1920_1080, 1600_1200 ]");
      }

      if (this->get_parameter("color_resolution").as_string() == "320_240")
         color_resolutionType = Synexens::SYRESOLUTION_320_240;
      else if (this->get_parameter("color_resolution").as_string() == "640_480")
         color_resolutionType = Synexens::SYRESOLUTION_640_480;
      else if (this->get_parameter("color_resolution").as_string() == "960_540")
         color_resolutionType = Synexens::SYRESOLUTION_960_540;
      else if (this->get_parameter("color_resolution").as_string() == "1920_1080")
         color_resolutionType = Synexens::SYRESOLUTION_1920_1080;
      else if (this->get_parameter("color_resolution").as_string() == "1600_1200")
         resolutionType = Synexens::SYRESOLUTION_1600_1200;
      else {
         RCLCPP_ERROR(this->get_logger(),"Invalid color_resolution selected. Please select one of [ 320_240, 640_480, 960_540, 1920_1080, 1600_1200 ]");
      }

      /* instantiate ros2 vars */
      cloud_out = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            this->get_parameter("cloud_out").as_string(), 10);

      /* Perform data collection from sensors */

      errorCode = Synexens::InitSDK();
      print_synexens_error(errorCode);

      errorCode = Synexens::RegisterFrameObserver(&frameObserver);
      print_synexens_error(errorCode);

      int versionLength;
      errorCode = Synexens::GetSDKVersion(versionLength,nullptr);
      print_synexens_error(errorCode);
      char * stringVersion = new char[versionLength];
      errorCode = Synexens::GetSDKVersion(versionLength,stringVersion);
      print_synexens_error(errorCode);
      RCLCPP_INFO(this->get_logger(),"version: %s",stringVersion);
      delete[] stringVersion;

      errorCode = Synexens::FindDevice(deviceCount);
      print_synexens_error(errorCode);
      deviceInfo = new Synexens::SYDeviceInfo[deviceCount];
      errorCode = Synexens::FindDevice(deviceCount,deviceInfo);
      print_synexens_error(errorCode);

      RCLCPP_INFO(this->get_logger(),"device count: %d",deviceCount);

      for (int i = 0; i < deviceCount; ++i) {
         errorCode = Synexens::OpenDevice(deviceInfo[i]);
         print_synexens_error(errorCode);

         errorCode = Synexens::GetDeviceSN(deviceInfo[i].m_nDeviceID, versionLength, nullptr);
         print_synexens_error(errorCode);
         char * stringSN = new char[versionLength];
         errorCode = Synexens::GetDeviceSN(deviceInfo[i].m_nDeviceID, versionLength, stringSN);
         print_synexens_error(errorCode);
         RCLCPP_INFO(this->get_logger(),"device info: %s",stringSN);
         delete[] stringSN;

         errorCode = Synexens::GetDeviceHWVersion(deviceInfo[i].m_nDeviceID, versionLength, nullptr);
         print_synexens_error(errorCode);
         char * stringHW = new char[versionLength];
         errorCode = Synexens::GetDeviceHWVersion(deviceInfo[i].m_nDeviceID, versionLength, stringHW);
         print_synexens_error(errorCode);
         RCLCPP_INFO(this->get_logger(),"device hardware: %s",stringHW);

         errorCode = Synexens::QueryDeviceSupportFrameType(deviceInfo[i].m_nDeviceID, versionLength);
         print_synexens_error(errorCode);
         Synexens::SYSupportType * supportType = new Synexens::SYSupportType[versionLength];
         errorCode = 
            Synexens::QueryDeviceSupportFrameType(deviceInfo[i].m_nDeviceID, versionLength, supportType);
         print_synexens_error(errorCode);

         RCLCPP_INFO(this->get_logger(),"support types:");
         for (int j = 0; j < versionLength; ++j) {
            switch(supportType[j]) {
               case Synexens::SYSUPPORTTYPE_DEPTH:
                  RCLCPP_INFO(this->get_logger(),"depth");
                  break;
               case Synexens::SYSUPPORTTYPE_RGB:
                  RCLCPP_INFO(this->get_logger(),"rgb");
                  break;
               case Synexens::SYSUPPORTTYPE_RGBD:
                  RCLCPP_INFO(this->get_logger(),"rgbd");
                  break;
               case Synexens::SYSUPPORTTYPE_RGBDAI:
                  RCLCPP_INFO(this->get_logger(),"rgbdai");
                  break;
               default:
                  RCLCPP_INFO(this->get_logger(),"invalid");
                  break;
            }

            int resolutionCount;
            errorCode =
               Synexens::QueryDeviceSupportResolution(deviceInfo[i].m_nDeviceID, supportType[j], resolutionCount);
            print_synexens_error(errorCode);
            Synexens::SYResolution * resolutions = new Synexens::SYResolution[resolutionCount];
            errorCode = Synexens::QueryDeviceSupportResolution(deviceInfo[i].m_nDeviceID, supportType[j],
                                                               resolutionCount, resolutions);

            for (int k = 0; k < resolutionCount; ++k) {
               switch(resolutions[k]) {
                  case Synexens::SYRESOLUTION_320_240:
                     RCLCPP_INFO(this->get_logger(),"320_240");
                     break;
                  case Synexens::SYRESOLUTION_640_480:
                     RCLCPP_INFO(this->get_logger(),"640_480");
                     break;
                  case Synexens::SYRESOLUTION_960_540:
                     RCLCPP_INFO(this->get_logger(),"960_540");
                     break;
                  case Synexens::SYRESOLUTION_1920_1080:
                     RCLCPP_INFO(this->get_logger(),"1920_1080");
                     break;
                  case Synexens::SYRESOLUTION_1600_1200:
                     RCLCPP_INFO(this->get_logger(),"1600_1200");
                     break;
                  case Synexens::SYRESOLUTION_NULL:
                     RCLCPP_INFO(this->get_logger(),"NULL");
                     break;
               }
            }

            delete[] resolutions;
         }

         delete[] supportType;

         streamOns.insert({deviceInfo[i].m_nDeviceID, false});

         switch(deviceInfo[i].m_deviceType) {
            case Synexens::SYDEVICETYPE_CS30_SINGLE:
            case Synexens::SYDEVICETYPE_CS30_DUAL:
            {
               RCLCPP_INFO(this->get_logger(),"Device is CS30");
               errorCode = Synexens::SetFrameResolution(deviceInfo[i].m_nDeviceID,Synexens::SYFRAMETYPE_DEPTH,
                                                        resolutionType);
               print_synexens_error(errorCode);

               errorCode = Synexens::SetFrameResolution(deviceInfo[i].m_nDeviceID,Synexens::SYFRAMETYPE_RGB,
                                                        color_resolutionType);
               print_synexens_error(errorCode);

               errorCode = Synexens::StartStreaming(deviceInfo[i].m_nDeviceID,streamType);
               print_synexens_error(errorCode);

               auto findStreamType = streamTypes.find(deviceInfo[i].m_nDeviceID);
               if (findStreamType != streamTypes.end())
                  findStreamType->second = streamType;
               else
                  streamTypes.insert({deviceInfo[i].m_nDeviceID,streamType});

               streamOns[deviceInfo[i].m_nDeviceID] = true;
               break;
            }
            case Synexens::SYDEVICETYPE_CS20_SINGLE:
            {
               RCLCPP_INFO(this->get_logger(),"Device is CS20 Single");
               errorCode = Synexens::SetFrameResolution(deviceInfo[i].m_nDeviceID,Synexens::SYFRAMETYPE_DEPTH,
                                                        resolutionType);
               print_synexens_error(errorCode);

               errorCode = Synexens::StartStreaming(deviceInfo[i].m_nDeviceID,streamType);
               print_synexens_error(errorCode);

               auto findStreamType = streamTypes.find(deviceInfo[i].m_nDeviceID);
               if (findStreamType != streamTypes.end())
                  findStreamType->second = streamType;
               else
                  streamTypes.insert({deviceInfo[i].m_nDeviceID,streamType});

               streamOns[deviceInfo[i].m_nDeviceID] = true;
               break;
            }
            case Synexens::SYDEVICETYPE_CS20_DUAL:
            {
               RCLCPP_INFO(this->get_logger(),"Device is CS20 Duel");
               errorCode = Synexens::SetFrameResolution(deviceInfo[i].m_nDeviceID, 
                     Synexens::SYFRAMETYPE_DEPTH, resolutionType);
               print_synexens_error(errorCode);
               //streamType = Synexens::SYSTREAMTYPE_DEPTH;
               errorCode = Synexens::StartStreaming(deviceInfo[i].m_nDeviceID, streamType);
               print_synexens_error(errorCode);

               auto findStreamType = streamTypes.find(deviceInfo[i].m_nDeviceID);
               if (findStreamType != streamTypes.end())
                  findStreamType->second = streamType;
               else
                  streamTypes.insert({deviceInfo[i].m_nDeviceID,streamType});

               streamOns[deviceInfo[i].m_nDeviceID] = true;
               break;
            }
            case Synexens::SYDEVICETYPE_CS20_P:
            {
               RCLCPP_INFO(this->get_logger(),"Device is CS20 P");
               errorCode = Synexens::SetFrameResolution(deviceInfo[i].m_nDeviceID,
                     Synexens::SYFRAMETYPE_DEPTH, resolutionType);
               print_synexens_error(errorCode);
               //streamType = Synexens::SYSTREAMTYPE_DEPTH;
               errorCode = Synexens::StartStreaming(deviceInfo[i].m_nDeviceID, streamType);
               print_synexens_error(errorCode);

               auto findStreamType = streamTypes.find(deviceInfo[i].m_nDeviceID);
               if (findStreamType != streamTypes.end())
                  findStreamType->second = streamType;
               else
                  streamTypes.insert({deviceInfo[i].m_nDeviceID,streamType});

               streamOns[deviceInfo[i].m_nDeviceID] = true;
               break;
            }
            case Synexens::SYDEVICETYPE_CS40:
            {
               RCLCPP_INFO(this->get_logger(),"Device is CS40");
               errorCode = Synexens::SetFrameResolution(deviceInfo[i].m_nDeviceID,
                     Synexens::SYFRAMETYPE_DEPTH, resolutionType);
               print_synexens_error(errorCode);
               //streamType = Synexens::SYSTREAMTYPE_DEPTH;
               errorCode = Synexens::StartStreaming(deviceInfo[i].m_nDeviceID, streamType);
               print_synexens_error(errorCode);

               auto findStreamType = streamTypes.find(deviceInfo[i].m_nDeviceID);
               if (findStreamType != streamTypes.end())
                  findStreamType->second = streamType;
               else
                  streamTypes.insert({deviceInfo[i].m_nDeviceID,streamType});

               streamOns[deviceInfo[i].m_nDeviceID] = true;
               break;
            }
            case Synexens::SYDEVICETYPE_CS40PRO:
            {
               RCLCPP_INFO(this->get_logger(),"Device is CS40PRO");
               errorCode = Synexens::SetFrameResolution(deviceInfo[i].m_nDeviceID,
                     Synexens::SYFRAMETYPE_DEPTH, resolutionType);
               print_synexens_error(errorCode);
               errorCode = Synexens::SetFrameResolution(deviceInfo[i].m_nDeviceID,
                     Synexens::SYFRAMETYPE_RGB, color_resolutionType);
               print_synexens_error(errorCode);
               //streamType = Synexens::SYSTREAMTYPE_DEPTH;
               errorCode = Synexens::StartStreaming(deviceInfo[i].m_nDeviceID, streamType);
               print_synexens_error(errorCode);

               auto findStreamType = streamTypes.find(deviceInfo[i].m_nDeviceID);
               if (findStreamType != streamTypes.end())
                  findStreamType->second = streamType;
               else
                  streamTypes.insert({deviceInfo[i].m_nDeviceID,streamType});

               streamOns[deviceInfo[i].m_nDeviceID] = true;
               break;
            }
            case Synexens::SYDEVICETYPE_NULL:
            {
               RCLCPP_INFO(this->get_logger(),"Device is NULL");
               break;
            }
         }
      }

      /* perform polling and processing loop */
      collection = std::thread(&SynexensNode::collect_and_process, this);

      /* this is done in a seperate thread to allow us to stop the node using ^C */
   }

private:

   void collect_and_process() {
      for ( ; ; ) {

         for (int i = 0; i < deviceCount; ++i) {

            Synexens::SYFrameData * lastFrame = nullptr;
            errorCode = Synexens::GetLastFrameData(deviceInfo[i].m_nDeviceID, lastFrame);
            if (errorCode == Synexens::SYERRORCODE_SUCCESS) {
            }
         }

         std::this_thread::sleep_for(std::chrono::milliseconds(this->get_parameter("poll_delay").as_int()));

      }


      errorCode = Synexens::UnInitSDK();
      print_synexens_error(errorCode);
   }

   void print_synexens_error(Synexens::SYErrorCode code) {
      if (code)
         RCLCPP_WARN(this->get_logger(),"error has occurred\n");
      switch(code) {
         case 0:
         break;
         case 1:
         RCLCPP_ERROR(this->get_logger(),"Total failure");
         return;
         break;
         case 2:
         RCLCPP_ERROR(this->get_logger(),"Device does not exist");
         return;
         break;
         case 3:
         RCLCPP_ERROR(this->get_logger(),"Device has not been opened");
         return;
         break;
         case 4:
         RCLCPP_ERROR(this->get_logger(),"Resolution is unsupported");
         return;
         break;
         case 5:
         RCLCPP_ERROR(this->get_logger(),"Device handle is empty");
         return;
         break;
         case 6:
         RCLCPP_ERROR(this->get_logger(),"Failed to set output data format");
         return;
         break;
         case 7:
         RCLCPP_ERROR(this->get_logger(),"Failed to get video stream control pointer");
         return;
         break;
         case 8:
         RCLCPP_ERROR(this->get_logger(),"Failed to start video stream");
         return;
         break;
         case 9:
         RCLCPP_ERROR(this->get_logger(),"Communication pointer empty");
         return;
         break;
         case 10:
         RCLCPP_ERROR(this->get_logger(),"Invalid SN number");
         return;
         break;
         case 11:
         RCLCPP_ERROR(this->get_logger(),"String length is out of range");
         return;
         break;
         case 12:
         RCLCPP_ERROR(this->get_logger(),"Invalid frame type");
         return;
         break;
         case 13:
         RCLCPP_ERROR(this->get_logger(),"Invalid device type");
         return;
         break;
         case 14:
         RCLCPP_ERROR(this->get_logger(),"Device object pointer empty");
         return;
         break;
         case 15:
         RCLCPP_ERROR(this->get_logger(),"Observer object pointer empty");
         return;
         break;
         case 16:
         RCLCPP_ERROR(this->get_logger(),"Observer object not found");
         return;
         break;
         case 17:
         RCLCPP_ERROR(this->get_logger(),"Count out of range");
         return;
         break;
         case 18:
         RCLCPP_ERROR(this->get_logger(),"UVC initialization failed");
         return;
         break;
         case 19:
         RCLCPP_ERROR(this->get_logger(),"UVC device search failed");
         return;
         break;
         case 20:
         RCLCPP_ERROR(this->get_logger(),"no data frame");
         return;
         break;
         case 21:
         RCLCPP_ERROR(this->get_logger(),"Failed to get application folder path");
         return;
         break;
         case 22:
         RCLCPP_ERROR(this->get_logger(),"Video stream not started");
         return;
         break;
         case 23:
         RCLCPP_ERROR(this->get_logger(),"Algorithm pointer empty");
         return;
         break;
         case 24:
         RCLCPP_ERROR(this->get_logger(),"Video stream already started");
         return;
         break;
         case 25:
         RCLCPP_ERROR(this->get_logger(),"Unkown stream type");
         return;
         break;
         case 26:
         RCLCPP_ERROR(this->get_logger(),"Data pointer is empty");
         return;
         break;
         case 27:
         RCLCPP_ERROR(this->get_logger(),"Repeated operation");
         return;
         break;
         case 28:
         RCLCPP_ERROR(this->get_logger(),"Firmware version too low");
         return;
         break;
      }
   }


};

int main(int argc, char ** argv) {
   rclcpp::init(argc,argv);
   rclcpp::spin(std::make_shared<SynexensNode>());
   rclcpp::shutdown();
   return 0;
}
