#include <MocapApi.h>

#include <chrono>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iostream>
#include <thread>
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace MocapApi;
using json = nlohmann::json;
// 服务IP地址
std::string strServerIP = "127.0.0.1";
// 服务端口号
uint32_t strServerPort = 7001;
// 本机的UDP端口号
uint32_t localPort = 7012;
/// 输出到文件的buffer
bool bPrint2Buffer = false;
// 输出缓冲区长度
size_t nBufferLen = 1024 * 200 * 1024;
// 输出缓冲区首地址
char* logBuffer = nullptr;
// 输出缓冲区首偏移
int logOffset = 0;

#define PrintInfo(format, ...)                                           \
  if (bPrint2Buffer) {                                                   \
    logQuit();                                                           \
    logOffset += snprintf(logBuffer + logOffset, nBufferLen - logOffset, \
                          format, ##__VA_ARGS__);                        \
  } else                                                                 \
    printf(format, ##__VA_ARGS__);

void logQuit() {
  if (logOffset >= nBufferLen - 200) {
    std::ofstream outfile;
    outfile.open("./record.log");
    outfile << logBuffer;
    outfile.close();
    exit(1);
  }
}

/*
 *@brief:                           时间戳转日期
 *@param: uint64_t timestamp        接口接收到的时间戳
 *@return: void
 *@remark:
 */
void printTimestamp(uint64_t timestamp) {
  auto tp = std::chrono::time_point<std::chrono::system_clock,
                                    std::chrono::microseconds>(
      std::chrono::microseconds(timestamp));
  auto tt = std::chrono::system_clock::to_time_t(tp);
  std::tm* datetime = std::localtime(&tt);

  //PrintInfo("%04d:%02d:%02d %02d:%02d:%02d.%06d\n", datetime->tm_year + 1900,
   //         datetime->tm_mon + 1, datetime->tm_mday, datetime->tm_hour,
  //          datetime->tm_min, datetime->tm_sec, timestamp % 1000'000);
}

void publish_joint_state();

/*
 *@brief:                           printTimestamp升级版
 */
void printTimestampEx(uint64_t timestamp, const char* type, int32_t count) {
  auto tp = std::chrono::time_point<std::chrono::system_clock,
                                    std::chrono::microseconds>(
      std::chrono::microseconds(timestamp));
  auto tt = std::chrono::system_clock::to_time_t(tp);
  std::tm* datetime = std::localtime(&tt);

  //PrintInfo("%04d:%02d:%02d %02d:%02d:%02d.%06d\t", datetime->tm_year + 1900,
  //          datetime->tm_mon + 1, datetime->tm_mday, datetime->tm_hour,
  //          datetime->tm_min, datetime->tm_sec, timestamp % 1000'000);

 // PrintInfo("type: %s\t count:%d\n", type, count);
}

// 骨骼节点定义
const char* tagnames[] = {"JointTag_Hips",
                          "JointTag_RightUpLeg",
                          "JointTag_RightLeg",
                          "JointTag_RightFoot",
                          "JointTag_LeftUpLeg",
                          "JointTag_LeftLeg",
                          "JointTag_LeftFoot",
                          "JointTag_Spine",
                          "JointTag_Spine1",
                          "JointTag_Spine2",
                          "JointTag_Neck",
                          "JointTag_Neck1",
                          "JointTag_Head",
                          "JointTag_RightShoulder",
                          "JointTag_RightArm",
                          "JointTag_RightForeArm",
                          "JointTag_RightHand",
                          "JointTag_RightHandThumb1",
                          "JointTag_RightHandThumb2",
                          "JointTag_RightHandThumb3",
                          "JointTag_RightInHandIndex",
                          "JointTag_RightHandIndex1",
                          "JointTag_RightHandIndex2",
                          "JointTag_RightHandIndex3",
                          "JointTag_RightInHandMiddle",
                          "JointTag_RightHandMiddle1",
                          "JointTag_RightHandMiddle2",
                          "JointTag_RightHandMiddle3",
                          "JointTag_RightInHandRing",
                          "JointTag_RightHandRing1",
                          "JointTag_RightHandRing2",
                          "JointTag_RightHandRing3",
                          "JointTag_RightInHandPinky",
                          "JointTag_RightHandPinky1",
                          "JointTag_RightHandPinky2",
                          "JointTag_RightHandPinky3",
                          "JointTag_LeftShoulder",
                          "JointTag_LeftArm",
                          "JointTag_LeftForeArm",
                          "JointTag_LeftHand",
                          "JointTag_LeftHandThumb1",
                          "JointTag_LeftHandThumb2",
                          "JointTag_LeftHandThumb3",
                          "JointTag_LeftInHandIndex",
                          "JointTag_LeftHandIndex1",
                          "JointTag_LeftHandIndex2",
                          "JointTag_LeftHandIndex3",
                          "JointTag_LeftInHandMiddle",
                          "JointTag_LeftHandMiddle1",
                          "JointTag_LeftHandMiddle2",
                          "JointTag_LeftHandMiddle3",
                          "JointTag_LeftInHandRing",
                          "JointTag_LeftHandRing1",
                          "JointTag_LeftHandRing2",
                          "JointTag_LeftHandRing3",
                          "JointTag_LeftInHandPinky",
                          "JointTag_LeftHandPinky1",
                          "JointTag_LeftHandPinky2",
                          "JointTag_LeftHandPinky3",
                          "JointTag_JointsCount"};

// 全局变量
rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
std::vector<std::string> joint_names_;

// 全局错误码
EMCPError error = EMCPError::Error_None;
// 全局定义App接口及句柄
IMCPApplication* pGlobalApp = nullptr;
MCPApplicationHandle_t globalAppHandle = 0;
// 全局定义设置接口及句柄
IMCPSettings* pGlobalSetting = nullptr;
MCPSettingsHandle_t globalSettingHandle = 0;
// #ifdef _SUPORT_ALICE_NEW_
// Alice 数据分发
IMCPAliceHub* pAliceDataHub = nullptr;
MCPAliceBusHandle_t aliceBusHandle = 0;
// #endif
// 人物接收接口及句柄
IMCPAvatar* pAvatarInterface = nullptr;
MCPAvatarHandle_t avatarHandle = 0;
// Device接收接口及句柄
IMCPTracker* pDeviceInterface = nullptr;
MCPTrackerHandle_t deviceHandle = 0;
// IMU接收接口及句柄
IMCPSensorModule* pImuInterface = nullptr;
MCPSensorModuleHandle_t imuHandle = 0;
// #ifdef _SUPORT_ALICE_NEW_
// Marker接收接口及句柄
IMCPMarker* pMarkerInterface = nullptr;
MCPMarkerHandle_t markerHandle = 0;
// #endif
// 刚体接收接口及句柄
IMCPRigidBody* pRigidbodyInterface = nullptr;
MCPRigidBodyHandle_t rigidbodyHandle = 0;
// #ifdef _SUPORT_ALICE_NEW_
// Tracker/PWR接收接口及句柄
IMCPPWR* pPWRInterface = nullptr;
MCPPWRHandle_t pwrHandle = 0;
// #endif
// 渲染相关的接口及句柄
IMCPRenderSettings* pRenderSetting = nullptr;
MCPRenderSettingsHandle_t renderSettingHandle = 0;

// 最多事件数量
static constexpr uint32_t nMaxEventCount = 16;
// 接收事件
MCPEvent_t pEvents[nMaxEventCount] = {};
// 当前批次事件数量
uint32_t nEventCount = 0;
// 人物关节句柄
MCPJointHandle_t joint = 0;
// 全身关节句柄
MCPJointHandle_t alljoint[59] = {};

// 机器人接口
IMCPRobot* robotInterface = nullptr;

MCPRobotHandle_t pHandle = 0;

int setMCPSettings(IMCPSettings* setting, MCPSettingsHandle_t handle) {
  if (setting == nullptr || handle == 0) {
    return 1;
  }

  error = setting->SetSettingsBvhData(BvhDataType_Binary, handle);
  error = setting->SetSettingsBvhRotation(BvhRotation_XYZ, handle);
  error =
      setting->SetSettingsBvhTransformation(BvhTransformation_Disable, handle);
  // error = setting->SetSettingsTCP(strServerIP.c_str(), strServerPort, handle);
  // error = setting->SetSettingsUDPServer(strServerIP.c_str(), strServerPort, handle);
  error = setting->SetSettingsUDP(localPort, handle);
  return 0;
}

void setup() {
  // 创建并初始化全局App接口
  error = MCPGetGenericInterface(IMCPApplication_Version,
                                 reinterpret_cast<void**>(&pGlobalApp));
  error = pGlobalApp->CreateApplication(&globalAppHandle);
  // 创建并初始化全局设置接口
  MocapApi::MCPGetGenericInterface(IMCPSettings_Version,
                                   reinterpret_cast<void**>(&pGlobalSetting));
  pGlobalSetting->CreateSettings(&globalSettingHandle);
  int result = setMCPSettings(pGlobalSetting, globalSettingHandle);
  // 激活设置并销毁不再需要的设置
  error =
      pGlobalApp->SetApplicationSettings(globalSettingHandle, globalAppHandle);
  error = pGlobalSetting->DestroySettings(globalSettingHandle);

  // 添加各种数据接收接口的关联
  error = MCPGetGenericInterface(IMCPAvatar_Version,
                                 reinterpret_cast<void**>(&pAvatarInterface));
  error = MCPGetGenericInterface(IMCPSensorModule_Version,
                                 reinterpret_cast<void**>(&pImuInterface));
  // #ifdef _SUPORT_ALICE_NEW_
  error = MCPGetGenericInterface(IMCPMarker_Version,
                                 reinterpret_cast<void**>(&pMarkerInterface));
  // #endif
  error = MCPGetGenericInterface(
      IMCPRigidBody_Version,
      reinterpret_cast<void**>(
          &pRigidbodyInterface));  // 注意这里选的是Rigidbody协议2
  // #ifdef _SUPORT_ALICE_NEW_
  error = MCPGetGenericInterface(IMCPPWR_Version,
                                 reinterpret_cast<void**>(&pPWRInterface));
  error = MCPGetGenericInterface(IMCPAliceHub_Version,
                                 reinterpret_cast<void**>(&pAliceDataHub));
  // #endif
  error = MCPGetGenericInterface(IMCPTracker_Version,
                                 reinterpret_cast<void**>(&pDeviceInterface));

  for (uint32_t i = 0; i < nMaxEventCount; i++) {
    pEvents[i].size = sizeof(MCPEvent_t);
  }

  // 创建并初始化渲染相关的接口
  error = MCPGetGenericInterface(IMCPRenderSettings_Version,
                                 reinterpret_cast<void**>(&pRenderSetting));
  pRenderSetting->GetPreDefRenderSettings(PreDefinedRenderSettings_Default,
                                          &renderSettingHandle);
  pGlobalApp->SetApplicationRenderSettings(renderSettingHandle,
                                           globalAppHandle);

  // Go
  error = pGlobalApp->OpenApplication(globalAppHandle);
}

void destroy() {
  error = pGlobalApp->CloseApplication(globalAppHandle);
  error = pGlobalApp->DestroyApplication(globalAppHandle);
}

void updateJoints(MCPJointHandle_t joint, MCPAvatarHandle_t avatar) {
  MocapApi::IMCPJoint* jointMgr = nullptr;
  MocapApi::MCPGetGenericInterface(MocapApi::IMCPJoint_Version,
                                   reinterpret_cast<void**>(&jointMgr));

  const char* name = nullptr;
  error = jointMgr->GetJointName(&name, joint);
  MCPJointHandle_t tmpjoint = 0;
  pAvatarInterface->GetAvatarJointByName(name, &tmpjoint, avatar);
  float p[3];
  float r[4];
  MCPJointHandle_t childjoints[5];
  EMCPJointTag jointtags[5];
  uint32_t numberOfChildren = 0;
  EMCPJointTag jointT;
  if (joint) {
    error = jointMgr->GetJointLocalRotation(&r[0], &r[1], &r[2], &r[3], joint);
    error = jointMgr->GetJointLocalPosition(&p[0], &p[1], &p[2], joint);
    if (error !=
        Error_None)  // 这里用来规避勾选“不带位移”后仍调用该接口所产生的野值
    {
      p[0] = p[1] = p[2] = 0;
    }
    error = jointMgr->GetJointTag(&jointT, joint);

    //PrintInfo(
    //    "JointName:%s  JointTag<%d>:%s \n    pos:(%f, %f, %f)   quat(%f, %f, "
    //    "%f, %f)\n",
    //    name, jointT, tagnames[jointT], p[0], p[1], p[2], r[3], r[0], r[1],
    //    r[2]);

    error = jointMgr->GetJointLocalRotationByEuler(&p[0], &p[1], &p[2], joint);

    //PrintInfo("    eular:(%f, %f, %f)\n", p[0], p[1], p[2]);

    const char* tmpname = nullptr;
    error = jointMgr->GetJointNameByTag(&tmpname, jointT);
    EMCPJointTag tmpjointT;
    error = jointMgr->GetJointParentJointTag(&tmpjointT, jointT);
    error = jointMgr->GetJointChild(nullptr, &numberOfChildren, joint);

    if (numberOfChildren > 0) {
      error = jointMgr->GetJointChild(childjoints, &numberOfChildren, joint);
      for (uint32_t j = 0; j < numberOfChildren; j++) {
        updateJoints(childjoints[j], avatar);
      }
    }
  }
  return;
}
// 函数声明
// 读取 JSON 文件
json readJsonFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        return nullptr;
    }

    json config;
    file >> config;
    file.close();
    return config;
}
void timer_callback()
{
    publish_joint_state();
}
int main(int argc, char** argv) {
  // if (argc > 1) strServerIP = argv[1];
  // if (argc > 2) strServerPort = atoi(argv[2]);
  // if (argc > 3) {
  //   bPrint2Buffer = true;
  //   nBufferLen = (std::max)(1, atoi(argv[3])) * 1024;
  //   logBuffer = new char[nBufferLen];
  // }

  if (argc > 1) localPort = atoi(argv[1]);

  printf("***MocapAPI***\n");
  printf("local port: %d\n", localPort);
  setup();

  // 免得每次都要构造和析构，提前定义一点变量
 

  // 根据版本获取机器人接口
  MCPGetGenericInterface(IMCPRobot_Version, (void**)&robotInterface);
  
  // 读取 retarget.json 文件
  json retarget = readJsonFile("retarget.json");

  if (retarget.is_null() || retarget.empty()) {
        std::cerr << "retarget.json is empty or invalid, please check." << std::endl;
    }
  // 创建一个机器人
  if (robotInterface) {
        std::string retarget_str = retarget.dump(); 
        robotInterface->CreateRobot(retarget_str.c_str(), &pHandle);
        robotInterface->SetRobotFPS(100, pHandle); // 设置 FPS
    }
  int fps = 100;
  // 输入数据帧数
  robotInterface->SetRobotFPS(fps, pHandle);
  // 确保从 JSON 中正确获取关节名称列表
  if (!retarget["urdfJointNames"].is_null() && retarget["urdfJointNames"].is_array()) {
      for (const auto& joint_name : retarget["urdfJointNames"]) {
          if (joint_name.is_string()) {
              joint_names_.push_back(joint_name.get<std::string>());
          }
      }
  }
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("json_to_joint_state_publisher");
  publisher_ = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  rclcpp::TimerBase::SharedPtr timer_ = node->create_wall_timer(std::chrono::milliseconds(100), timer_callback);
  rclcpp::spin(node);

  rclcpp::shutdown();
  robotInterface->DestroyRobot(pHandle);
  destroy();
  return 0;
}


void publish_joint_state(){


    if (!(&pHandle)|| !robotInterface) {
      return;
    }
    int32_t itemsCount;
    uint64_t timestamp;
    uint32_t count;
    uint32_t id;
    int id2, status;
    nEventCount = 0;
    uint32_t size;
    int32_t indexes[1024];
    float pos[3], acce[3], gyro[3], quat[4], axisAngle[4];

    time_t now = time(0);      // 当前系统时间精确到秒
    tm* gm = localtime(&now);  // 转为gmt时间
    time_t utc = mktime(gm);   // gmt对应的时间戳

    int64_t time = utc - now;  // 秒级时区转化
    uint64_t baseTime = time * 1e3;

    error = pGlobalApp->PollApplicationNextEvent(nullptr, &nEventCount,
                                                 globalAppHandle);
    if (nEventCount) {
      error = pGlobalApp->PollApplicationNextEvent(pEvents, &nEventCount,
                                                   globalAppHandle);
      for (uint32_t i = 0; i < nEventCount; i++) {
        if (pEvents[i].eventType == MCPEvent_AvatarUpdated) {
          avatarHandle = pEvents[i].eventData.motionData.avatarHandle;
          const char* tmpName = nullptr;
          error = pAvatarInterface->GetAvatarRootJoint(&joint, avatarHandle);
          error = pAvatarInterface->GetAvatarName(&tmpName, avatarHandle);

          //PrintInfo("AvatarName: %s\n", tmpName);
          uint32_t jointSize = 0;
          error = pAvatarInterface->GetAvatarJoints(nullptr, &jointSize,
                                                    avatarHandle);
          error = pAvatarInterface->GetAvatarJoints(alljoint, &jointSize,
                                                    avatarHandle);
          MCPJointHandle_t jointHandle = 0;
          error = pAvatarInterface->GetAvatarJointByName("Hips", &jointHandle,
                                                         avatarHandle);
          if (error == Error_None && jointSize > 0) {
            updateJoints(jointHandle, avatarHandle);
          }

          // 每帧获取机器人的关节数据
          robotInterface->UpdateRobot(avatarHandle, pHandle);
          float p[3];  // xyz [14/01/2025 brian.wang]
          float r[4];  // xyzw [14/01/2025 brian.wang]
          // root节点数据
          robotInterface->GetRobotRootPosition(&p[0], &p[1], &p[2], pHandle);
          robotInterface->GetRobotRootRotation(&r[0], &r[1], &r[2], &r[3],
                                               pHandle);

          //PrintInfo("RobotRoot: pos:(%f, %f, %f)   quat(%f, %f, %f, %f)\n",
          //        p[0], p[1], p[2], r[3], r[0], r[1], r[2]);

          // jointName为此retarget的具体关节节点，可根据需求输入，调用后返回一个角度。
          const char* jointName = "r-j1";
          float value = 0;
          robotInterface->GetRobotRetargetJointAngle(jointName, &value,
                                                     pHandle);
          //PrintInfo("RobotjointName:%s :%f  \n", jointName, value);
          // 获取jsonStr
          bool compress = true;
          const char* result;
	  sensor_msgs::msg::JointState joint_state_msg;
          EMCPError robotStatus =
              robotInterface->GetRobotRosFrameJson(&result, compress, pHandle);
          if (robotStatus == Error_None) {
            //PrintInfo("jsonStr: %s  \n", result);
            json real_time_data = json::parse(result);
            joint_state_msg.header.stamp = rclcpp::Clock().now();
            joint_state_msg.name = joint_names_;
            std::vector<double> joint_positions(joint_names_.size(), 0.0);
            for (size_t i = 0; i < joint_names_.size(); ++i)
            {
                if (real_time_data["joint_positions"].contains(joint_names_[i]))
                {
                    joint_positions[i] = real_time_data["joint_positions"][joint_names_[i]].get<double>();
                }
            }

            joint_state_msg.position = joint_positions;
            publisher_->publish(joint_state_msg);
	    // for (int i = 0; i < joint_positions.size(); i++) {
		//std::cout << joint_names_[i] << ",";
		//std::cout << joint_positions[i] << ",";
	   // }

          } else {
          }
          robotInterface->RunRobotStep(pHandle);

        }
        // #ifdef _SUPORT_ALICE_NEW_
        else if (pEvents[i].eventType == MCPEvent_AliceMarkerUpdated) {
          pAliceDataHub->GetMarkerList(
              nullptr, &count);  // 第一次传入参1为nullptr，意为获取对象的数量
          if (count > 0) {
            MCPMarkerHandle_t* recv =
                new MCPMarkerHandle_t[count];  // 动态开辟接收内存
            pAliceDataHub->GetMarkerList(
                recv,
                &count);  // 第二次传入参1不为nullptr，将接收实际的handle列表
            pAliceDataHub->GetMarkerTimestamp(
                &timestamp);  // 以包为单位的时间戳

            printTimestampEx(timestamp, "Marker", count);
            for (size_t i = 0; i < count; ++i) {
              pMarkerInterface->GetMarkerPosition(&pos[0], &pos[1], &pos[2],
                                                  recv[i]);
              //PrintInfo("marker:(%f, %f, %f)\n", pos[0], pos[1], pos[2]);
            }

            delete[] recv;
            recv = nullptr;
          }
        } else if (pEvents[i].eventType == MCPEvent_AliceIMUUpdated) {
          // 详见 MCPEvent_AliceMarkerUpdated 类型事件接收的注释
          pAliceDataHub->GetSensorModuleList(nullptr, &count);
          if (count > 0) {
            MCPSensorModuleHandle_t* recv = new MCPSensorModuleHandle_t[count];
            pAliceDataHub->GetSensorModuleList(recv, &count);
            pAliceDataHub->GetSensorModuleTimestamp(&timestamp);

            printTimestampEx(timestamp, "IMU", count);
            for (size_t i = 0; i < count; ++i) {
              pImuInterface->GetSensorModuleId(&id, recv[i]);
              pImuInterface->GetSensorModuleAcceleratedVelocity(
                  &acce[0], &acce[1], &acce[2], recv[i]);
              pImuInterface->GetSensorModuleAngularVelocity(&gyro[0], &gyro[1],
                                                            &gyro[2], recv[i]);
              pImuInterface->GetSensorModulePosture(
                  &quat[0], &quat[1], &quat[2], &quat[3], recv[i]);

              //PrintInfo(
               //   "id:(%d), accelerometer:(%f, %f, %f), gyroscope:(%f, %f, "
                //  "%f), quat:(%f, %f, %f, %f)\n",
                //  id, acce[0], acce[1], acce[2], gyro[0], gyro[1], gyro[2],
                 // quat[0], quat[1], quat[2], quat[3]);
            }
            delete[] recv;
            recv = nullptr;
          }
        } else if (pEvents[i].eventType == MCPEvent_AliceRigidbodyUpdated) {
          // 详见 MCPEvent_AliceMarkerUpdated 类型事件接收的注释
          pAliceDataHub->GetRigidBodyList(nullptr, &count);
          if (count > 0) {
            MCPRigidBodyHandle_t* recv = new MCPRigidBodyHandle_t[count];
            pAliceDataHub->GetRigidBodyList(recv, &count);
            pAliceDataHub->GetRigidBodyTimestamp(&timestamp);

            printTimestampEx(timestamp, "Rigid body", count);
            for (size_t i = 0; i < count; ++i) {
              pRigidbodyInterface->GetRigidBodyId(&id2, recv[i]);
              pRigidbodyInterface->GetRigidBodyPosition(&pos[0], &pos[1],
                                                        &pos[2], recv[i]);
              pRigidbodyInterface->GetRigidBodyRotation(
                  &quat[0], &quat[1], &quat[2], &quat[3], recv[i]);
              pRigidbodyInterface->GetRigidBodyAxisAngle(
                  &axisAngle[0], &axisAngle[1], &axisAngle[2], &axisAngle[3],
                  recv[i]);
             // PrintInfo(
              //    "id:(%d), pos:(%f, %f, %f), quat:(%f, %f, %f, "
              //    "%f(w))\naxisAngle:((%f, %f, %f), %f)\n",
              //    id2, pos[0], pos[1], pos[2], quat[0], quat[1], quat[2],
               //   quat[3], axisAngle[0], axisAngle[1], axisAngle[2],
                //  axisAngle[3]);
            }
            delete[] recv;
            recv = nullptr;
          }
        } else if (pEvents[i].eventType == MCPEvent_AliceTrackerUpdated) {
          // 详见 MCPEvent_AliceMarkerUpdated 类型事件接收的注释
          pAliceDataHub->GetPWRList(nullptr, &count);
          if (count > 0) {
            MCPPWRHandle_t* recv = new MCPPWRHandle_t[count];
            pAliceDataHub->GetPWRList(recv, &count);
            pAliceDataHub->GetPWRTimestamp(&timestamp);

            printTimestampEx(timestamp, "Tracker", count);

            for (size_t i = 0; i < count; ++i) {
              pPWRInterface->GetPWRId(&id, recv[i]);
              pPWRInterface->GetPWRStatus(&status, recv[i]);
              pPWRInterface->GetPWRPosition(&pos[0], &pos[1], &pos[2], recv[i]);
              pPWRInterface->GetPWRQuaternion(&quat[0], &quat[1], &quat[2],
                                              &quat[3], recv[i]);
             // PrintInfo(
              //    "id:(%d), status(%d), pos:(%f, %f, %f), quat:(%f, %f, %f, "
               //   "%f)\n",
                //  id, status, pos[0], pos[1], pos[2], quat[0], quat[1], quat[2],
                //  quat[3]);
            }

            delete[] recv;
            recv = nullptr;
          }
        }
        // #endif
        else if (pEvents[i].eventType == MCPEvent_TrackerUpdated) {
          deviceHandle = pEvents[i].eventData.trackerData._trackerHandle;
          int count = 0;
          pDeviceInterface->GetDeviceCount(&count, deviceHandle);

          auto tp = std::chrono::time_point_cast<std::chrono::milliseconds>(
              std::chrono::system_clock::now());
          //PrintInfo("[Device Timestamp]%lld\n",
           //         std::chrono::duration_cast<std::chrono::milliseconds>(
            //            tp.time_since_epoch())
           //                 .count() +
            //            baseTime);
          for (int i = 0; i < count; ++i) {
            const char* name;  // 这里无需开辟内存
            pDeviceInterface->GetDeviceName(i, &name, deviceHandle);
            pDeviceInterface->GetTrackerPosition(&pos[0], &pos[1], &pos[2],
                                                 name, deviceHandle);
            pDeviceInterface->GetTrackerRotation(&quat[0], &quat[1], &quat[2],
                                                 &quat[3], name, deviceHandle);

           // PrintInfo(
           //     "[Device]name:(%s), pos:(%f, %f, %f), quat:(%f, %f, %f, %f)\n",
            //    name, pos[0], pos[1], pos[2], quat[0], quat[1], quat[2],
             //   quat[3]);
          }
        }
      }
    }
  }
