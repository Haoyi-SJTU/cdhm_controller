#pragma once

#include <onnxruntime_cxx_api.h>
#include <vector>
#include <string>


class RobotAIController {
private:
    Ort::Env env;
    Ort::SessionOptions session_options;
    Ort::Session* session = nullptr;
    Ort::MemoryInfo memory_info;

    // ONNX 导出时定义的输入输出节点名称
    const char* input_names[1] = {"state_input"};
    const char* output_names[1] = {"residual_output"};
    std::vector<float> input_data;
    std::vector<int64_t> input_shape;

    std::vector<float> q_idea_mean;
    std::vector<float> q_idea_std;

    std::vector<float> c_tens_mean;
    std::vector<float> c_tens_std;

    std::vector<float> c_resi_mean;
    std::vector<float> c_resi_std;

public:

    RobotAIController(const std::string& model_path);// 初始化并加载模型
    ~RobotAIController();// 释放 ONNX Session 内存
    // 推理函数：在控制循环中调用
    std::vector<float> computeResidualForces(const std::vector<float>& q_idea, const std::vector<float>& c_tens);

    std::vector<float> normalize_q_idea(const std::vector<float>& q_idea) const;// q_idea归一化 6维
    std::vector<float> normalize_c_tens(const std::vector<float>& c_tens) const;// c_tens归一化
    std::vector<float> denormalize_c_resi(const std::vector<float>& c_resi_norm) const;// 输出c_resi 反归一化 9维
};
