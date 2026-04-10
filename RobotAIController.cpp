#include <iostream>
#include <stdexcept>
#include <RobotAIController.h>
#include <algorithm> // 用于 std::copy

// 使用训练好的模型(onnx文件)，输入真机数据输出预测驱动绳残差




// 初始化并加载模型
RobotAIController::RobotAIController(const std::string &model_path)
    : env(ORT_LOGGING_LEVEL_WARNING, "RobotAIEnv"),
    memory_info(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
{
    session_options.SetIntraOpNumThreads(1); // 设置 1 个线程执行计算，对于这种小型 MLP，单线程反而比多线程快（减少上下文切换）
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL); // 开启所有图优化

    q_idea_mean = { -6.9245e-19f, 1.8465e-19f, -3.2314e-19f, 7.3861e-19f, -3.6930e-19f, 1.4772e-18f};
    q_idea_std = { 0.0834f, 0.1004f, 0.0834f, 0.1004f, 0.1004f, 0.1149f };
    c_tens_mean = { 119.7899f, 126.8709f, 16.6652f, 42.9557f, 9.1331f, 40.1373f, 38.7174f, 16.4306f, 50.5339f };
    c_tens_std = { 27.3804f, 48.1706f, 8.9759f, 18.6421f, 5.3066f, 23.8574f, 18.4439f, 7.5011f, 30.1357f };
    c_resi_mean = { 0.0266f, -0.3540f, -0.7525f, 0.1789f, 0.4407f, 1.0291f, -0.2014f, -0.1181f, -0.4055f };
    c_resi_std = { 1.8674f, 3.3826f, 4.6855f, 1.5866f, 3.2868f, 5.7514f, 1.8913f, 2.8387f, 4.0341f };

    // 在构造阶段一次性预分配好内存
    input_shape = {1, 15};
    input_data.resize(15, 0.0f);

    try
    {
        session = new Ort::Session(env, model_path.c_str(), session_options);
    }
    catch (const Ort::Exception &e)
    {
        std::cerr << "ONNX 模型加载失败: " << e.what() << std::endl;
    }
}

RobotAIController::~RobotAIController()
{
    if (session != nullptr) {
        delete session;
        session = nullptr;
    }
}

// 推理函数：在控制循环的子线程中调用
std::vector<float> RobotAIController::computeResidualForces(const std::vector<float> &q_idea, const std::vector<float> &c_tens)
{
    if (session == nullptr)
    {
        throw std::runtime_error("模型未加载或加载失败，无法进行推理！");
    }

    // 输入维度的安全校验
    if (q_idea.size() != 6 || c_tens.size() != 9)
    {
        throw std::invalid_argument("输入数据维度错误：q_idea 需要 6 维，c_tens 需要 9 维！");
    }

    // 直接覆盖已预分配的内存，全程零内存分配 (Zero Allocation)
    std::copy(q_idea.begin(), q_idea.end(), input_data.begin());
    std::copy(c_tens.begin(), c_tens.end(), input_data.begin() + q_idea.size());

    try
    {
        // 将 C++ 数据包装成 ONNX 张量 (绑定指针)
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info,
            input_data.data(),
            input_data.size(),
            input_shape.data(),
            input_shape.size());

        // 执行推理
        auto output_tensors = session->Run(
            Ort::RunOptions{nullptr},
            input_names,
            &input_tensor,
            1,
            output_names,
            1);

        // 提取输出数据
        float *floatarr = output_tensors.front().GetTensorMutableData<float>();
        return std::vector<float>(floatarr, floatarr + 9);
    }
    catch (const Ort::Exception& e)
    {
        std::cerr << "ONNX 推理过程中发生异常: " << e.what() << std::endl;
        return std::vector<float>(9, 0.0f);// 发生异常时，返回全 0 的安全残差
    }
}

// q_idea归一化 6维
std::vector<float> RobotAIController::normalize_q_idea(const std::vector<float>& q_idea) const {
    std::vector<float> res(6);
    for (size_t i = 0; i < 6; ++i) {
        res[i] = (q_idea[i] - q_idea_mean[i]) / q_idea_std[i];
    }
    return res;
}

// c_tens归一化
std::vector<float> RobotAIController::normalize_c_tens(const std::vector<float>& c_tens) const {
    std::vector<float> res(9);
    for (size_t i = 0; i < 9; ++i) {
        res[i] = (c_tens[i] - c_tens_mean[i]) / c_tens_std[i];
    }
    return res;
}

// 输出c_resi 反归一化 9维
std::vector<float> RobotAIController::denormalize_c_resi(const std::vector<float>& c_resi_norm) const {
    std::vector<float> res(9);
    for (size_t i = 0; i < 9; ++i) {
        res[i] = c_resi_norm[i] * c_resi_std[i] + c_resi_mean[i];
    }
    return res;
}











