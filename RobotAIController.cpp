// #include <onnxruntime_cxx_api.h>
// #include <vector>
#include <iostream>
#include <stdexcept>
#include <RobotAIController.h>

// 使用训练好的模型(onnx文件)，输入真机数据输出预测驱动绳残差

#include <stdexcept>
#include <algorithm> // 用于 std::copy


// 初始化并加载模型
RobotAIController::RobotAIController(const std::string &model_path)
    : env(ORT_LOGGING_LEVEL_WARNING, "RobotAIEnv"),
    memory_info(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
{
    session_options.SetIntraOpNumThreads(1); // 设置 1 个线程执行计算，对于这种小型 MLP，单线程反而比多线程快（减少上下文切换）
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL); // 开启所有图优化

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

    // 输入维度的安全校验，防止数组越界导致崩溃
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




