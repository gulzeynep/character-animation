#include "n8ro_motion/motion_controller.hpp"

#include <model/AnimationModel.h>
#include <model/IModel.h>
#include <model/ModelFactoryRegistry.h>
#include <plugin/IModelPluginService.h>
#include <plugin/IPlugin.h>
#include <plugin/IPluginServices.h>
#include <plugin/PluginContext.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

namespace {

constexpr std::string_view kPluginId = "character_plugin_220201014";
constexpr std::string_view kModelType = "animationModelNathanHuman";
constexpr std::string_view kWalkAnimationCode = "Zeynep Walk";
constexpr std::string_view kSquatAnimationCode = "Zeynep Squat";
constexpr const char* kRuntimeLogPath = "C:\\N8RO2\\userPlugins\\sim\\character_plugin_220201014_runtime.log";
constexpr std::array<std::string_view, 7> kAnimationCodes{
    kWalkAnimationCode,
    kSquatAnimationCode,
    "Idle Neutral",
    "Idle Breathing",
    "Idle Alert",
    "Idle Shake",
    "Idle Stopped",
};

bool equals(std::string_view left, const std::string& right)
{
    return left.size() == right.size() && right.compare(0, right.size(), left.data(), left.size()) == 0;
}

bool isHandledAnimationCode(const std::string& animation_code)
{
    for (const auto code : kAnimationCodes) {
        if (equals(code, animation_code)) {
            return true;
        }
    }

    return false;
}

void writeRuntimeLog(const std::string& line)
{
    std::ofstream log(kRuntimeLogPath, std::ios::app);
    if (log) {
        log << line << '\n';
    }
}

void writeEvaluateSample(const std::string& animation_code, double time_seconds, std::size_t override_count)
{
    static int sample_counter = 0;
    ++sample_counter;
    if (sample_counter % 50 != 1) {
        return;
    }

    std::ofstream log(kRuntimeLogPath, std::ios::app);
    if (log) {
        log << "evaluate activeAnimationCode=\"" << animation_code
            << "\" t=" << time_seconds
            << " overrides=" << override_count << '\n';
    }
}

bool hasJoint(const std::unordered_set<std::string>& joints, const char* joint_id)
{
    if (joint_id == nullptr || *joint_id == '\0') {
        return false;
    }

    return joints.empty() || joints.find(joint_id) != joints.end();
}

double angle(const n8ro_motion::JointAngles& angles, n8ro_motion::JointIndex index)
{
    return angles.radians[static_cast<std::size_t>(index)];
}

bool isSquatAnimationCode(const std::string& animation_code)
{
    return equals(kSquatAnimationCode, animation_code);
}

void addJointOverride(arkheon::astsim::AnimationModelOutput& output,
                      const std::unordered_set<std::string>& available_joints,
                      const char* joint_id,
                      double x_rad,
                      double y_rad,
                      double z_rad)
{
    if (!hasJoint(available_joints, joint_id)) {
        return;
    }

    output.jointOverrides.push_back({joint_id, x_rad, y_rad, z_rad});
}

class ComWalkAnimationModel final : public arkheon::astsim::IModel, public arkheon::astsim::IAnimationModel {
public:
    [[nodiscard]] std::string getTypeName() const override
    {
        return "simCharAnimComWalkModel";
    }

    [[nodiscard]] std::unique_ptr<arkheon::astsim::IModel> clone() const override
    {
        return std::make_unique<ComWalkAnimationModel>(*this);
    }

    [[nodiscard]] bool evaluate(const arkheon::astsim::AnimationModelInput& input,
                                arkheon::astsim::AnimationModelOutput& output) override
    {
        if (!isHandledAnimationCode(input.entity.activeAnimationCode)) {
            return false;
        }

        std::unordered_set<std::string> available_joints;
        available_joints.reserve(input.entity.joints.size());
        for (const auto& joint : input.entity.joints) {
            available_joints.insert(joint.jointId);
        }

        n8ro_motion::MotionInput motion{};
        motion.time_seconds = input.simulationTimeSeconds;
        motion.delta_seconds = input.deltaTimeSeconds > 0.0 ? input.deltaTimeSeconds : 0.02;
        motion.target_speed = 1.0;

        const n8ro_motion::JointAngles angles = gait_.update(motion);

        output.clearExistingJointOverrides = true;
        output.jointOverrides.clear();
        output.jointOverrides.reserve(n8ro_motion::kJointCount);

        if (isSquatAnimationCode(input.entity.activeAnimationCode)) {
            const double t = input.simulationTimeSeconds;
            const double squat = 0.5 + 0.5 * std::sin(t * 2.0);
            const double balance = std::sin(t * 2.8 + 0.6);
            const double shoulder_lift = 0.85 + balance * 0.22;
            const double elbow_bend = 0.95 + squat * 0.35;
            const double knee_bend = 0.70 + squat * 0.42;
            const double ankle_counter = -0.32 - squat * 0.18;

            addJointOverride(output, available_joints, "leftHip", -0.50 - squat * 0.18, balance * 0.08, 1.05);
            addJointOverride(output, available_joints, "rightHip", -0.50 - squat * 0.18, -balance * 0.08, -1.05);
            addJointOverride(output, available_joints, "leftKnee", knee_bend, 0.0, 0.0);
            addJointOverride(output, available_joints, "rightKnee", knee_bend, 0.0, 0.0);
            addJointOverride(output, available_joints, "leftAnkle", ankle_counter, 0.0, 0.0);
            addJointOverride(output, available_joints, "rightAnkle", ankle_counter, 0.0, 0.0);
            addJointOverride(output, available_joints, "leftShoulder", shoulder_lift, balance * 0.18, -1.25 + balance * 0.20);
            addJointOverride(output, available_joints, "rightShoulder", shoulder_lift, -balance * 0.18, 1.25 - balance * 0.20);
            addJointOverride(output, available_joints, "leftElbow", elbow_bend, balance * 0.08, -0.25);
            addJointOverride(output, available_joints, "rightElbow", elbow_bend, -balance * 0.08, 0.25);
            writeEvaluateSample(input.entity.activeAnimationCode, input.simulationTimeSeconds, output.jointOverrides.size());
            return !output.jointOverrides.empty();
        }

        const double left_hip = angle(angles, n8ro_motion::JointIndex::LeftHipPitch);
        const double right_hip = angle(angles, n8ro_motion::JointIndex::RightHipPitch);
        const double left_knee = angle(angles, n8ro_motion::JointIndex::LeftKneePitch);
        const double right_knee = angle(angles, n8ro_motion::JointIndex::RightKneePitch);
        const double left_ankle = angle(angles, n8ro_motion::JointIndex::LeftAnklePitch);
        const double right_ankle = angle(angles, n8ro_motion::JointIndex::RightAnklePitch);
        const double left_shoulder = angle(angles, n8ro_motion::JointIndex::LeftShoulderPitch);
        const double right_shoulder = angle(angles, n8ro_motion::JointIndex::RightShoulderPitch);
        const double left_elbow = angle(angles, n8ro_motion::JointIndex::LeftElbowPitch);
        const double right_elbow = angle(angles, n8ro_motion::JointIndex::RightElbowPitch);
        const double proof_step = std::sin(input.simulationTimeSeconds * 4.2);
        const double arm_wave = std::sin(input.simulationTimeSeconds * 5.6 + 0.8);
        const double left_lift = std::max(0.0, proof_step);
        const double right_lift = std::max(0.0, -proof_step);

        // NathanHuman needs these mirrored Z offsets; the larger swing makes the DLL easy to verify in GLB.
        addJointOverride(output, available_joints, "leftHip", -0.12 + left_hip * 0.95 + left_lift * 0.20, arm_wave * 0.06, 1.05);
        addJointOverride(output, available_joints, "rightHip", -0.12 + right_hip * 0.95 + right_lift * 0.20, -arm_wave * 0.06, -1.05);
        addJointOverride(output, available_joints, "leftKnee", 0.10 + left_knee * 1.05 + left_lift * 0.22, 0.0, 0.0);
        addJointOverride(output, available_joints, "rightKnee", 0.10 + right_knee * 1.05 + right_lift * 0.22, 0.0, 0.0);
        addJointOverride(output, available_joints, "leftAnkle", -0.12 + left_ankle * 0.75 - left_lift * 0.10, 0.0, 0.0);
        addJointOverride(output, available_joints, "rightAnkle", -0.12 + right_ankle * 0.75 - right_lift * 0.10, 0.0, 0.0);
        addJointOverride(output, available_joints, "leftShoulder", 1.05 + left_shoulder * 0.75 - proof_step * 0.38, arm_wave * 0.12, -1.35 - proof_step * 0.22);
        addJointOverride(output, available_joints, "rightShoulder", 1.05 + right_shoulder * 0.75 + proof_step * 0.38, -arm_wave * 0.12, 1.35 - proof_step * 0.22);
        addJointOverride(output, available_joints, "leftElbow", 0.65 + left_elbow * 0.55 + right_lift * 0.15, arm_wave * 0.08, -0.15);
        addJointOverride(output, available_joints, "rightElbow", 0.65 + right_elbow * 0.55 + left_lift * 0.15, -arm_wave * 0.08, 0.15);

        writeEvaluateSample(input.entity.activeAnimationCode, input.simulationTimeSeconds, output.jointOverrides.size());
        return !output.jointOverrides.empty();
    }

private:
    n8ro_motion::GaitController gait_;
};

class ComWalkPlugin final : public arkheon::astlib::IPlugin {
public:
    [[nodiscard]] int getInterfaceVersion() const override
    {
        return 1;
    }

    [[nodiscard]] arkheon::astlib::PluginMetadata getMetadata() const override
    {
        arkheon::astlib::PluginMetadata metadata;
        metadata.setPluginId(kPluginId);
        metadata.setVersion("1.0.0");
        metadata.setAuthor("Character Animation Final Project");
        return metadata;
    }

    void initialize(arkheon::astlib::PluginContext& context) override
    {
        {
            std::ofstream log(kRuntimeLogPath, std::ios::trunc);
            if (log) {
                log << "initialize pluginId=" << kPluginId << '\n';
            }
        }

        initialized_ = true;
        shutdown_ = false;
        plugin_id_ = context.metadata.pluginId();
        if (plugin_id_.empty()) {
            plugin_id_ = std::string(kPluginId);
        }

        model_factory_registry_ = nullptr;
        if (context.services != nullptr) {
            auto* service = context.services->getService(arkheon::astsim::IModelPluginService::kPluginServiceId);
            auto* model_plugin_service = static_cast<arkheon::astsim::IModelPluginService*>(service);
            model_factory_registry_ = model_plugin_service != nullptr ? &model_plugin_service->modelFactoryRegistry() : nullptr;
        }

        if (model_factory_registry_ == nullptr) {
            writeRuntimeLog("model factory registry not available");
            return;
        }

        auto* prototype_base = model_factory_registry_->getRegisteredPrototype(kModelType);
        auto* prototype_animation_model = dynamic_cast<arkheon::astsim::IAnimationModel*>(prototype_base);
        if (prototype_animation_model == nullptr) {
            writeRuntimeLog("animationModelNathanHuman prototype not available");
            return;
        }

        for (const auto animation_code : kAnimationCodes) {
            registerAnimationCode(*prototype_animation_model, std::string(animation_code));
        }
    }

    void tick(double dt) override
    {
        static_cast<void>(dt);
        if (!initialized_ || shutdown_) {
            return;
        }
    }

    void shutdown() override
    {
        if (model_factory_registry_ != nullptr) {
            auto* prototype_base = model_factory_registry_->getRegisteredPrototype(kModelType);
            auto* prototype_animation_model = dynamic_cast<arkheon::astsim::IAnimationModel*>(prototype_base);
            for (const auto& animation_code : registered_animation_codes_) {
                if (prototype_animation_model != nullptr) {
                    static_cast<void>(prototype_animation_model->registerAnimation(
                        animation_code,
                        arkheon::astsim::IAnimationModel::AnimationEvaluationFunction{}));
                }
            }
        }

        registered_animation_codes_.clear();
        model_factory_registry_ = nullptr;
        shutdown_ = true;
    }

private:
    void registerAnimationCode(arkheon::astsim::IAnimationModel& prototype_animation_model, std::string animation_code)
    {
        const bool registered = prototype_animation_model.registerAnimation(
            animation_code,
            [](const arkheon::astsim::AnimationModelInput& input,
               arkheon::astsim::AnimationModelOutput& output) {
                thread_local ComWalkAnimationModel model;
                return model.evaluate(input, output);
            });

        if (registered) {
            writeRuntimeLog(std::string("registered animationCode=\"") + animation_code + "\"");
            registered_animation_codes_.push_back(std::move(animation_code));
        }
    }

    bool initialized_ = false;
    bool shutdown_ = false;
    std::string plugin_id_ = std::string(kPluginId);
    arkheon::astsim::ModelFactoryRegistry* model_factory_registry_ = nullptr;
    std::vector<std::string> registered_animation_codes_;
};

}  // namespace

extern "C" {

ARKHEON_ASTLIB_API arkheon::astlib::IPlugin* create_plugin()
{
    return new ComWalkPlugin();
}

ARKHEON_ASTLIB_API void destroy_plugin(arkheon::astlib::IPlugin* plugin)
{
    delete plugin;
}

ARKHEON_ASTLIB_API const char* get_plugin_signature()
{
    return "ARKHEON_PLUGIN_V1";
}

}  // extern "C"
