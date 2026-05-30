#pragma once
#include <include/arkheon/character/ICharacterController.h>
#include <cmath>
#include <algorithm>

struct AnimationSampler
{
    float blend_factor;
    int current_clip;
    int target_clip;
    int clip_ids[3] = {12, 47, 83};

    struct MotionData
    {
        float phases[10];
        float amplitudes[10];
    };

    MotionData walk = {
        {0.0f, 0.5f, 0.2f, 0.7f, 0.0f, 0.5f, 0.2f, 0.7f, 0.0f, 0.5f},
        {0.2f, 0.2f, 0.3f, 0.3f, 0.2f, 0.2f, 0.3f, 0.3f, 0.1f, 0.1f}};

    MotionData push = {
        {0.0f, 0.0f, 0.5f, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        {0.1f, 0.1f, 0.4f, 0.4f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}};

    MotionData climb = {
        {0.0f, 0.0f, 0.0f, 0.0f, 0.3f, 0.7f, 0.3f, 0.7f, 0.0f, 0.0f},
        {0.3f, 0.3f, 0.0f, 0.0f, 0.4f, 0.4f, 0.5f, 0.5f, 0.2f, 0.2f}};

    void initialize()
    {
        blend_factor = 1.0f;
        current_clip = 0;
        target_clip = 0;
    }

    void startBlend(int new_clip)
    {
        if (new_clip != current_clip)
        {
            target_clip = new_clip;
            blend_factor = 0.0f;
        }
    }

    void update(float blend_time)
    {
        if (target_clip != current_clip)
        {
            blend_factor = std::min(1.0f, blend_time / 0.2f);
            if (blend_factor >= 1.0f)
            {
                current_clip = target_clip;
            }
        }
        else
        {
            blend_factor = 1.0f;
        }
    }

    void getReferenceQuaternions(float time, arkheon_quat out_quats[10])
    {
        MotionData *cur = getMotion(current_clip);
        MotionData *tgt = getMotion(target_clip);

        for (int i = 0; i < 10; i++)
        {
            float angle_cur = cur->amplitudes[i] * std::sin(time * 2.0f * 3.14159f + cur->phases[i]);
            float angle_tgt = tgt->amplitudes[i] * std::sin(time * 2.0f * 3.14159f + tgt->phases[i]);

            float angle = (1.0f - blend_factor) * angle_cur + blend_factor * angle_tgt;

            float half = angle * 0.5f;
            float s = std::sin(half);
            float c = std::cos(half);

            switch (i)
            {
            case ARK_JOINT_UPPERARM_L:
            case ARK_JOINT_UPPERARM_R:
                out_quats[i] = {s, 0, 0, c};
                break;
            case ARK_JOINT_LOWERARM_L:
            case ARK_JOINT_LOWERARM_R:
                out_quats[i] = {0, 0, s, c};
                break;
            case ARK_JOINT_THIGH_L:
            case ARK_JOINT_THIGH_R:
                out_quats[i] = {s, 0, 0, c};
                break;
            case ARK_JOINT_CALF_L:
            case ARK_JOINT_CALF_R:
                out_quats[i] = {0, 0, s, c};
                break;
            case ARK_JOINT_FOOT_L:
            case ARK_JOINT_FOOT_R:
                out_quats[i] = {0, s, 0, c};
                break;
            default:
                out_quats[i] = {0, 0, 0, 1};
            }
        }
    }

private:
    MotionData *getMotion(int clip)
    {
        switch (clip)
        {
        case 0:
            return &walk;
        case 1:
            return &push;
        case 2:
            return &climb;
        default:
            return &walk;
        }
    }
};
