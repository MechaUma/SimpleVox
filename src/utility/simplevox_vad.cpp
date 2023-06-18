/*!
 * SimpleVox
 *
 * Copyright (c) 2023 MechaUma
 *
 * This software is released under the MIT.
 * see https://opensource.org/licenses/MIT
 */

#include "simplevox_vad.h"

#include <algorithm>
#include <atomic>
#include <new>

#include <esp_vad.h>

namespace
{
    int DivCeil(int dividend, int divisor)
    {
        return (dividend + divisor - 1) / divisor;
    }
}

namespace simplevox
{
    bool VadEngine::init(const VadConfig &config)
    {
        if (vad_inst_ != nullptr)
        {
            return false;
        }

        if (config.frame_time_ms != 10)
        {
            return false;
        }

        if (config.sample_rate != 8000 && config.sample_rate != 16000)
        {
            return false;
        }

        if (config.hangbefore_ms < 0 || config.decision_time_ms < 0 || config.hangover_ms < 0)
        {
            return false;
        }

        switch (config.vad_mode)
        {
        case VadMode::Aggression_LV0:
            vad_inst_ = vad_create(VAD_MODE_0);
            break;
        case VadMode::Aggression_LV1:
            vad_inst_ = vad_create(VAD_MODE_1);
            break;
        case VadMode::Aggression_LV2:
            vad_inst_ = vad_create(VAD_MODE_2);
            break;
        case VadMode::Aggression_LV3:
            vad_inst_ = vad_create(VAD_MODE_3);
            break;
        case VadMode::Aggression_LV4:
            vad_inst_ = vad_create(VAD_MODE_4);
            break;
        default:
            vad_inst_ = nullptr;
            break;
        }

        if (vad_inst_ == nullptr)
        {
            return false;
        }

        reset();
        vad_config_ = config;
        return true;
    }

    void VadEngine::deinit()
    {
        vad_destroy(vad_inst_);
        vad_inst_ = nullptr;
    }

    void VadEngine::reset()
    {
        this->frame_count_ = 0;
        this->state_count_ = 0;
        this->has_satisfied_hangbefore_ = false;
        this->vad_state_ = VadState::Warmup;
    }

    VadState VadEngine::process(const int16_t* data)
    {
        const auto& config = vad_config_;
        const int frame_length = config.frame_length();

        state_count_++;
        const int state_length = frame_length * state_count_;
        const bool IsSpeech = (has_satisfied_hangbefore_)
                            ? VAD_SPEECH == vad_process(vad_inst_, (int16_t*)data, config.sample_rate, config.frame_time_ms)
                            : false;
        switch (vad_state_)
        {
        case VadState::Warmup:
            if (state_length >= config.warmup_length())
            {
                state_count_ = 0;
                vad_state_ = VadState::Setup;
            }
            break;
        case VadState::Setup:
            state_count_ = 0;
            vad_state_ = VadState::Silence;
            break;
        case VadState::Silence:
            if (!has_satisfied_hangbefore_)
            {
                frame_count_++;
                if (state_length >= config.before_length())
                {
                    has_satisfied_hangbefore_ = true;
                }
                break;
            }
            
            if (IsSpeech)
            {
                state_count_ = 0;
                frame_count_++;
                vad_state_ = VadState::PreDetection;
            }
            break;
        case VadState::PreDetection:
            if (IsSpeech)
            {
                const int pass_count = DivCeil(config.decision_length(), frame_length);
                frame_count_++;
                if (state_count_ >= pass_count)
                {
                    state_count_ = 0;
                    vad_state_ = VadState::Speech;
                }
            }
            else
            {
                frame_count_ -= state_count_;
                state_count_ = 0;
                vad_state_ = VadState::Silence;
            }
            break;
        case VadState::Speech:
            frame_count_++;
            if (!IsSpeech)
            {
                state_count_ = 0;
                vad_state_ = VadState::PostDetection;
            }
            break;
        case VadState::PostDetection:
            frame_count_++;
            if (IsSpeech)
            {
                state_count_ = 0;
                vad_state_ = VadState::Speech;
            }
            else
            {
                const int over_count = DivCeil(config.over_length(), frame_length);
                if (state_count_ >= over_count)
                {
                    state_count_ = 0;
                    vad_state_ = VadState::Detected;
                }
            }
            break;
        case VadState::Detected:
            /* NOP */
            break;
        default:
            state_count_ = 0;
            frame_count_ = 0;
            vad_state_ = VadState::Warmup;
            break;
        }
        return vad_state_;
    }

    int VadEngine::detect(int16_t* dest, int length, const int16_t* data)
    {
        const auto frame_length = vad_config_.frame_length();
        const auto sound_length = frame_length * frame_count_;

        if (vad_state_ == VadState::Detected)
        {
            return sound_length;
        }
        else if (length < sound_length + frame_length)
        {
            return (vad_state_ >= VadState::Speech)
                    ? sound_length
                    : -1;
        }
        else
        {
            const auto prev_frame_count = frame_count_;
            const auto state = process(data);
            
            if (prev_frame_count + 1 == frame_count_)
            {
                std::copy_n(data, frame_length, &dest[sound_length]);
            }
            else if (state == VadState::Silence
                    && prev_frame_count >= frame_count_)
            {
                const int shift_count = prev_frame_count - frame_count_ + 1;
                const int shift_length = frame_length * shift_count;
                if (sound_length > shift_length)    // Setup -> Silenceの場合 sound_length < shift_length
                {
                    std::copy_n(&dest[shift_length], sound_length - shift_length, dest);
                    std::copy_n(data, frame_length, &dest[sound_length - shift_length]);
                }
            }

            return (state == VadState::Detected)
                    ? frame_length * frame_count_
                    : -1;
        }
    }

} // namespace simplevox
