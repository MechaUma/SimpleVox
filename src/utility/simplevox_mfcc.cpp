/*!
 * SimpleVox
 *
 * Copyright (c) 2023 MechaUma
 *
 * This software is released under the MIT.
 * see https://opensource.org/licenses/MIT
 */

#include "simplevox_mfcc.h"

#include <math.h>
#include <memory>
#include <new>
#include <stdio.h>

#include <esp_heap_caps.h>
#include <esp_dsp.h>

namespace
{
    constexpr int kPreEmphaCoef = 100;
    constexpr int kWindowCoef = 10000;
    constexpr int kDctCoef = 10000;
    constexpr int kNormalizeCoef = 1000;
    constexpr int kDistanceCoef = 1000;

    bool VerifyMfccConfig(const simplevox::MfccConfig& config)
    {
        if (config.fft_num < 0 || !dsp_is_power_of_two(config.fft_num))
        {
            return false;
        }

        if (config.mel_channel < 0
            || config.coef_num < 0
            || config.pre_emphasis < 0
            || config.frame_time_ms < 0)
        {
            return false;
        }

        if (config.sample_rate != 8000
            && config.sample_rate != 16000)
        {
            return false;
        }

        if (config.frame_length() > config.fft_num)
        {
            return false;
        }

        return true;
    }

    bool SetupHammingWindow(int16_t* window, int length)
    {
        if (window == nullptr)
        {
            return false;
        }

        for (int i = 0; i < length; i++)
        {
            window[i] = roundf(kWindowCoef * (0.54f - 0.46f * cosf(2 * M_PI * i / (length - 1))));
        }
        return true;
    }

    float HzToMel(float freq)
    {
        return 2595.0f * logf(freq / 700.0f + 1.0f);
    }

    float MelToHz(float mel_freq)
    {
        return 700.0f * (expf(mel_freq / 2595.0f) - 1.0f);
    }

    /**
     * @brief Mel-Filterの三角波の始点と終点を設定する
     * @details
     * Mel-Filterを構成する三角波は
     * ある三角波の中心位置から終端位置までの距離と
     * 次の三角波の開始位置から中心位置までの距離が一致するようにできている。
     * したがって、構成する三角波の中心位置が分かればその傾きは算出できる。
     * 本関数では始点(0)、三角波の中心位置、終点(fft_num/2)の位置情報を設定する。
     * @param[out]  position        設定先
     * @param[in]   sample_rate   サンプリングレート
     * @param[in]   fft_num         FFTのデータ点数
     * @param[in]   channel_num     メルチャンネル数
     * @retval  true    設定成功
     * @retval  false   設定失敗
     */
    bool SetupMelFilter(int16_t *position, int sample_rate, int fft_num, int channel_num)
    {
        if (position == nullptr) { return false; }
        const int fn = sample_rate / 2;
        const float mel_fn = HzToMel(fn);
        const float delta_mel = mel_fn / (channel_num + 1);
        const float delta_freq = (float)sample_rate / fft_num;
        const int end = fft_num / 2;

        for (int i = 1; i <= channel_num; i++)
        {
            const float center_mel = i * delta_mel;
            const float center_freq = MelToHz(center_mel);
            position[i] = roundf(center_freq / delta_freq);
        }
        position[0] = 0;
        position[channel_num + 1] = end;

        return true;
    }

    bool SetupDctTable(int16_t* dct_table, int coef_num, int mel_channel)
    {
        if (dct_table == nullptr) { return false; }
        for (int i = 0; i < coef_num; i++)
        {
            for (int j = 0; j < mel_channel; j++)
            {
                dct_table[i * mel_channel + j] = roundf(kDctCoef * cosf(M_PI / mel_channel * (j + 0.5f) * (i + 1)));   // (i + 1) … i = 0は直流成分なので省く
            }
        }
        return true;
    }

    void ApplyMelFilter(const float *src, const int16_t *mel_position, int channel_num, float *dest)
    {
        const int posEnd = channel_num + 2;

        for (int i = 1; i <= channel_num; i++)
        {
            const float increment = 1.0f / (mel_position[i] - mel_position[i - 1]);
            float coef = 0;
            dest[i - 1] = 0;
            for (int j = mel_position[i - 1]; j < mel_position[i]; j++)
            {
                coef += increment;
                dest[i - 1] += coef * src[j];
            }
            const float decrement = 1.0f / (mel_position[i + 1] - mel_position[i]);
            for (int j = mel_position[i]; j < mel_position[i + 1]; j++)
            {
                coef -= decrement;
                dest[i - 1] += coef * src[j];
            }
        }
    }

    enum class MfccTag: uint8_t
    {
        VERSION1 = 1,
    };
}


namespace simplevox
{
    MfccFeature::MfccFeature(int frame_num, int coef_num): frame_num_(frame_num), coef_num_(coef_num)
    {
        feature_ = (int16_t*)heap_caps_malloc(sizeof(*feature_) * frame_num_ * coef_num_, MALLOC_CAP_8BIT);
    }

    MfccFeature::~MfccFeature()
    {
        if (feature_ != NULL)
        {
            heap_caps_free(feature_);
            feature_ = NULL;
        }
    }

    void MfccEngine::release()
    {
        fft_data_.reset();
        mel_data_.reset();
        dctII_table_.reset();
        mel_position_.reset();
        window_.reset();
    }

    bool MfccEngine::init(const MfccConfig &config)
    {
        if (!VerifyMfccConfig(config))
        {
            printf("Argument error\n");
            return false;
        }

        window_.reset(new (std::nothrow) int16_t[config.frame_length()]);
        if (!SetupHammingWindow(window_.get(), config.frame_length()))
        {
            printf("Setup window error\n");
            release();
            return false;
        }

        mel_position_.reset(new (std::nothrow) int16_t[config.mel_channel + 2]);
        if (!SetupMelFilter(mel_position_.get(), config.sample_rate, config.fft_num, config.mel_channel))
        {
            printf("Setup MelFilter error\n");
            release();
            return false;
        }

        dctII_table_.reset(new (std::nothrow) int16_t[config.coef_num * config.mel_channel]);
        if (!SetupDctTable(dctII_table_.get(), config.coef_num, config.mel_channel))
        {
            printf("Setup DctTable error\n");
            release();
            return false;
        }

        mel_data_.reset(new (std::nothrow) float[config.mel_channel]);
        if (!mel_data_)
        {
            printf("Failed to create heap\n");
            release();
            return false;
        }
     
        fft_data_.reset(new (std::nothrow) float[config.fft_num]);
        if (!fft_data_)
        {
            printf("Failed to create heap\n");
            release();
            return false;
        }

        if (dsps_fft4r_initialized != 0)
        {
            printf("DSP is already initialized\n");
            release();
            return false;
        }
        if (dsps_fft4r_init_fc32(NULL, config.fft_num / 2) != ESP_OK)
        {
            printf("DSP init error\n");
            release();
            return false;
        }

        mfcc_config_ = config;
        return true;
    }

    void MfccEngine::deinit()
    {
        if (dctII_table_ == nullptr) { return; }
        dsps_fft4r_deinit_fc32();
        release();
    }

    void MfccEngine::calculate(const int16_t* frame, float* mfcc)
    {
        const int frame_length = mfcc_config_.frame_length();
        const int fft_num = mfcc_config_.fft_num;
        {
            const int pre_emphasis = mfcc_config_.pre_emphasis;
            int prev_val = 0;
            for (int i = 0; i < frame_length; i++)
            {
                const int curt_val = frame[i];
                const float pre_emphasised = curt_val - pre_emphasis * prev_val / kPreEmphaCoef;
                fft_data_[i] = pre_emphasised * window_[i] / kWindowCoef;
                prev_val = curt_val;
            }
            for (int i = frame_length; i < fft_num; i++)
            {
                fft_data_[i] = 0;
            }
        }

        dsps_fft4r_fc32(fft_data_.get(), fft_num / 2);
        dsps_bit_rev4r_fc32(fft_data_.get(), fft_num / 2);
        dsps_cplx2real_fc32(fft_data_.get(), fft_num / 2);

        static int frame_count = 0;
        float* power_spectrum = fft_data_.get();
        for (int i = 0; i < fft_num / 2; i++)
        {
            const float re = fft_data_[2 * i];
            const float im = fft_data_[2 * i + 1];
            power_spectrum[i] = re*re + im*im;
        }

        const int mel_channel = mfcc_config_.mel_channel;
        float* mel_spectrum = mel_data_.get();
        ApplyMelFilter(power_spectrum, mel_position_.get(), mel_channel, mel_spectrum);

        float* logmel_spectrum = mel_spectrum;
        for (int i = 0; i < mel_channel; i++)
        {
            logmel_spectrum[i] = 10.0f * log10f(mel_spectrum[i]);
        }
        frame_count++;

        const int coef_num = mfcc_config_.coef_num;
        for (int i = 0; i < coef_num; i++)
        {
            const auto* dct = &dctII_table_[i * mel_channel];
            float mfcc_val = 0;
            for (int j = 0; j < mel_channel; j++)
            {
                mfcc_val += logmel_spectrum[j] * dct[j] / kDctCoef;
            }
            mfcc[i] = mfcc_val;
        }
    }

    void MfccEngine::normalize(const float* src, int frame_num, int coef_num, int16_t* dest)
    {
        float sum_val = 0;
        for (int i = 0; i < frame_num; i++)
        {
            for (int j = 0; j < coef_num; j++)
            {
                sum_val += src[i * coef_num + j];
            }
        }
        const float mean_val = sum_val / (frame_num * coef_num);

        sum_val = 0;
        for (int i = 0; i < frame_num; i++)
        {
            for (int j = 0; j < coef_num; j++)
            {
                const float value = src[i * coef_num + j] - mean_val;
                sum_val += (value * value);
            }
        }
        // 値がすべて等しい場合は０、０除算回避のため１とする
        const float stddev = (fabsf(sum_val) < __FLT_EPSILON__)
                            ? 1
                            : sqrtf(sum_val / (frame_num * coef_num));
        for (int i = 0; i < frame_num; i++)
        {
            for (int j = 0; j < coef_num; j++)
            {
                const float normalized_val = kNormalizeCoef * (src[i * coef_num + j] - mean_val) / stddev;
                if (normalized_val < INT16_MIN)
                {
                    dest[i * coef_num + j] = INT16_MIN;
                }
                else if (INT16_MAX < normalized_val)
                {
                    dest[i * coef_num + j] = INT16_MAX;
                }
                else
                {
                    dest[i * coef_num + j] = normalized_val;
                }
            }
        }
    }

    bool MfccEngine::saveFile(const char *path, const MfccFeature &mfcc)
    {
        auto* file = fopen(path, "wb");
        if (file == NULL) { return false; }

        const auto tag = static_cast<uint8_t>(MfccTag::VERSION1);
        if (fwrite(&tag, sizeof(tag), 1, file) != 1)
        {
            fclose(file); return false;
        }

        const int32_t size = mfcc.size();
        if (fwrite(&size, sizeof(size), 1, file) != 1)
        {
            fclose(file); return false;
        }
        const int32_t coef_num = mfcc.dimension();
        if (fwrite(&coef_num, sizeof(coef_num), 1, file) != 1)
        {
            fclose(file); return false;
        }

        const auto data_byte = sizeof(*mfcc.feature_);
        const int data_num = size * coef_num;
        if (fwrite(mfcc.feature_, data_byte, data_num, file) != data_num)
        {
            fclose(file); return false;
        }

        fclose(file);
        return true;
    }

    MfccFeature *MfccEngine::loadFile(const char *path)
    {
        auto* file = fopen(path, "rb");
        if (file == NULL) { return nullptr; }

        MfccTag tag;
        if (fread(&tag, sizeof(tag), 1, file) != 1)
        {
            fclose(file); return nullptr;
        }

        int32_t size, coef_num;
        if (fread(&size, sizeof(size), 1, file) != 1)
        {
            fclose(file); return nullptr;
        }
        if (fread(&coef_num, sizeof(coef_num), 1, file) != 1)
        {
            fclose(file); return nullptr;
        }

        MfccFeature* mfcc = new MfccFeature(size, coef_num);
        if (mfcc == nullptr || mfcc->feature_ == nullptr)
        {
            fclose(file); return nullptr;
        }

        const auto data_byte = sizeof(*mfcc->feature_);
        const int data_num = size * coef_num;
        if (fread(mfcc->feature_, data_byte, data_num, file) != data_num)
        {
            delete mfcc;
            fclose(file); return nullptr;
        }

        fclose(file);
        return mfcc;
    }

    MfccFeature *MfccEngine::create(const int16_t *raw_audio, int length)
    {
        const int frame_length = mfcc_config_.frame_length();
        const int hop_length = mfcc_config_.hop_length();
        const int frame_num = (length - (frame_length - hop_length)) / hop_length;
        const int coef_num = mfcc_config_.coef_num;
        if (frame_num <= 0) { return nullptr; }

        auto* mfcc = new MfccFeature(frame_num, coef_num);
        std::unique_ptr<float[]> temp_feature(new float[frame_num * coef_num]);
        if (mfcc->feature_ == NULL || !temp_feature)
        {
            printf("Failed to create heap.\n");
            delete mfcc;
            return nullptr;
        }

        for (int fnum = 0; fnum < frame_num; fnum++)
        {
            const auto* frame = &raw_audio[fnum * hop_length];
            calculate(frame, &temp_feature[fnum * coef_num]);
        }

        normalize(temp_feature.get(), frame_num, coef_num, mfcc->feature_);
        return mfcc;
    }

    MfccFeature* MfccEngine::create(const float* mfccs, int frame_num, int coef_num)
    {
        auto* mfcc = new MfccFeature(frame_num, coef_num);
        if (mfcc->feature_ == NULL)
        {
            printf("Failed to create heap.\n");
            delete mfcc;
            return nullptr;
        }

        normalize(mfccs, frame_num, coef_num, mfcc->feature_);
        return mfcc;
    }

    

} // namespace simplevox
