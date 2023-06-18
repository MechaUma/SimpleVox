/*!
 * SimpleVox
 *
 * Copyright (c) 2023 MechaUma
 *
 * This software is released under the MIT.
 * see https://opensource.org/licenses/MIT
 */

#ifndef SIMPLEVOX_VAD_H_
#define SIMPLEVOX_VAD_H_

#include <stdint.h>

namespace simplevox
{
    enum class VadState
    {
        None,
        Warmup,
        Setup,
        Silence,
        PreDetection,   ///< 瞬時的なノイズかそうでないかを判断中
        Speech,         ///< 音声検出中
        PostDetection,  ///< 音声の終了か瞬時的な途切れかを判断中
        Detected,
    };

    enum class VadMode
    {
        Aggression_LV0,
        Aggression_LV1,
        Aggression_LV2,
        Aggression_LV3,
        Aggression_LV4,
    };

    struct VadConfig
    {
        static const int frame_time_ms = 10;

        /**
         * @brief マイクのウォームアップ時間
         * @note マイクのハードウェア的特性などによりウォームアップが必要な場合設定する
         */
        int warmup_time_ms = 0;

        /**
         * @brief 音声区間として含まれる音声検出前の時間
         */
        int hangbefore_ms = 100;

        /**
         * @brief 音声として判断するための時間（音声ありの判定がこの時間以上続くと音声区間と判断される）
         */
        int decision_time_ms = 200;

        /**
         * @brief 音声区間として含まれる音声終了後の時間
         */
        int hangover_ms = 200;

        /**
         * @brief サンプリングレート(8000Hz or 16000Hz)
         */
        int sample_rate = 16000;

        /**
         * @brief VADの動作モード LVの上昇に合わせて検出判定が厳しくなる
         */
        VadMode vad_mode = VadMode::Aggression_LV0;

        int frame_length() const { return frame_time_ms * sample_rate / 1000; }
        int warmup_length() const { return warmup_time_ms * sample_rate / 1000; }
        int before_length() const { return hangbefore_ms * sample_rate / 1000; }
        int decision_length() const { return decision_time_ms * sample_rate / 1000; }
        int over_length() const { return hangover_ms * sample_rate / 1000; }
    };

    using esp_vad_handle_t = void*;
    class VadEngine
    {
    private:
        esp_vad_handle_t vad_inst_ = nullptr;
        VadConfig vad_config_;
        VadState vad_state_;
        int state_count_;
        int frame_count_;
        bool has_satisfied_hangbefore_;
    
    public:
        VadConfig config() { return vad_config_; }
        
        /**
         * @brief   指定したコンフィグに沿って初期化処理を行います
         * @param[in]   config  コンフィグ
         * @return 初期化成功ならtrue, 失敗ならfalse 
         */
        bool init(const VadConfig& config);

        /**
         * @brief   リソースを開放します
         */
        void deinit();

        /**
         * @brief   VADの判定状況をリセットします。
         * @details
         * 内部の状態をリセットし以降の判定に備えます。
         * 音声検出後に新たに判定を行う場合はreset()が必要です。
         */
        void reset();

        /**
         * @brief   音声区間の検出を行います
         * @param[in]   data    １フレーム(frame_length())分のサウンドデータ
         * @retval  Detected        音声検出完了
         * @retval  PostDetection   音声の終了か瞬時的な途切れかを判断中
         * @retval  Speech          音声検出中
         * @retval  PreDetection    瞬時的なノイズかそうでないかを判断中
         * @retval  Silence         無音と判断
         * @retval  Setup           検出のための準備中
         * @retval  Warmup          ハードウェアのウォームアップ中
         */
        VadState process(const int16_t* data);

        /**
         * @brief   音声区間の検出を行います
         * @param[out]  dest    検出したサウンドデータの格納先
         * @param[in]   length  格納先バッファ長 
         * @param[in]   data    １フレーム(frame_length())分のサウンドデータ
         * @retval >0   検出した音声の長さ（データ数）
         * @retval <0   音声未検出
         */
        int detect(int16_t* dest, int length, const int16_t* data);
    };

} // namespace simplevox

#endif // SIMPLEVOX_VAD_H_