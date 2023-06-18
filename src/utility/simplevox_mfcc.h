/*!
 * SimpleVox
 *
 * Copyright (c) 2023 MechaUma
 *
 * This software is released under the MIT.
 * see https://opensource.org/licenses/MIT
 */

#ifndef SIMPLEVOX_MFCC_H_
#define SIMPLEVOX_MFCC_H_

#include <memory>
#include <stdint.h>

#include "simplevox_feature.h"

namespace simplevox
{
    struct MfccConfig
    {
        /**
         * @brief FFTのデータ点数
         */
        int fft_num = 512;

        /**
         * @brief メルフィルタバンクのチャンネル数
         */
        int mel_channel = 24;

        /**
         * @brief MFCCの係数の数(12 -> 1~12を取り出す(0は除外))
         */
        int coef_num = 12;

        /**
         * @brief 高域成分強調のためのプリエンファシス係数[%] (97 -> 0.97)
         */
        int pre_emphasis = 97;
        
        /**
         * @brief サンプリングレート(8000Hz or 16000Hz)
         */
        int sample_rate = 16000;
        
        /**
         * @brief Time length of one frame
         * @note
         * Typical values are 20-40 ms.
         * The default value is 32 ms, which is a nice round number for FFT (= 32[ms] * 16000[Hz] = 512).
         */
        int frame_time_ms = 32;

        int frame_length() const { return frame_time_ms * sample_rate / 1000; }
        int hop_length() const { return frame_length() / 2; }
    };

    class MfccFeature: public ISoundFeature<MfccFeature>
    {
    friend class MfccEngine;
    public:
        ~MfccFeature();
        /**
         * @brief   特徴量の総数
         */
        int size() const { return frame_num_; }

        /**
         * @brief   特徴量の次元数(MFCCの係数の個数)
         */
        int dimension() const { return coef_num_; }
        
        /**
         * @brief   任意の番号の特徴量
         * @param[in] number 特徴量の番号(フレーム番号)
         */
        const int16_t* feature(int number) const { return &feature_[number * coef_num_]; }

    private:
        MfccFeature(int frame_num, int coef_num);
        int frame_num_;
        int coef_num_;
        int16_t* feature_ = nullptr;
    };

    class MfccEngine
    {
    private:
        MfccConfig mfcc_config_;
        std::unique_ptr<int16_t[]> window_;
        std::unique_ptr<int16_t[]> mel_position_;
        std::unique_ptr<int16_t[]> dctII_table_;
        std::unique_ptr<float[]> mel_data_;
        std::unique_ptr<float[]> fft_data_;
        void release();
    public:
        MfccConfig config() const { return mfcc_config_; }
        /**
         * @brief   指定したコンフィグに沿って初期化処理を行います
         * @param[in]   config  コンフィグ
         * @return 初期化成功ならtrue, 失敗ならfalse 
         */
        bool init(const MfccConfig& config);
        
        /**
         * @brief   リソースを開放します
         */
        void deinit();

        /**
         * @brief MFCCを算出します
         * @param[in]   frame   １フレーム(frame_length())分のサウンドデータ
         * @param[out]  mfcc    算出した特徴量(MFCC, coef_num個)
         */
        void calculate(const int16_t* frame, float* mfcc);

        /**
         * @brief 各フレームのMFCCを標準化します
         * @details
         * 入力のMFCCを平均０、分散１に標準化します。
         * 標準化の性質からほとんどの値は－３～３の範囲に収まることが期待されます。
         * これに合わせてdestに１０００倍した値を変換結果として格納します。
         * これは精度を満たしながら(floatに比べて)処理コストおよびサイズを削減するためです。
         * 一方で、一般的な音声信号では多くの場合問題ありませんが、
         * 極端に信号に偏りが見られる場合は値がint16_tの範囲から逸脱する場合があります。
         * それらの値はINT16_MINおよびINT16_MAXにクリップされます
         * @param[in]   src         MFCC (frame_num * coef_num)
         * @param[in]   frame_num   総フレーム数 
         * @param[in]   coef_num    係数の個数
         * @param[out]  dest        標準化後のMFCC(値は1000倍されたもの)
         */
        void normalize(const float* src, int frame_num, int coef_num, int16_t* dest);

        /**
         * @brief   MFCCをファイルに保存します
         * @param[in]   path    ファイルのパス
         * @param[in]   mfcc    保存するMFCC
         * @return  保存に成功したらtrue, そうでなければfalse
         */
        static bool saveFile(const char *path, const MfccFeature& mfcc);

        /**
         * @brief   ファイルからMFCCを読み出します
         * @param[in]   path    ファイルのパス
         * @return  読み出しに成功したらnullptr以外, 失敗したらnullptr 
         */
        static MfccFeature* loadFile(const char *path);

        /**
         * @brief   raw audio dataを基にMFCCを作成します
         * @param[in]   raw_audio   raw audio
         * @param[in]   length      raw audioの長さ
         * @return  作成に成功したらnullptr以外, 失敗したらnullptr 
         */
        MfccFeature* create(const int16_t* raw_audio, int length);

        /**
         * @brief   フレームごとの特徴量(標準化前)を基にMFCCを作成します
         * @param[in]   mfccs       MFCC (frame_num * coef_num)
         * @param[in]   frame_num   総フレーム数
         * @param[in]   coef_num    係数の個数
         * @return  作成に成功したらnullptr以外, 失敗したらnullptr 
         */
        MfccFeature* create(const float* mfccs, int frame_num, int coef_num);
    };
} // namespace simplevox



#endif