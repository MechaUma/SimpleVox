#include <M5Unified.h>

#include "simplevox_vad.h"

int soundLength;
int16_t* sound;
constexpr int kRxBufferNum = 3;
constexpr int kSampleRate = 16000;
int16_t* rxBuffer;
simplevox::VadConfig vadConfig;
simplevox::VadEngine vadEngine;

/**
 * @brief １フレームの録音を行い読み取り可能なバッファを返します
 * @return  読み取り可能な１フレーム分のバッファ
 */
int16_t* rxMic()
{
  static int rxIndex = 0;
  const int frameLength = vadConfig.frame_length();

  if (!M5.Mic.record(&rxBuffer[frameLength * rxIndex], frameLength, kSampleRate))
  {
    return nullptr;
  }
  rxIndex++;
  rxIndex = (rxIndex >= kRxBufferNum) ? 0 : rxIndex;
  return &rxBuffer[rxIndex * frameLength];
}

/**
 * @brief 指定したタイムアウトが経過するまで音声を検出する
 * @param[in]   vad       検出を行うVadEngine
 * @param[out]  dest      検出した音声の格納先
 * @param[in]   length    格納先バッファの長さ
 * @param[in]   timeoutMs タイムアウト時間、負の場合は無限
 * @return  音声を検出した場合はその長さ、未検出なら-1
 */
int pollDetect(simplevox::VadEngine& vad, int16_t* dest, int length, int timeoutMs)
{
  const unsigned long utimeoutMs = timeoutMs;
  const auto begin = millis();
  int retval = -1;
  vad.reset();
  while(retval < 0)
  {
    if (timeoutMs >= 0 && utimeoutMs < (millis() - begin))
    {
      break;
    }

    auto* data = rxMic();
    if (data == nullptr) { continue; }

    retval = vad.detect(dest, length, data);
  }
  return retval;
}

void setup() {
  vadConfig.vad_mode = simplevox::VadMode::Aggression_LV0;
  vadConfig.sample_rate = kSampleRate;
  const int frameLength = vadConfig.frame_length();

  soundLength = 3 * kSampleRate;
  sound = (int16_t*)heap_caps_malloc(sizeof(*sound) * soundLength, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
  rxBuffer = (int16_t*)heap_caps_malloc(sizeof(*rxBuffer) * kRxBufferNum * frameLength, MALLOC_CAP_8BIT);

  M5.begin();

  auto speakerConfig = M5.Speaker.config();
  speakerConfig.stereo = false;
  speakerConfig.sample_rate = kSampleRate;
  M5.Speaker.config(speakerConfig);
  M5.Speaker.setVolume(255);
  M5.Speaker.end();

  auto micConfig = M5.Mic.config();
  micConfig.stereo = false;
  micConfig.sample_rate = kSampleRate;
  M5.Mic.config(micConfig);
  if(!M5.Mic.begin())
  {
    M5.Display.println("Failed to begin microphone.");
    while(true) delay(100);
  }

  if(!vadEngine.init(vadConfig))
  {
    M5.Display.println("Failed to initialize vad.");
    while(true) delay(100);
  }

  M5.Display.printf("Sample rate: %d\n", kSampleRate);
  M5.Display.println("Start !!");
  delay(2000);
}

void loop() {
  M5.update();
#if 1
  auto* data = rxMic();
  if (data == nullptr){ return; }
  const int length = vadEngine.detect(sound, soundLength, data);
#elif 0
  const int length = pollDetect(vadEngine, sound, soundLength, -1);
#else
  M5.Display.print("*");
  const int length = pollDetect(vadEngine, sound, soundLength, 5000);
#endif
  if (length > 0)
  {
    M5.Mic.end();
    if (M5.Speaker.begin())
    {
      M5.Speaker.playRaw(sound, length, kSampleRate);
      while (M5.Speaker.isPlaying()) delay(10);
      M5.Speaker.end();
    }
    M5.Mic.begin();
    vadEngine.reset();
  }
}
