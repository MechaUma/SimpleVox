#include <M5Unified.h>

#include <memory>

#include <esp_ns.h>
#include <SPIFFS.h>

#include "simplevox.h"

#define base_path "/spiffs"
#define file_name "/wakeword.bin"
constexpr int kSampleRate = 16000;
constexpr int audioLength = kSampleRate * 3;  // 3 seconds
constexpr int kRxBufferNum = 3;
int16_t* rawAudio;
int16_t* rxBuffer;
ns_handle_t nsInst;
simplevox::VadEngine vadEngine;
simplevox::MfccEngine mfccEngine;
simplevox::MfccFeature* mfcc = nullptr;

LGFX_Button regButton;
LGFX_Button cmpButton;

/**
 * @brief １フレームの録音を行い読み取り可能なバッファを返します
 * @return  読み取り可能な１フレーム分のバッファ
 */
int16_t* rxMic()
{
  static int rxIndex = 0;
  const int frameLength = vadEngine.config().frame_length();

  if (!M5.Mic.record(&rxBuffer[frameLength * rxIndex], frameLength, kSampleRate))
  {
    return nullptr;
  }
  rxIndex++;
  rxIndex = (rxIndex >= kRxBufferNum) ? 0 : rxIndex;
  return &rxBuffer[rxIndex * frameLength];
}


void setup() {
  auto vadConfig = vadEngine.config();
  vadConfig.sample_rate = kSampleRate;
  auto mfccConfig = mfccEngine.config();
  mfccConfig.sample_rate = kSampleRate;
  constexpr uint32_t memCaps = (CONFIG_SPIRAM) ? (MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM) : MALLOC_CAP_8BIT;
  rawAudio = (int16_t*)heap_caps_malloc(audioLength * sizeof(*rawAudio), memCaps);
  rxBuffer = (int16_t*)heap_caps_malloc(kRxBufferNum * vadConfig.frame_length() * sizeof(*rxBuffer), MALLOC_CAP_8BIT);

  M5.begin();
  int w = M5.Lcd.width();
  int h = M5.Lcd.height();
  regButton.initButton(&M5.Lcd, w/2 - 100, h - 50/2, 100, 50, TFT_BLACK, TFT_GREEN, TFT_BLACK, "REGIST");
  cmpButton.initButton(&M5.Lcd, w/2 + 100, h - 50/2, 100, 50, TFT_BLACK, TFT_SKYBLUE, TFT_BLACK, "COMPARE");
  regButton.drawButton();
  cmpButton.drawButton();

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
  M5.Mic.begin();

  M5.Display.println("Setup !!");

  nsInst = ns_pro_create(vadConfig.frame_time_ms, 1, vadConfig.sample_rate);
  if (nsInst == NULL)
  {
    M5.Display.println("Failed to initialize ns.");
    while(true) delay(10);
  }
  if (!vadEngine.init(vadConfig))
  {
    M5.Display.println("Failed to initialize vad.");
    while(true) delay(10);
  }
  if (!mfccEngine.init(mfccConfig))
  {
    M5.Display.println("Failed to initialize mfcc.");
    while(true) delay(10);
  }
  
  SPIFFS.begin(true);
  if (SPIFFS.exists(file_name))
  {
    M5.Display.println("File exists !!");
    mfcc = mfccEngine.loadFile(base_path file_name);
  }

  M5.Display.println("Start !!");
  delay(1000);
}

void loop() {
  static int mode = 0;   // 0: none, <0: REGIST, >0: COMPARE
  M5.update();

  if (M5.Touch.getDetail().wasPressed() && mode == 0)
  {
    const auto point = M5.Touch.getTouchPointRaw();
    if (regButton.contains(point.x, point.y) || M5.BtnA.isPressed())
    {
      mode = -1;
    }
    else if (cmpButton.contains(point.x, point.y) || M5.BtnC.isPressed())
    {
      mode = 1;
    }
  }
  if (mode == 0) { M5.Display.drawString("None   ", 0, 0); return; }
  else if (mode < 0) { M5.Display.drawString("REGIST  ", 0, 0); }
  else if (mode > 0) { M5.Display.drawString("COMPARE", 0, 0); }

  auto* data = rxMic();
  if (data == nullptr) { return; }
  ns_process(nsInst, data, data);

  int length = vadEngine.detect(rawAudio, audioLength, data);
  if (length <= 0) { return; }

  if (mode < 0) // 検出した音声を再生しMFCCを登録及びファイルに保存します
  {
    M5.Mic.end();
    if (M5.Speaker.begin())
    {
      M5.Speaker.playRaw(rawAudio, length, kSampleRate);
      while (M5.Speaker.isPlaying()) { delay(10); }
      M5.Speaker.end();
    }
    M5.Mic.begin();

    if (mfcc != nullptr){ delete mfcc; }
    mfcc = mfccEngine.create(rawAudio, length);

    if (mfcc)
    {
      mfccEngine.saveFile(base_path file_name, *mfcc);
    }
  }
  else  // 登録したMFCCと検出した音声のMFCCを比較します
  {
    if (mfcc != nullptr)
    {
      std::unique_ptr<simplevox::MfccFeature> feature(mfccEngine.create(rawAudio, length));
      const auto dist = simplevox::calcDTW(*mfcc, *feature);
      char cbuf[64];
      char pass = (dist < 180) ? '!': '?';  // 180未満で一致と判定, しきい値は要調整
      sprintf(cbuf, "Dist: %6lu, %c", dist, pass);
      M5.Display.drawString(cbuf, 0, 50);
    }
  }
  vadEngine.reset();
  mode = 0;
}
