#include <M5Unified.h>
#include <M5GFX.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_heap_caps.h>

// ---------- Parameters ----------
struct RenderParams
{
  // Julia parameters
  float c_re = -0.8f, c_im = 0.156f;
  int max_iter = 120;
  float escape_radius = 2.0f;

  // View
  float center_x = 0.0f, center_y = 0.0f;
  float scale = 240.0f; // larger = zoomed out

  // Display / quality
  int strip_h = 16;    // sprite band height for drawing
  float gamma = 0.85f; // <1 brighter, >1 darker
  bool show_time = true;
  bool smooth = true; // smooth coloring on/off
  int segments = 16;  // <-- number of compute segments (task partition)
  m5gfx::epd_mode_t progressive_mode = m5gfx::epd_mode_t::epd_fast;
  m5gfx::epd_mode_t final_mode = m5gfx::epd_mode_t::epd_quality;
};

// ---------- Globals ----------
static TaskHandle_t g_mainTask = nullptr;
static uint8_t *g_frame8 = nullptr; // W*H 8-bit gray (post-gamma)
static int gW = 0, gH = 0;
static RenderParams g_rp;               // snapshot for workers
static QueueHandle_t g_workQ = nullptr; // work queue

// Gamma LUT cache
static uint8_t GAMMA_LUT[256];
static float LAST_GAMMA = -1.0f;
static inline void ensure_gamma(float gamma)
{
  if (gamma == LAST_GAMMA)
    return;
  for (int i = 0; i < 256; ++i)
  {
    float x = i / 255.0f;
    float y = powf(x, gamma);
    int v = (int)lroundf(y * 255.0f);
    if (v < 0)
      v = 0;
    if (v > 255)
      v = 255;
    GAMMA_LUT[i] = (uint8_t)v;
  }
  LAST_GAMMA = gamma;
}

static inline uint16_t gray16(uint8_t g)
{
  // 8-bit grayscale → RGB565
  return ((g & 0xF8) << 8) | ((g & 0xFC) << 3) | (g >> 3);
}

static void ensureFrameBuffer()
{
  int W = M5.Display.width();
  int H = M5.Display.height();
  if (W != gW || H != gH || g_frame8 == nullptr)
  {
    gW = W;
    gH = H;
    if (g_frame8)
    {
      heap_caps_free(g_frame8);
      g_frame8 = nullptr;
    }
    // Prefer PSRAM; fall back to internal RAM
    g_frame8 = (uint8_t *)heap_caps_malloc(gW * gH, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!g_frame8)
    {
      g_frame8 = (uint8_t *)heap_caps_malloc(gW * gH, MALLOC_CAP_8BIT);
    }
  }
}

// ---------- Work item ----------
struct WorkRange
{
  int y_begin;
  int y_end;
}; // [y_begin, y_end)

// ---------- Worker (runs on both cores; pulls from queue) ----------
static void computeWorkerQ(void *arg)
{
  (void)arg;
  const RenderParams rp = g_rp; // snapshot
  ensure_gamma(rp.gamma);

  const float esc2 = rp.escape_radius * rp.escape_radius;
  const float halfW = 0.5f * gW, halfH = 0.5f * gH;
  const float invS = 1.0f / rp.scale;

  for (;;)
  {
    WorkRange r;
    if (xQueueReceive(g_workQ, &r, portMAX_DELAY) != pdTRUE)
      continue;
    if (r.y_begin < 0)
      break; // sentinel to exit

    for (int py = r.y_begin; py < r.y_end; ++py)
    {
      float zy0 = rp.center_y + (py - halfH) * invS;
      uint8_t *row = &g_frame8[py * gW];

      for (int px = 0; px < gW; ++px)
      {
        float zx = rp.center_x + (px - halfW) * invS;
        float zy = zy0;

        int iter = 0;
        float r2 = 0.f;
        while (iter < rp.max_iter)
        {
          float zx2 = zx * zx - zy * zy + rp.c_re;
          float zy2 = 2.0f * zx * zy + rp.c_im;
          zx = zx2;
          zy = zy2;
          r2 = zx * zx + zy * zy;
          if (r2 > esc2)
            break;
          ++iter;
        }

        uint8_t g8;
        if (!rp.smooth)
        {
          g8 = (iter >= rp.max_iter) ? 0 : (uint8_t)(255.0f * iter / rp.max_iter);
        }
        else
        {
          if (iter >= rp.max_iter)
          {
            g8 = 0;
          }
          else
          {
            // Accurate smooth coloring
            float zn = sqrtf(r2);
            float nu = log2f(logf(zn) / logf(rp.escape_radius));
            float t = (iter + 1.0f - nu) / rp.max_iter; // 0..1
            if (t < 0)
              t = 0;
            if (t > 1)
              t = 1;
            g8 = (uint8_t)lroundf(t * 255.0f);
          }
        }
        row[px] = GAMMA_LUT[g8]; // post-gamma grayscale
      }
    }
  }

  // done
  xTaskNotifyGive(g_mainTask);
  vTaskDelete(nullptr);
}

// ---------- Kick two workers and wait ----------
static void computeFrameWithQueue(const RenderParams &rp)
{
  g_rp = rp; // snapshot
  // (Re)create queue sized for all segments + 2 sentinels
  int segs = rp.segments < 2 ? 2 : rp.segments;
  if (g_workQ)
  {
    vQueueDelete(g_workQ);
    g_workQ = nullptr;
  }
  g_workQ = xQueueCreate(segs + 2, sizeof(WorkRange));

  // Partition H into 'segs' chunks
  int base = gH / segs, rem = gH % segs;
  int y = 0;
  for (int i = 0; i < segs; ++i)
  {
    int h = base + (i < rem ? 1 : 0);
    WorkRange r{y, y + h};
    xQueueSend(g_workQ, &r, portMAX_DELAY);
    y += h;
  }
  // Push 2 sentinels (one per worker)
  WorkRange s{-1, -1};
  xQueueSend(g_workQ, &s, portMAX_DELAY);
  xQueueSend(g_workQ, &s, portMAX_DELAY);

  // Start two workers: one on each core
  xTaskCreatePinnedToCore(computeWorkerQ, "wrk0", 8192, nullptr, 1, nullptr, 0); // Core0
  xTaskCreatePinnedToCore(computeWorkerQ, "wrk1", 8192, nullptr, 1, nullptr, 1); // Core1

  // Wait for both workers to notify completion
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  // Clean up queue
  vQueueDelete(g_workQ);
  g_workQ = nullptr;
}

// ---------- Rendering ----------
static void renderJulia(const RenderParams &rp)
{
  const int W = M5.Display.width();
  const int H = M5.Display.height();

  ensureFrameBuffer();

  uint32_t t0 = millis();

  M5.Display.setEpdMode(rp.progressive_mode);
  M5.Display.clear();

  // ---- compute on two cores with segmented work queue ----
  computeFrameWithQueue(rp);

  // ---- draw with drawPixel exactly as before ----
  lgfx::LGFX_Sprite spr(&M5.Display);
  spr.setColorDepth(16);
  spr.createSprite(W, rp.strip_h);

  for (int y0 = 0; y0 < H; y0 += rp.strip_h)
  {
    int h = std::min(rp.strip_h, H - y0);
    spr.clear();
    for (int yy = 0; yy < h; ++yy)
    {
      int py = y0 + yy;
      const uint8_t *src = &g_frame8[py * W];
      for (int px = 0; px < W; ++px)
      {
        spr.drawPixel(px, yy, gray16(src[px]));
      }
    }
    spr.pushSprite(0, y0);
    M5.Display.display();
  }

  // Final GC16 for smooth tone
  M5.Display.setEpdMode(rp.final_mode);
  M5.Display.display();

  uint32_t elapsed_ms = millis() - t0;

  if (rp.show_time)
  {
    const int pad = 4, box_h = 24, box_w = 160;
    M5.Display.fillRect(0, H - box_h, box_w, box_h, TFT_WHITE);
    M5.Display.setTextColor(TFT_BLACK, TFT_WHITE);
    M5.Display.setTextDatum(bottom_left);
    M5.Display.setTextSize(2);
    char buf[48];
    snprintf(buf, sizeof(buf), "%lu ms", (unsigned long)elapsed_ms);
    M5.Display.drawString(buf, pad, H - pad);
    M5.Display.display();
  }
}

// ---------- UI ----------
static RenderParams rp;

void setup()
{
  g_mainTask = xTaskGetCurrentTaskHandle();

  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(3);

  renderJulia(rp);
}

void loop()
{
  M5.update();

  if (M5.BtnB.wasPressed())
  {
    rp.show_time = !rp.show_time;
    renderJulia(rp);
  }
  if (M5.BtnA.wasPressed())
  {
    rp.scale *= 1.2f;
    renderJulia(rp);
  }
  if (M5.BtnC.wasPressed())
  {
    rp.scale /= 1.2f;
    renderJulia(rp);
  }
}
