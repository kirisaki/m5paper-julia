#include <M5Unified.h>
#include <M5GFX.h>
#include <math.h>

// ---------------- Parameter bundle ----------------
struct RenderParams
{
  // Fractal parameters
  float c_re = -0.8f;
  float c_im = 0.156f;
  int max_iter = 120;
  float escape_radius = 2.0f;

  // View transform
  float center_x = 0.0f;
  float center_y = 0.0f;
  float scale = 240.0f; // larger = zoomed out

  // Display / quality
  int strip_h = 16;       // sprite band height
  float gamma = 0.85f;    // <1 brighter, >1 darker
  bool show_time = false; // overlay render time
  m5gfx::epd_mode_t progressive_mode = m5gfx::epd_mode_t::epd_fast;
  m5gfx::epd_mode_t final_mode = m5gfx::epd_mode_t::epd_quality;
};

// ---------------- Utilities ----------------
static inline uint16_t gray16(uint8_t g)
{
  // Convert 8-bit grayscale to RGB565
  return ((g & 0xF8) << 8) | ((g & 0xFC) << 3) | (g >> 3);
}

// Gamma LUT with simple caching
static uint8_t GAMMA_LUT[256];
static float LAST_GAMMA = -1.0f;
static void ensure_gamma_lut(float gamma)
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

// ---------------- Rendering ----------------
static void renderJulia(const RenderParams &rp)
{
  const int W = M5.Display.width();
  const int H = M5.Display.height();

  ensure_gamma_lut(rp.gamma);

  M5.Display.setEpdMode(rp.progressive_mode);
  M5.Display.clear();

  const float esc2 = rp.escape_radius * rp.escape_radius;

  lgfx::LGFX_Sprite spr(&M5.Display);
  spr.setColorDepth(16);
  spr.createSprite(W, rp.strip_h);

  uint32_t t0 = millis();

  for (int y0 = 0; y0 < H; y0 += rp.strip_h)
  {
    int h = std::min(rp.strip_h, H - y0);
    spr.clear();

    for (int yy = 0; yy < h; ++yy)
    {
      int py = y0 + yy;
      for (int px = 0; px < W; ++px)
      {
        // Map pixel -> complex plane
        float zx = rp.center_x + (px - W * 0.5f) / rp.scale;
        float zy = rp.center_y + (py - H * 0.5f) / rp.scale;

        // Iterate z <- z^2 + c
        int iter = 0;
        while (iter < rp.max_iter)
        {
          float zx2 = zx * zx - zy * zy + rp.c_re;
          float zy2 = 2.0f * zx * zy + rp.c_im;
          zx = zx2;
          zy = zy2;
          if (zx * zx + zy * zy > esc2)
            break;
          ++iter;
        }

        // Iteration -> grayscale with smoothing
        uint8_t g;
        if (iter >= rp.max_iter)
        {
          g = 0;
        }
        else
        {
          float zn = sqrtf(zx * zx + zy * zy);
          float nu = log2f(logf(zn) / logf(rp.escape_radius));
          float t = (iter + 1.0f - nu) / rp.max_iter; // 0..1
          if (t < 0)
            t = 0;
          if (t > 1)
            t = 1;
          g = (uint8_t)(t * 255.0f);
        }

        // Gamma correction & draw
        g = GAMMA_LUT[g];
        spr.drawPixel(px, yy, gray16(g));
      }
    }
    spr.pushSprite(0, y0);
    M5.Display.display(); // progressive refresh
  }

  uint32_t elapsed_ms = millis() - t0;

  // Final high-quality refresh
  M5.Display.setEpdMode(rp.final_mode);
  M5.Display.display();

  if (rp.show_time)
  {
    // Overlay at bottom-left
    const int pad = 4;
    const int box_h = 24;
    const int box_w = 140;
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

// ---------------- Runtime params & UI ----------------
RenderParams rp; // global params for runtime edits

void setup()
{
  auto cfg = M5.config();
  M5.begin(cfg);

  // Screen orientation as requested
  M5.Display.setRotation(3);

  // Initial render
  renderJulia(rp);
}

void loop()
{
  M5.update();

  // BtnA: toggle render-time overlay
  if (M5.BtnB.wasPressed())
  {
    rp.show_time = !rp.show_time;
    renderJulia(rp);
  }

  // BtnB: zoom in
  if (M5.BtnA.wasPressed())
  {
    rp.scale *= 1.2f;
    renderJulia(rp);
  }

  // BtnC: zoom out
  if (M5.BtnC.wasPressed())
  {
    rp.scale /= 1.2f;
    renderJulia(rp);
  }
}
