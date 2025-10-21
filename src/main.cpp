#include <M5Unified.h>
#include <M5GFX.h>
#include <math.h>

static inline uint16_t gray16(uint8_t g)
{
  // Convert 8-bit grayscale to RGB565 format
  return ((g & 0xF8) << 8) | ((g & 0xFC) << 3) | (g >> 3);
}

// ---- Gamma correction LUT ----
static uint8_t GAMMA_LUT[256];
static void build_gamma_lut(float gamma)
{
  // Build lookup table: input 0..255 → output 0..255
  // gamma < 1 makes the image brighter, gamma > 1 makes it darker
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
}

void setup()
{
  auto cfg = M5.config();
  M5.begin(cfg);

  // Slightly brighter gamma curve
  build_gamma_lut(0.85f);

  // Initialize E-Ink display in high-quality mode
  M5.Display.setEpdMode(m5gfx::epd_mode_t::epd_quality);
  M5.Display.setRotation(1);
  M5.Display.clear();

  const int W = M5.Display.width();
  const int H = M5.Display.height();

  // ===== Julia set parameters =====
  // Complex constant c = a + bi; tweak these for different patterns
  const float c_re = -0.8f;
  const float c_im = 0.156f;

  // Complex-plane center and scale (larger scale = zoomed out)
  const float center_x = 0.0f;
  const float center_y = 0.0f;
  const float scale = 240.0f;

  const int max_iter = 120;
  const float escape_r = 2.0f;
  const float esc2 = escape_r * escape_r;

  // Draw by small horizontal strips to save memory
  const int STRIP = 16;
  lgfx::LGFX_Sprite spr(&M5.Display);
  spr.setColorDepth(16);
  spr.createSprite(W, STRIP);

  for (int y0 = 0; y0 < H; y0 += STRIP)
  {
    int h = std::min(STRIP, H - y0);
    spr.clear();

    for (int yy = 0; yy < h; ++yy)
    {
      int py = y0 + yy;
      for (int px = 0; px < W; ++px)
      {
        // Map pixel (px, py) to complex plane (zx, zy)
        float zx = center_x + (px - W * 0.5f) / scale;
        float zy = center_y + (py - H * 0.5f) / scale;

        int iter = 0;
        // Iterate z ← z² + c
        while (iter < max_iter)
        {
          float zx2 = zx * zx - zy * zy + c_re;
          float zy2 = 2.0f * zx * zy + c_im;
          zx = zx2;
          zy = zy2;
          if (zx * zx + zy * zy > esc2)
            break;
          ++iter;
        }

        // Convert iteration count to grayscale intensity
        uint8_t g;
        if (iter >= max_iter)
        {
          g = 0; // Inside the set → black
        }
        else
        {
          // Smooth coloring: reduces banding
          float zn = sqrtf(zx * zx + zy * zy);
          float nu = log2f(logf(zn) / logf(escape_r));
          float t = (iter + 1.0f - nu) / max_iter;
          if (t < 0)
            t = 0;
          if (t > 1)
            t = 1;
          g = (uint8_t)(t * 255.0f);
        }

        // Apply gamma correction
        g = GAMMA_LUT[g];
        spr.drawPixel(px, yy, gray16(g));
      }
    }

    // Push strip to the display and update E-Ink region
    spr.pushSprite(0, y0);
    M5.Display.display(); // Refresh for “drawing-in-progress” effect
  }
}

void loop()
{
  // Nothing to do — static image (E-Ink consumes almost no power)
}
