#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "filament_change.h"

#define M_PI       3.14159265358979323846   // pi

//static const int m_max_flush_volume = 750.f;
static const int g_min_flush_volume_from_support = 420.f;
static const int g_flush_volume_to_support = 230;
static const float g_min_flush_multiplier = 0.f;
static const float g_max_flush_multiplier = 3.f;
//const int m_min_flush_volume = 107;
// const int m_max_flush_volume = 800;
const float m_max_flush_volume = 800;

static float to_radians(float degree)
{
    return degree / 180.f * M_PI;
}

static float DeltaHS_BBS(float h1, float s1, float v1, float h2, float s2, float v2)
{
    float h1_rad = to_radians(h1);
    float h2_rad = to_radians(h2);

    float dx = cos(h1_rad) * s1 * v1 - cos(h2_rad) * s2 * v2;
    float dy = sin(h1_rad) * s1 * v1 - sin(h2_rad) * s2 * v2;
    float dxy = sqrt(dx * dx + dy * dy);
    return fmin(1.2f, dxy);
}


// The input r, g, b values should be in range [0, 1]. The output h is in range [0, 360], s is in range [0, 1] and v is in range [0, 1].
static void RGB2HSV(float r, float g, float b, float* h, float* s, float* v)
{
    float Cmax = fmax(fmax(r, g), b);
    float Cmin = fmin(fmin(r, g), b);
    float delta = Cmax - Cmin;

    if (fabs(delta) < 0.001) {
        *h = 0.f;
    }
    else if (Cmax == r) {
        *h = 60.f * fmod((g - b) / delta, 6.f);
    }
    else if (Cmax == g) {
        *h = 60.f * ((b - r) / delta + 2);
    }
    else {
        *h = 60.f * ((r - g) / delta + 4);
    }

    if (fabs(Cmax) < 0.001) {
        *s = 0.f;
    }
    else {
        *s = delta / Cmax;
    }

    *v = Cmax;
}

static float get_luminance(float r, float g, float b)
{
    return r * 0.3 + g * 0.59 + b * 0.11;
}

static float calc_triangle_3rd_edge(float edge_a, float edge_b, float degree_ab)
{
    return sqrt(edge_a * edge_a + edge_b * edge_b - 2 * edge_a * edge_b * cos(to_radians(degree_ab)));
}

static int calc_flushing_volume(const rgb_t from, const rgb_t to,float extra_flush_volume)
{
    float from_hsv_h, from_hsv_s, from_hsv_v;
    float to_hsv_h, to_hsv_s, to_hsv_v;

    // Calculate color distance in HSV color space
    RGB2HSV((float)from.r / 255.f, (float)from.g / 255.f, (float)from.b / 255.f, &from_hsv_h, &from_hsv_s, &from_hsv_v);
    RGB2HSV((float)to.r / 255.f, (float)to.g / 255.f, (float)to.b / 255.f, &to_hsv_h, &to_hsv_s, &to_hsv_v);
    float hs_dist = DeltaHS_BBS(from_hsv_h, from_hsv_s, from_hsv_v, to_hsv_h, to_hsv_s, to_hsv_v);

    // 1. Color difference is more obvious if the dest color has high luminance
    // 2. Color difference is more obvious if the source color has low luminance
    float from_lumi = get_luminance((float)from.r / 255.f, (float)from.g / 255.f, (float)from.b / 255.f);
    float to_lumi = get_luminance((float)to.r / 255.f, (float)to.g / 255.f, (float)to.b / 255.f);
    float lumi_flush = 0.f;
    if (to_lumi >= from_lumi) {
        lumi_flush = pow(to_lumi - from_lumi, 0.7f) * 560.f;
    }
    else {
        lumi_flush = (from_lumi - to_lumi) * 80.f;

        float inter_hsv_v = 0.67 * to_hsv_v + 0.33 * from_hsv_v;
        hs_dist = fmin(inter_hsv_v, hs_dist);
    }
    float hs_flush = 230.f * hs_dist;

    float flush_volume = calc_triangle_3rd_edge(hs_flush, lumi_flush, 120.f);
    flush_volume = fmax(flush_volume, 60.f);

    //float flush_multiplier = atof(m_flush_multiplier_ebox->GetValue().c_str());
    flush_volume += extra_flush_volume;
    return fmin((int)flush_volume, m_max_flush_volume);
}

/**
 * @description: 
 * @return {*}
 * @param {rgb_t} source
 * @param {rgb_t} target
 */
int get_flushing_volume(const rgb_t source, const rgb_t target)
{
    return calc_flushing_volume(source, target, 0);
}

int main()
{
    rgb_t from = {.r = 0, .g = 0, .b = 0};
    rgb_t to = {.r = 225, .g = 55, .b = 180};
    int volume = get_flushing_volume(from, to);
    printf("volume = %d\n", volume);
}

