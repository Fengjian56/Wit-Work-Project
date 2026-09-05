/* lowpass_filter.c - one-pole recursive low-pass filter
 *
 * out[n] = alpha * x[n] + (1 - alpha) * out[n-1]
 *
 * A smaller alpha gives more smoothing at the cost of more lag.
 */
#include "lowpass_filter.h"

float Lowpass_Next(float raw, float prev, float alpha)
{
  return alpha * raw + (1.0f - alpha) * prev;
}
