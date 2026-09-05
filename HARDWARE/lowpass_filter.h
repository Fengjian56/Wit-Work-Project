/* lowpass_filter.h - first-order recursive low-pass filter */
#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

float Lowpass_Next(float raw, float prev, float alpha);

#endif
