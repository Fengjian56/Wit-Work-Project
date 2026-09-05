/* foc_app.h - application orchestration on top of the HAL skeleton
 *
 * FocApp_Init() is called once after all MX_*_Init() peripheral setup.
 * FocApp_Loop() runs forever from main() and generates test targets plus
 * telemetry.  The module owns PID tuning, open-loop alignment and mode
 * selection so that CubeMX generated files stay thin.
 */
#ifndef FOC_APP_H
#define FOC_APP_H

void FocApp_Init(void);
void FocApp_Tick(void);

#endif
