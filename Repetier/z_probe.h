#ifndef __Z_PROBEH

#define __Z_PROBEH
#include "Reptier.h"

#if defined(PROBE_PIN) && (PROBE_PIN > -1)
  void probe_4points();
  void probe_3points();
  void probe_2points();
  void probe_1point();
  void probe_status();
  float Probe_Bed(float x_pos, float y_pos,int n);
  void probe_calibrate();

#else //no probe pin

  FORCE_INLINE void probe_4points() {};
  FORCE_INLINE void probe_3points() {};
  FORCE_INLINE void probe_2points() {};
  FORCE_INLINE void probe_1point() {};
  FORCE_INLINE void probe_status() {};
  FORCE_INLINE float Probe_Bed(float x_pos, float y_pos,int n) {return 0;}
  FORCE_INLINE void probe_calibrate();
#endif //PROBE_PIN

#endif
