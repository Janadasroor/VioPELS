/* Minimal FMI 2.0 Co-Simulation FMU for import tests: bang-bang controller.
 *
 * Variables: vout (Real input), vref (Real parameter), err (Real output),
 * gate (Boolean output). doStep latches gate = (vout < vref); err tracks
 * vref - vout. No resources, no events, deterministic. Compiled with the
 * surrounding C++ toolchain (C subset only) so CI needs no extra setup.
 */
#include <stddef.h>
#include <stdlib.h>

#include "fmi/fmi2Functions.h"

typedef struct {
  double vout;
  double vref;
  double err;
  int gate;
} Stub;

const char* fmi2GetTypesPlatform(void) { return fmi2TypesPlatform; }
const char* fmi2GetVersion(void) { return fmi2Version; }

fmi2Status fmi2SetDebugLogging(fmi2Component, fmi2Boolean, size_t,
                               const fmi2String[]) {
  return fmi2OK;
}

fmi2Component fmi2Instantiate(fmi2String, fmi2Type fmuType, fmi2String,
                              fmi2String, const fmi2CallbackFunctions*,
                              fmi2Boolean, fmi2Boolean) {
  if (fmuType != fmi2CoSimulation) return NULL;
  Stub* s = (Stub*)calloc(1, sizeof(Stub));
  if (s) {
    s->vout = 0.0;
    s->vref = 6.0; /* matches modelDescription.xml start */
    s->err = 6.0;
    s->gate = 1;
  }
  return (fmi2Component)s;
}

void fmi2FreeInstance(fmi2Component c) { free(c); }

fmi2Status fmi2SetupExperiment(fmi2Component c, fmi2Boolean,
                               fmi2Real, fmi2Real, fmi2Boolean, fmi2Real) {
  return c ? fmi2OK : fmi2Error;
}

fmi2Status fmi2EnterInitializationMode(fmi2Component c) {
  return c ? fmi2OK : fmi2Error;
}

fmi2Status fmi2ExitInitializationMode(fmi2Component c) {
  if (!c) return fmi2Error;
  Stub* s = (Stub*)c;
  s->err = s->vref - s->vout;
  s->gate = (s->vout < s->vref) ? 1 : 0;
  return fmi2OK;
}

fmi2Status fmi2Terminate(fmi2Component c) { return c ? fmi2OK : fmi2Error; }

/* VR map: 0=vout(R,in) 1=vref(R,param) 2=err(R,out) 3=gate(B,out) */
static fmi2Status setReal(Stub* s, const fmi2ValueReference vr[], size_t n,
                          const fmi2Real v[]) {
  size_t i;
  for (i = 0; i < n; ++i) {
    if (vr[i] == 0)
      s->vout = v[i];
    else if (vr[i] == 1)
      s->vref = v[i];
    else
      return fmi2Error;
  }
  return fmi2OK;
}

static fmi2Status getReal(Stub* s, const fmi2ValueReference vr[], size_t n,
                          fmi2Real v[]) {
  size_t i;
  for (i = 0; i < n; ++i) {
    if (vr[i] == 0)
      v[i] = s->vout;
    else if (vr[i] == 1)
      v[i] = s->vref;
    else if (vr[i] == 2)
      v[i] = s->err;
    else
      return fmi2Error;
  }
  return fmi2OK;
}

fmi2Status fmi2SetReal(fmi2Component c, const fmi2ValueReference vr[],
                       size_t n, const fmi2Real v[]) {
  return c ? setReal((Stub*)c, vr, n, v) : fmi2Error;
}
fmi2Status fmi2GetReal(fmi2Component c, const fmi2ValueReference vr[],
                       size_t n, fmi2Real v[]) {
  return c ? getReal((Stub*)c, vr, n, v) : fmi2Error;
}
fmi2Status fmi2SetInteger(fmi2Component c, const fmi2ValueReference[],
                          size_t, const fmi2Integer[]) {
  return c ? fmi2Error : fmi2Error;
}
fmi2Status fmi2GetInteger(fmi2Component c, const fmi2ValueReference[],
                          size_t, fmi2Integer[]) {
  return c ? fmi2Error : fmi2Error;
}
fmi2Status fmi2SetBoolean(fmi2Component c, const fmi2ValueReference[],
                          size_t, const fmi2Boolean[]) {
  return c ? fmi2Error : fmi2Error;
}
static fmi2Status getBool(Stub* s, const fmi2ValueReference vr[], size_t n,
                          fmi2Boolean v[]) {
  size_t i;
  for (i = 0; i < n; ++i) {
    if (vr[i] == 3)
      v[i] = s->gate ? fmi2True : fmi2False;
    else
      return fmi2Error;
  }
  return fmi2OK;
}
fmi2Status fmi2GetBoolean(fmi2Component c, const fmi2ValueReference vr[],
                          size_t n, fmi2Boolean v[]) {
  return c ? getBool((Stub*)c, vr, n, v) : fmi2Error;
}
fmi2Status fmi2SetString(fmi2Component c, const fmi2ValueReference[],
                         size_t, const fmi2String[]) {
  return c ? fmi2Error : fmi2Error;
}
fmi2Status fmi2GetString(fmi2Component c, const fmi2ValueReference[],
                         size_t, fmi2String[]) {
  return c ? fmi2Error : fmi2Error;
}

fmi2Status fmi2DoStep(fmi2Component c, fmi2Real, fmi2Real stepSize,
                      fmi2Boolean) {
  Stub* s;
  if (!c) return fmi2Error;
  if (!(stepSize > 0.0)) return fmi2Error;
  s = (Stub*)c;
  s->err = s->vref - s->vout;
  s->gate = (s->vout < s->vref) ? 1 : 0;
  return fmi2OK;
}

fmi2Status fmi2CancelStep(fmi2Component c) { return c ? fmi2OK : fmi2Error; }
