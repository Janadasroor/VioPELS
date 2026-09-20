#pragma once
#include <cstddef>
#include <string>
#include <vector>

#include "power_engine/device.h"

namespace power_engine {

/// Circuit graph: collection of two-terminal devices + node set.
/// Nodes are ints, 0 is always ground. Phase 2: + Switch/Diode.
class Circuit {
 public:
  Circuit() = default;

  void addResistor(const std::string& name, int n1, int n2, double r);
  void addCapacitor(const std::string& name, int n1, int n2, double c,
                    double vc0 = 0.0);
  void addInductor(const std::string& name, int n1, int n2, double l,
                   double il0 = 0.0);
  void addVoltageSource(const std::string& name, int np, int nm, double v);
  void addCurrentSource(const std::string& name, int np, int nm, double i);
  /// Ideal switch between n1-n2. closed=false => Roff, true => Ron.
  /// eon/eoff: switching energy [J] dissipated on turn-on/off edges.
  /// ttail/tailk: exponential turn-off tail (Itail0 = tailk*Ioff), 0 = none.
  void addSwitch(const std::string& name, int n1, int n2, double ron = 5e-3,
                 double roff = 1e6, bool closed = false, double eon = 0.0,
                 double eoff = 0.0, double ttail = 0.0, double tailk = 0.1);
  /// Ideal diode, n1=anode, n2=cathode. Starts blocking.
  /// qrr/trr: triangular reverse recovery (Irr = 2*Qrr/trr); 0 = ideal.
  void addDiode(const std::string& name, int anode, int cathode,
                double vf = 0.0, double ron = 10e-3, double roff = 1e6,
                double qrr = 0.0, double trr = 0.0);
  /// Ideal transformer: primary (np1,nm1), secondary (np2,nm2),
  /// Vp/Vs = ratio, ratio*Ip + Is = 0. Algebraic (no magnetics).
  void addTransformer(const std::string& name, int np1, int nm1, int np2, int nm2,
                      double ratio = 1.0);
  /// Coupled inductors: winding 1 (n1a,n1b, L1), winding 2 (n2a,n2b, L2),
  /// dots at n1a and n2a, coupling 0 < k < 1 (M = k*sqrt(L1*L2)).
  /// k = 1 is rejected (singular companion); use Transformer instead.
  void addCoupledInductors(const std::string& name, int n1a, int n1b, int n2a, int n2b,
                           double l1, double l2, double k, double il10 = 0.0,
                           double il20 = 0.0);
  /// Saturable inductor: λ(i) = Lsat*i + (Lunsat-Lsat)*Isat*tanh(i/Isat).
  /// Requires 0 < Lsat <= Lunsat and Isat > 0.
  void addSaturableInductor(const std::string& name, int n1, int n2, double lunsat,
                            double lsat, double isat, double il0 = 0.0);

  /// Gate control for switches. Throws if name is not a Switch.
  void setSwitch(const std::string& name, bool closed);
  bool switchClosed(const std::string& name) const;
  bool diodeConducting(const std::string& name) const;

  const std::vector<Device>& devices() const { return devices_; }
  std::vector<Device>& mutableDevices() { return devices_; }

  const Device& findDevice(const std::string& name) const;
  Device& findDevice(const std::string& name);

  /// Sorted unique non-ground node ids.
  std::vector<int> nodes() const;
  /// Number of voltage sources (== number of extra MNA unknowns).
  std::size_t numVoltageSources() const;
  /// Total extra MNA unknowns (1 per V-source, 2 per transformer).
  std::size_t numExtraUnknowns() const;

 private:
  static void checkNodes(int n1, int n2);
  static void checkPositive(double v, const char* what);
  static void checkNameUnique(const std::vector<Device>& devs,
                              const std::string& name);

  std::vector<Device> devices_;
};

}  // namespace power_engine
