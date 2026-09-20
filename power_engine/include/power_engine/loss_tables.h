#pragma once
#include <algorithm>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace power_engine {
namespace loss {

/// Standard axis names: "I" [A], "V" [V], "TJ" [degC].
/// A Table is either switching energy E(I,V,TJ) [J], resistance R(TJ) [Ohm],
/// or forward drop VF(TJ) [V] — the meaning comes from where it is attached.
///
/// N-dimensional grid table with multilinear interpolation and CLAMPING
/// outside the grid (datasheet practice: never extrapolate loss energy).
/// Single-point axes are allowed (constant along that axis).
/// Values are row-major with the LAST axis fastest:
///   index = ((i0*n1 + i1)*n2 + i2) ... for axes (A0, A1, A2, ...).
class Table {
 public:
  Table() = default;
  Table(std::vector<std::string> axes, std::vector<std::vector<double>> grids,
        std::vector<double> values, std::string tag = "");

  bool empty() const { return axes_.empty(); }
  std::size_t dims() const { return axes_.size(); }
  const std::vector<std::string>& axes() const { return axes_; }
  const std::vector<std::vector<double>>& grids() const { return grids_; }
  const std::vector<double>& values() const { return values_; }
  /// Value-kind tag ("E", "R", "VF" for netlist round-trip; free-form).
  const std::string& tag() const { return tag_; }

  /// Interpolate at coords (axis name -> value). Every table axis must be
  /// present (throws otherwise); extra coords are ignored. Out-of-range
  /// coords clamp to the grid edge.
  double at(const std::map<std::string, double>& coords) const;

 private:
  std::vector<std::string> axes_;
  std::vector<std::vector<double>> grids_;
  std::vector<double> values_;
  std::string tag_;
};

/// Datasheet loss model for one switch/diode. Empty tables are inactive and
/// the engine falls back to the device scalars (eon/eoff/ron/vf).
struct DeviceLossModel {
  Table eon;    ///< switching energy E(I,V,TJ) [J] at turn-on (switch)
  Table eoff;   ///< switching energy E(I,V,TJ) [J] at turn-off (switch)
  Table ronTj;  ///< on-resistance R(TJ) [Ohm] (switch + diode)
  Table vfTj;   ///< forward drop VF(TJ) [V] (diode)
  double tamb = 25.0;  ///< fallback Tj [degC] when no thermal network attached

  bool hasSwitching() const { return !eon.empty() || !eoff.empty(); }
  bool hasRon() const { return !ronTj.empty(); }
  bool hasVf() const { return !vfTj.empty(); }
};

}  // namespace loss
}  // namespace power_engine
