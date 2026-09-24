#include "power_engine/statespace.h"

#include <cmath>
#include <map>
#include <stdexcept>
#include <utility>

#include "power_engine/circuit.h"

namespace power_engine {
namespace statespace {
namespace {

// Node id -> row index (-1 for ground).
int nodeRow(const std::map<int, int>& idx, int node) {
  if (node == 0) return -1;
  auto it = idx.find(node);
  if (it == idx.end()) throw std::runtime_error("statespace: unknown node");
  return it->second;
}

void addG(Eigen::MatrixXd& g, int r1, int r2, double v) {
  if (r1 >= 0) g(r1, r1) += v;
  if (r2 >= 0) g(r2, r2) += v;
  if (r1 >= 0 && r2 >= 0) {
    g(r1, r2) -= v;
    g(r2, r1) -= v;
  }
}

int probeNode(const std::string& key) {
  if (key.size() < 3 || key[0] != 'v' || key[1] != ':') {
    throw std::runtime_error("statespace: output must be 'v:N', got '" + key + "'");
  }
  const int node = std::stoi(key.substr(2));
  if (node <= 0) throw std::runtime_error("statespace: cannot probe ground");
  return node;
}

}  // namespace

StateSpace exportStateSpace(const Circuit& circuit,
                            const std::vector<std::string>& outputs) {
  if (outputs.empty()) throw std::runtime_error("statespace: no outputs requested");
  // Node indexing (ground excluded).
  std::map<int, int> idx;
  for (int n : circuit.nodes()) {
    if (n == 0) continue;
    idx[n] = static_cast<int>(idx.size());
  }
  const int n = static_cast<int>(idx.size());
  if (n == 0) throw std::runtime_error("statespace: no dynamic nodes");

  Eigen::MatrixXd gn = Eigen::MatrixXd::Zero(n, n);
  Eigen::VectorXd aff = Eigen::VectorXd::Zero(n);
  // Incidence columns for dynamic elements (ground rows skipped).
  std::vector<std::pair<int, int>> lNodes;  // (n1, n2) per inductor
  std::vector<double> lVals;
  std::vector<std::pair<int, int>> cNodes;
  std::vector<double> cVals;
  std::vector<std::string> lNames;
  struct VSrc {
    int node = -1;      // forced node id
    double gain = 1.0;  // v(node) = gain * value
    std::string name;
    double value = 0.0;
  };
  std::vector<VSrc> vsrcs;
  struct ISrc {
    int n1 = -1, n2 = -1;
    std::string name;
    double value = 0.0;
  };
  std::vector<ISrc> isrcs;

  for (const auto& d : circuit.devices()) {
    const int r1 = nodeRow(idx, d.n1), r2 = nodeRow(idx, d.n2);
    switch (d.type) {
      case DeviceType::Resistor:
        addG(gn, r1, r2, 1.0 / d.value);
        break;
      case DeviceType::Switch: {
        if (d.recT != 0.0) {
          throw std::runtime_error("statespace: switch '" + d.name + "' has active tail");
        }
        addG(gn, r1, r2, 1.0 / (d.closed ? d.ron : d.roff));
        break;
      }
      case DeviceType::Diode: {
        if (d.recT != 0.0) {
          throw std::runtime_error("statespace: diode '" + d.name + "' in recovery");
        }
        if (d.conducting) {
          const double g = 1.0 / d.ron;
          addG(gn, r1, r2, g);
          if (r1 >= 0) aff(r1) -= g * d.vf;
          if (r2 >= 0) aff(r2) += g * d.vf;
        } else {
          addG(gn, r1, r2, 1.0 / d.roff);
        }
        break;
      }
      case DeviceType::Inductor:
        lNodes.emplace_back(d.n1, d.n2);
        lVals.push_back(d.value);
        lNames.push_back(d.name);
        break;
      case DeviceType::Capacitor:
        cNodes.emplace_back(d.n1, d.n2);
        cVals.push_back(d.value);
        break;
      case DeviceType::VoltageSource: {
        if (d.n1 != 0 && d.n2 != 0) {
          throw std::runtime_error("statespace: floating V-source '" + d.name + "'");
        }
        VSrc s;
        s.name = d.name;
        s.value = d.value;
        if (d.n2 == 0) {
          s.node = d.n1;
          s.gain = 1.0;
        } else {
          s.node = d.n2;
          s.gain = -1.0;
        }
        vsrcs.push_back(s);
        break;
      }
      case DeviceType::CurrentSource:
        // Incidence is KCL-leaving form like eL/eC (+1 at n1, -1 at n2);
        // values enter through gAI/gDI input columns, never const.
        isrcs.push_back({d.n1, d.n2, d.name, d.value});
        break;
      default:
        throw std::runtime_error("statespace: unsupported device '" + d.name + "'");
    }
  }

  const int nL = static_cast<int>(lNodes.size());
  const int nC = static_cast<int>(cNodes.size());
  // E matrices (rows skip ground).
  Eigen::MatrixXd eL = Eigen::MatrixXd::Zero(n, nL);
  for (int k = 0; k < nL; ++k) {
    const int r1 = nodeRow(idx, lNodes[k].first);
    const int r2 = nodeRow(idx, lNodes[k].second);
    if (r1 >= 0) eL(r1, k) = 1.0;
    if (r2 >= 0) eL(r2, k) = -1.0;
  }
  Eigen::MatrixXd eC = Eigen::MatrixXd::Zero(n, nC);
  for (int j = 0; j < nC; ++j) {
    const int r1 = nodeRow(idx, cNodes[j].first);
    const int r2 = nodeRow(idx, cNodes[j].second);
    if (r1 >= 0) eC(r1, j) = 1.0;
    if (r2 >= 0) eC(r2, j) = -1.0;
  }
  // Partition: F = source-forced, A = algebraic (no C incidence), D = rest.
  std::vector<int> isF(n, 0);
  for (const auto& s : vsrcs) isF[nodeRow(idx, s.node)] = 1;
  std::vector<int> isA(n, 0);
  for (int r = 0; r < n; ++r) {
    if (isF[r]) continue;
    bool hasC = false;
    for (int j = 0; j < nC; ++j) {
      if (eC(r, j) != 0.0) {
        hasC = true;
        break;
      }
    }
    if (!hasC) isA[r] = 1;
  }
  auto part = [&](const std::vector<int>& mask) {
    std::vector<int> rows;
    for (int r = 0; r < n; ++r) {
      if (mask[r] != 0) rows.push_back(r);
    }
    return rows;
  };
  std::vector<int> allF(n, 0), allD(n, 0);
  for (int r = 0; r < n; ++r) {
    allF[r] = isF[r];
    allD[r] = (!isF[r] && !isA[r]) ? 1 : 0;
  }
  const std::vector<int> rowsF = part(allF);
  const std::vector<int> rowsA = part(isA);
  const std::vector<int> rowsD = part(allD);
  const int nF = static_cast<int>(rowsF.size());
  const int nA = static_cast<int>(rowsA.size());
  const int nD = static_cast<int>(rowsD.size());
  if (nD + nL == 0) throw std::runtime_error("statespace: no states");

  auto sub = [&](const Eigen::MatrixXd& m, const std::vector<int>& rr,
                 const std::vector<int>& cc) {
    Eigen::MatrixXd o(static_cast<int>(rr.size()), static_cast<int>(cc.size()));
    for (std::size_t i = 0; i < rr.size(); ++i) {
      for (std::size_t j = 0; j < cc.size(); ++j) o(static_cast<int>(i), static_cast<int>(j)) = m(rr[i], cc[j]);
    }
    return o;
  };
  auto subV = [&](const Eigen::VectorXd& v, const std::vector<int>& rr) {
    Eigen::VectorXd o(static_cast<int>(rr.size()));
    for (std::size_t i = 0; i < rr.size(); ++i) o(static_cast<int>(i)) = v(rr[i]);
    return o;
  };
  const Eigen::MatrixXd gAA = sub(gn, rowsA, rowsA);
  const Eigen::MatrixXd gAD = sub(gn, rowsA, rowsD);
  const Eigen::MatrixXd gAF = sub(gn, rowsA, rowsF);
  const Eigen::MatrixXd gDA = sub(gn, rowsD, rowsA);
  const Eigen::MatrixXd gDD = sub(gn, rowsD, rowsD);
  const Eigen::MatrixXd gDF = sub(gn, rowsD, rowsF);
  std::vector<int> allL(static_cast<std::size_t>(nL));
  for (int k = 0; k < nL; ++k) allL[static_cast<std::size_t>(k)] = k;
  const Eigen::MatrixXd eLA = sub(eL, rowsA, allL);
  const Eigen::MatrixXd eLD = sub(eL, rowsD, allL);
  const Eigen::MatrixXd eLF = sub(eL, rowsF, allL);
  // eC blocks for the capacitance matrix M_DD = E_CD * C * E_CD'.
  Eigen::MatrixXd eCD(nD, nC);
  for (int i = 0; i < nD; ++i) {
    for (int j = 0; j < nC; ++j) eCD(i, j) = eC(rowsD[i], j);
  }
  const Eigen::VectorXd affA = subV(aff, rowsA);
  const Eigen::VectorXd affD = subV(aff, rowsD);

  // Input vector: [Vsrc values..., Isrc values..., 1(const)].
  const int nV = static_cast<int>(vsrcs.size());
  const int nI = static_cast<int>(isrcs.size());
  const int nU = nV + nI + 1;
  // Forced-node map: v_F = W_F * u (+ nothing affine: sources exact).
  Eigen::MatrixXd wF = Eigen::MatrixXd::Zero(nF, nU);
  for (int k = 0; k < nV; ++k) {
    const int r = nodeRow(idx, vsrcs[k].node);
    for (int i = 0; i < nF; ++i) {
      if (rowsF[i] == r) wF(i, k) = vsrcs[k].gain;
    }
  }
  // I-source incidence onto A/D rows (KCL-leaving form, like eL/eC).
  Eigen::MatrixXd gAI = Eigen::MatrixXd::Zero(nA, nI);
  Eigen::MatrixXd gDI = Eigen::MatrixXd::Zero(nD, nI);
  for (int k = 0; k < nI; ++k) {
    const int r1 = nodeRow(idx, isrcs[k].n1);
    const int r2 = nodeRow(idx, isrcs[k].n2);
    for (int i = 0; i < nA; ++i) {
      if (rowsA[i] == r1) gAI(i, k) += 1.0;
      if (rowsA[i] == r2) gAI(i, k) -= 1.0;
    }
    for (int i = 0; i < nD; ++i) {
      if (rowsD[i] == r1) gDI(i, k) += 1.0;
      if (rowsD[i] == r2) gDI(i, k) -= 1.0;
    }
  }

  // Deferred construction: FullPivLU asserts on empty (0x0) blocks in
  // Debug (no-A-node or no-D-node circuits are legitimate); guarded use
  // below only touches the factorization when its block is nonempty.
  Eigen::FullPivLU<Eigen::MatrixXd> luA;
  if (nA > 0) luA.compute(gAA);
  if (nA > 0 && !luA.isInvertible()) {
    throw std::runtime_error("statespace: singular algebraic block (floating node?)");
  }
  Eigen::MatrixXd mDD = Eigen::MatrixXd::Zero(nD, nD);
  for (int j = 0; j < nC; ++j) mDD += cVals[j] * eCD.col(j) * eCD.col(j).transpose();
  Eigen::FullPivLU<Eigen::MatrixXd> luM;
  if (nD > 0) luM.compute(mDD);
  if (nD > 0 && !luM.isInvertible()) {
    throw std::runtime_error("statespace: singular capacitance block (C-V loop?)");
  }
  Eigen::VectorXd lDiag(nL);
  for (int k = 0; k < nL; ++k) lDiag(k) = lVals[k];

  // v_A = S * (-(gAD*v_D + eLA*i_L + gAF*v_F + gAI*u_I + affA)).
  // Assemble A (states [i_L; v_D]) and B column by column via solves.
  const int nX = nL + nD;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nX, nX);
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(nX, nU);
  // Helper: given (v_D coeffs per state/input + const), solve v_A part.
  // Everything is KCL-leaving form (like eL/eC/gn: +1 at branch from-node,
  // -1 at to-node; aff carries diode-Vf leaving constants): driven source
  // values enter through gAI/gDI input columns, never const, so:
  //   v_A,q = -luA.solve(gAD*qD + eLA*qL + gAF*qF + gAI*qI + qc).
  // Inductor rows: L*di/dt = eLA'*v_A + eLD'*v_D + eLF'*v_F.
  // KCL_D rows: M*dv_D/dt = -(gDA*v_A + gDD*v_D + gDF*v_F + eLD*i_L)
  //                            -(gDI*u_I + affD).
  auto vAfor = [&](const Eigen::VectorXd& qD, const Eigen::VectorXd& qL,
                   const Eigen::VectorXd& qF, const Eigen::VectorXd& qI,
                   const Eigen::VectorXd& qc) {
    Eigen::VectorXd r = Eigen::VectorXd::Zero(nA);
    if (nD > 0) r += gAD * qD;
    if (nL > 0) r += eLA * qL;
    if (nF > 0) r += gAF * qF;
    if (nI > 0) r += gAI * qI;
    r += qc;
    if (nA == 0) return r;
    const Eigen::VectorXd vA = -luA.solve(r);
    return vA;
  };
  // A columns: unit state directions.
  for (int c = 0; c < nX; ++c) {
    Eigen::VectorXd qD = Eigen::VectorXd::Zero(nD);
    Eigen::VectorXd qL = Eigen::VectorXd::Zero(nL);
    if (c < nL) {
      qL(c) = 1.0;
    } else {
      qD(c - nL) = 1.0;
    }
    const Eigen::VectorXd vA =
        vAfor(qD, qL, Eigen::VectorXd::Zero(nF), Eigen::VectorXd::Zero(nI),
              Eigen::VectorXd::Zero(nA));
    Eigen::VectorXd di = Eigen::VectorXd::Zero(nL);
    if (nL > 0) {
      Eigen::VectorXd rhs = Eigen::VectorXd::Zero(nL);
      if (nA > 0) rhs += eLA.transpose() * vA;
      if (nD > 0) rhs += eLD.transpose() * qD;
      di = rhs.cwiseQuotient(lDiag);
    }
    Eigen::VectorXd dv = Eigen::VectorXd::Zero(nD);
    if (nD > 0) {
      Eigen::VectorXd rhs = Eigen::VectorXd::Zero(nD);
      if (nA > 0) rhs += gDA * vA;
      rhs += gDD * qD;
      if (nL > 0) rhs += eLD * qL;
      dv = -luM.solve(rhs);
    }
    for (int k = 0; k < nL; ++k) A(k, c) = di(k);
    for (int k = 0; k < nD; ++k) A(nL + k, c) = dv(k);
  }
  // B columns: V inputs, I inputs, const 1.
  for (int c = 0; c < nU; ++c) {
    Eigen::VectorXd qF = Eigen::VectorXd::Zero(nF);
    Eigen::VectorXd qI = Eigen::VectorXd::Zero(nI);
    Eigen::VectorXd qc = Eigen::VectorXd::Zero(nA);
    if (c < nV) {
      qF = wF.col(c);
    } else if (c < nV + nI) {
      qI(c - nV) = 1.0;
    } else {
      qc = affA;
    }
    // KCL_D const part needs affD only for the const column (driven
    // source values live in their own columns via gAI/gDI).
    Eigen::VectorXd qIc = Eigen::VectorXd::Zero(nI);
    const Eigen::VectorXd vA = vAfor(Eigen::VectorXd::Zero(nD), Eigen::VectorXd::Zero(nL),
                                     qF, (c < nV + nI && c >= nV) ? qI : qIc, qc);
    Eigen::VectorXd di = Eigen::VectorXd::Zero(nL);
    if (nL > 0) {
      Eigen::VectorXd rhs = Eigen::VectorXd::Zero(nL);
      if (nA > 0) rhs += eLA.transpose() * vA;
      if (nF > 0) rhs += eLF.transpose() * qF;
      di = rhs.cwiseQuotient(lDiag);
    }
    Eigen::VectorXd dv = Eigen::VectorXd::Zero(nD);
    if (nD > 0) {
      Eigen::VectorXd rhs = Eigen::VectorXd::Zero(nD);
      if (nA > 0) rhs += gDA * vA;
      if (nF > 0) rhs += gDF * qF;
      if (c >= nV && c < nV + nI) rhs += gDI * qI;
      if (c == nU - 1) rhs += affD;  // diode-Vf affine only (driven values
      // live in their own columns, never const)
      dv = -luM.solve(rhs);
    }
    for (int k = 0; k < nL; ++k) B(k, c) = di(k);
    for (int k = 0; k < nD; ++k) B(nL + k, c) = dv(k);
  }

  // Outputs: y = C*x + D*u from v_A/v_D/v_F expressions.
  const int nY = static_cast<int>(outputs.size());
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(nY, nX);
  Eigen::MatrixXd D = Eigen::MatrixXd::Zero(nY, nU);
  for (int o = 0; o < nY; ++o) {
    const int node = probeNode(outputs[o]);
    const int r = nodeRow(idx, node);
    // Locate in F/A/D.
    int fi = -1, ai = -1, di = -1;
    for (int i = 0; i < nF; ++i) {
      if (rowsF[i] == r) fi = i;
    }
    for (int i = 0; i < nA; ++i) {
      if (rowsA[i] == r) ai = i;
    }
    for (int i = 0; i < nD; ++i) {
      if (rowsD[i] == r) di = i;
    }
    if (fi >= 0) {
      for (int c = 0; c < nU; ++c) D(o, c) = wF(fi, c);
    } else if (di >= 0) {
      C(o, nL + di) = 1.0;
    } else {
      // ai >= 0 guaranteed: F/A/D partition every non-ground row, and
      // unknown/ground outputs throw in probeNode/nodeRow above.
      // Row ai of v_A expression: recompute via unit solves.
      for (int c = 0; c < nX; ++c) {
        Eigen::VectorXd qD = Eigen::VectorXd::Zero(nD);
        Eigen::VectorXd qL = Eigen::VectorXd::Zero(nL);
        if (c < nL) {
          qL(c) = 1.0;
        } else {
          qD(c - nL) = 1.0;
        }
        C(o, c) = vAfor(qD, qL, Eigen::VectorXd::Zero(nF), Eigen::VectorXd::Zero(nI),
                        Eigen::VectorXd::Zero(nA))(ai);
      }
      for (int c = 0; c < nU; ++c) {
        Eigen::VectorXd qF = Eigen::VectorXd::Zero(nF);
        Eigen::VectorXd qI = Eigen::VectorXd::Zero(nI);
        Eigen::VectorXd qc = Eigen::VectorXd::Zero(nA);
        if (c < nV) {
          qF = wF.col(c);
        } else if (c < nV + nI) {
          qI(c - nV) = 1.0;
        } else {
          qc = affA;
        }
        D(o, c) = vAfor(Eigen::VectorXd::Zero(nD), Eigen::VectorXd::Zero(nL), qF, qI, qc)(ai);
      }
    }
  }

  StateSpace ss;
  ss.a = std::move(A);
  ss.b = std::move(B);
  ss.c = std::move(C);
  ss.d = std::move(D);
  for (const auto& nm : lNames) ss.stateNames.push_back("i:" + nm);
  for (int i = 0; i < nD; ++i) {
    int node = -1;
    for (const auto& [id, row] : idx) {
      if (row == rowsD[i]) node = id;
    }
    ss.stateNames.push_back("v:" + std::to_string(node));
  }
  for (const auto& s : vsrcs) ss.inputNames.push_back(s.name);
  for (const auto& s : isrcs) ss.inputNames.push_back(s.name);
  ss.inputNames.emplace_back("1");
  ss.outputNames = outputs;
  return ss;
}

StateSpace averageStateSpace(const StateSpace& on, const StateSpace& off, double duty) {
  if (!(duty >= 0.0) || !(duty <= 1.0) || !std::isfinite(duty)) {
    throw std::runtime_error("averageStateSpace: duty must be in [0,1]");
  }
  if (on.a.rows() != off.a.rows() || on.a.cols() != off.a.cols() ||
      on.b.rows() != off.b.rows() || on.b.cols() != off.b.cols() ||
      on.c.rows() != off.c.rows() || on.c.cols() != off.c.cols() ||
      on.d.rows() != off.d.rows() || on.d.cols() != off.d.cols()) {
    throw std::runtime_error("averageStateSpace: dimension mismatch");
  }
  if (on.stateNames != off.stateNames || on.inputNames != off.inputNames ||
      on.outputNames != off.outputNames) {
    throw std::runtime_error("averageStateSpace: name mismatch (different circuits?)");
  }
  StateSpace avg = on;
  avg.a = duty * on.a + (1.0 - duty) * off.a;
  avg.b = duty * on.b + (1.0 - duty) * off.b;
  avg.c = duty * on.c + (1.0 - duty) * off.c;
  avg.d = duty * on.d + (1.0 - duty) * off.d;
  return avg;
}

std::complex<double> evalTransfer(const StateSpace& ss, int outIdx, int inIdx,
                                  std::complex<double> s) {
  if (outIdx < 0 || outIdx >= ss.c.rows() || inIdx < 0 || inIdx >= ss.b.cols()) {
    throw std::runtime_error("evalTransfer: index out of range");
  }
  const int nx = static_cast<int>(ss.a.rows());
  Eigen::MatrixXcd m = s * Eigen::MatrixXcd::Identity(nx, nx) - ss.a.cast<std::complex<double>>();
  Eigen::FullPivLU<Eigen::MatrixXcd> lu(m);
  if (!lu.isInvertible()) throw std::runtime_error("evalTransfer: sI-A singular");
  const Eigen::VectorXcd x =
      lu.solve(ss.b.col(inIdx).cast<std::complex<double>>());
  return (ss.c.row(outIdx).cast<std::complex<double>>() * x)(0) + ss.d(outIdx, inIdx);
}

std::complex<double> evalDutyTransfer(const StateSpace& on, const StateSpace& off,
                                      double duty, const Eigen::VectorXd& uSs,
                                      int outIdx, std::complex<double> s) {
  const StateSpace avg = averageStateSpace(on, off, duty);  // validates match + duty
  if (outIdx < 0 || outIdx >= avg.c.rows()) {
    throw std::runtime_error("evalDutyTransfer: output index out of range");
  }
  if (uSs.size() != avg.b.cols()) {
    throw std::runtime_error("evalDutyTransfer: uSs/input dimension mismatch");
  }
  // Steady operating point X = -Aavg^-1 * Bavg * u.
  Eigen::FullPivLU<Eigen::MatrixXd> luA(avg.a);
  if (!luA.isInvertible()) throw std::runtime_error("evalDutyTransfer: A singular");
  const Eigen::VectorXd xSs = -luA.solve(avg.b * uSs);
  // Duty sensitivity of the averaged dynamics at (X, u).
  const Eigen::VectorXd bHat =
      (on.a - off.a) * xSs + (on.b - off.b) * uSs;
  const int nx = static_cast<int>(avg.a.rows());
  Eigen::MatrixXcd m = s * Eigen::MatrixXcd::Identity(nx, nx) - avg.a.cast<std::complex<double>>();
  Eigen::FullPivLU<Eigen::MatrixXcd> lu(m);
  if (!lu.isInvertible()) throw std::runtime_error("evalDutyTransfer: sI-A singular");
  const Eigen::VectorXcd x = lu.solve(bHat.cast<std::complex<double>>());
  return (avg.c.row(outIdx).cast<std::complex<double>>() * x)(0);
}

}  // namespace statespace
}  // namespace power_engine
