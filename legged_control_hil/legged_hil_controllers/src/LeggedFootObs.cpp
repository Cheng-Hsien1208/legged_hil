#include <legged_hil_controllers/LeggedFootObs.h>

#include <algorithm>
#include <stdexcept>

#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <chrono>
#include <ctime>
#include <sstream>

static bool pathExists(const std::string& path) {
  struct stat st;
  return (stat(path.c_str(), &st) == 0) && S_ISDIR(st.st_mode);
}

// Recursive mkdir like "mkdir -p"
static void mkdirs(const std::string& path, mode_t mode = 0755) {
  if (path.empty()) return;
  if (pathExists(path)) return;

  std::string cur;
  cur.reserve(path.size());

  // Handle absolute path
  size_t i = 0;
  if (path[0] == '/') {
    cur = "/";
    i = 1;
  }

  for (; i < path.size(); ++i) {
    const char c = path[i];
    cur.push_back(c);

    if (c == '/' || i == path.size() - 1) {
      // remove trailing '/'
      std::string dir = cur;
      while (dir.size() > 1 && dir.back() == '/') dir.pop_back();

      if (!dir.empty() && !pathExists(dir)) {
        if (::mkdir(dir.c_str(), mode) != 0) {
          if (errno != EEXIST) {
            throw std::runtime_error("[LeggedFootObs] mkdir failed: " + dir + " : " + std::string(strerror(errno)));
          }
        }
      }
    }
  }
}

static bool fileExistsAndNonEmpty(const std::string& file) {
  struct stat st;
  if (stat(file.c_str(), &st) != 0) return false;
  return (st.st_size > 0);
}


namespace legged {

LeggedFootObs::LeggedFootObs(const std::string& urdfFile, const std::vector<std::string>& jointNames,
                             const ocs2::CentroidalModelInfo& centroidalInfo, const std::vector<std::string>& footNames, const std::string& csvRootDir)
    : urdfFile_(urdfFile), jointNames_(jointNames), footNames_(footNames), mapping_(centroidalInfo){
  // Build pinocchio interface from urdf + jointNames (OCS2 helper)
  pinocchioInterfacePtr_ = std::make_unique<ocs2::PinocchioInterface>(ocs2::centroidal_model::createPinocchioInterface(urdfFile_, jointNames_));
  eeKinematicsPtr_ = std::make_shared<ocs2::PinocchioEndEffectorKinematics>(*pinocchioInterfacePtr_, mapping_, footNames_);

  // Set interface for caching inside ee kinematics
  eeKinematicsPtr_->setPinocchioInterface(*pinocchioInterfacePtr_);

  for (const auto& name : footNames_) {
    footStates_[name] = FootState{};
  }

  // Initialize buffers
  qJoints_.setZero(static_cast<int>(centroidalInfo.actuatedDofNum));
  vJoints_.setZero(static_cast<int>(centroidalInfo.actuatedDofNum));
  tauJoints_.setZero(static_cast<int>(centroidalInfo.actuatedDofNum));

  initHampelBuffers_(static_cast<int>(centroidalInfo.actuatedDofNum));

  // =============================
  // Create time-stamped log dir (C++11/14, no filesystem)
  // =============================
  if (!csvRootDir.empty()) {
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_c = std::chrono::system_clock::to_time_t(now);

    std::tm tm{};
    localtime_r(&now_c, &tm);

    char buf[64];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d_%H-%M-%S", &tm);
    const std::string timeStr(buf);

    // logDir = csvRootDir + "/" + timeStr
    std::string logDir = csvRootDir;
    if (!logDir.empty() && logDir.back() != '/') logDir += "/";
    logDir += timeStr;

    // mkdir -p
    mkdirs(logDir);

    // csvPath = logDir + "/legged_foot_obs.csv"
    csvPath_ = logDir + "/legged_foot_obs.csv";
  }

  initCsvIfNeeded_();
}

void LeggedFootObs::buildPinocchioQv_(const ocs2::CentroidalModelInfo& info, vector_t& qPinocchio, vector_t& vPinocchio) {
  // measuredRbdState layout: [rpy(3), pos(3), qJoints(n), omega(3), v(3), vJoints(n)]
  // qPinocchio: [base_pos(3), base_rpy_zyx(3), joint_pos(n)]
  // vPinocchio: [base_lin_vel(3), base_rpy_dot(3), joint_vel(n)]

  const int n = static_cast<int>(info.actuatedDofNum);

  if (measuredRbdState_.size() < (6 + n + 6 + n)) { throw std::runtime_error("[LeggedFootObs] measuredRbdState size too small for assumed layout."); }
  if (qJoints_.size() < n || vJoints_.size() < n) { throw std::runtime_error("[LeggedFootObs] qJoints_/vJoints_ size smaller than actuatedDoF."); }

  qPinocchio.head<3>() = measuredRbdState_.segment<3>(3); // base position
  qPinocchio.segment<3>(3) = measuredRbdState_.head<3>(); // base rpy_zyx
  qPinocchio.tail(n) = qJoints_.head(n);                  // joint positions 

  const int omegaOffset = 6 + n;          // omega starts after base(6) + joints(n)
  const int vLinOffset = omegaOffset + 3; // v starts after omega(3)

  const Eigen::Matrix<ocs2::scalar_t, 3, 1> omegaW = measuredRbdState_.segment<3>(omegaOffset).cast<ocs2::scalar_t>();

  vPinocchio.head<3>() = measuredRbdState_.segment<3>(vLinOffset);  // base linear velocity
  vPinocchio.segment<3>(3) = 
    ocs2::getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<ocs2::scalar_t>(
      qPinocchio.segment<3>(3), measuredRbdState_.segment<3>(omegaOffset)); // base rpy dot
  vPinocchio.tail(n) = vJoints_.head(n);  // joint velocities
}

void LeggedFootObs::initHampelBuffers_(int nJoints) {
  tauWindow_.resize(nJoints);
  tauWindowCount_.assign(nJoints, 0);
  tauWindowHead_.assign(nJoints, 0);

  for (int i = 0; i < nJoints; ++i) {
    tauWindow_[i].fill(0.0);
  }
}

double LeggedFootObs::median_(std::array<double, kHampelWindow> a, int count) {
  int mid = count / 2;
  std::nth_element(a.begin(), a.begin() + mid, a.begin() + count);
  double med = a[mid];

  if ((count % 2) == 0) {
    std::nth_element(a.begin(), a.begin() + mid - 1, a.begin() + count);
    med = 0.5 * (med + a[mid - 1]);
  }
  return med;
}

// Hampel outlier removal (per joint). Returns filtered value.
double LeggedFootObs::hampelFilter_(int jointIdx, double x) {
  // Update ring buffer
  int& head = tauWindowHead_[jointIdx];
  int& count = tauWindowCount_[jointIdx];

  tauWindow_[jointIdx][head] = x;
  head = (head + 1) % kHampelWindow;
  if (count < kHampelWindow) count++;

  std::array<double, kHampelWindow> w{};
  for (int i = 0; i < count; ++i) {
    w[i] = tauWindow_[jointIdx][i];
  }

  const double med = median_(w, count);

  // MAD
  for (int i = 0; i < count; ++i) {
    w[i] = std::abs(w[i] - med);
  }
  const double mad = median_(w, count);

  // Outlier test
  if (std::abs(x - med) > kHampelK * mad) {
    return med;  // replace outlier with median
  }
  return x;
}

void LeggedFootObs::initCsvIfNeeded_() {
  if (csvPath_.empty()) return;

  // ensure parent directory exists
  const auto pos = csvPath_.find_last_of('/');
  if (pos != std::string::npos) {
    const std::string parent = csvPath_.substr(0, pos);
    if (!parent.empty()) mkdirs(parent);
  }

  csv_.open(csvPath_, std::ios::out | std::ios::app);
  if (!csv_.is_open()) {
    throw std::runtime_error("[LeggedFootObs] Failed to open CSV file: " + csvPath_);
  }

  bool needHeader = !fileExistsAndNonEmpty(csvPath_);
  if (needHeader && !csvHeaderWritten_) {
    csv_ << "time,leg_id,F_ratio,F_ratio_delta,v_z,d_z,contact_gt\n";
    csvHeaderWritten_ = true;
    csv_.flush();
  }
}

void LeggedFootObs::logFeatures_() {
  if (csvPath_.empty()) return;
  if (!csv_.is_open()) initCsvIfNeeded_();

  // Need 4 legs
  if (footNames_.size() < 4) return;

  // 2) write per-leg row
  csv_ << std::fixed << std::setprecision(9);

  for (int leg = 0; leg < 4; ++leg) {
    auto it = footStates_.find(footNames_[leg]);
    if (it == footStates_.end()) continue;
    const double F_ratio = it->second.F_ratio;
    const double F_ratio_delta = it->second.F_ratio_delta;
    const double v_z = it->second.velocity_W.z();
    const double d_z = it->second.position_W.z();
    const int contact_gt = contactGt_[leg];

    csv_ << timeSec_ << ","
         << leg << ","
         << F_ratio << ","
         << F_ratio_delta << ","
         << v_z << ","
         << d_z << ","
         << contact_gt << "\n";
  }
  csv_.flush();
}

void LeggedFootObs::update() {

  if (!pinocchioInterfacePtr_) {
    throw std::runtime_error("[LeggedFootObs] pinocchioInterfacePtr_ is null.");
  }
  if (footNames_.size() < 4) {
    // You can still run with <4, but A is fixed 12 dims; simplest is to require 4
    throw std::runtime_error("[LeggedFootObs] footNames_ must contain 4 foot frame names.");
  }

  const auto& model = pinocchioInterfacePtr_->getModel();
  auto& data = pinocchioInterfacePtr_->getData();
  const auto& info = mapping_.getCentroidalModelInfo();
  const int n = static_cast<int>(info.actuatedDofNum);

  // -----------------------------
  // 1) Build q/v
  // -----------------------------
  vector_t qPinocchio(info.generalizedCoordinatesNum);
  vector_t vPinocchio(info.generalizedCoordinatesNum);
  buildPinocchioQv_(info, qPinocchio, vPinocchio);

  // -----------------------------
  // 2) FK cache (for ee position/velocity)
  // -----------------------------
  pinocchio::forwardKinematics(model, data, qPinocchio, vPinocchio);
  pinocchio::computeJointJacobians(model, data);
  pinocchio::updateFramePlacements(model, data);

  // -----------------------------
  // 3) Compute foot position/velocity
  // -----------------------------
  const auto footPositionsW = eeKinematicsPtr_->getPosition(vector_t{});
  std::vector<Eigen::Vector3d> footPositionsRel;
  footPositionsRel.resize(footPositionsW.size());

  const Eigen::Vector3d basePosW = qPinocchio.head<3>();  // base position in world

  for (size_t i = 0; i < footPositionsW.size(); ++i) {
    footPositionsRel[i] = footPositionsW[i] - basePosW;
  }

  const auto footVelocities = eeKinematicsPtr_->getVelocity(vector_t{}, vector_t{});

  for (int leg = 0; leg < 4; ++leg) {
    const std::string& footName = footNames_[leg];
    auto& fs = footStates_[footName];

    // position/velocity
    if (leg < static_cast<int>(footPositionsRel.size())) fs.position_W = footPositionsRel[leg];
    if (leg < static_cast<int>(footVelocities.size())) fs.velocity_W = footVelocities[leg];
  }

  // -----------------------------
  // 4) Compute contact forces
  // -----------------------------
  for (int leg = 0; leg < 4; ++leg) {
    const std::string& footName = footNames_[leg];
    const std::string& jointNameHAA = jointNames_[leg * 3];
    const std::string& jointNameHFE = jointNames_[leg * 3 + 1];
    const std::string& jointNameKFE = jointNames_[leg * 3 + 2];


    // frame id
    const pinocchio::FrameIndex frameId = model.getFrameId(footName);
    if (frameId == static_cast<pinocchio::FrameIndex>(-1)) {
      throw std::runtime_error("[LeggedFootObs] Foot frame not found in model: " + footName);
    }

    // full 6 x nv Jacobian in WORLD
    ocs2::matrix_t J_full = ocs2::matrix_t::Zero(6, model.nv);
    pinocchio::getFrameJacobian(model, data, frameId, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_full);

    Eigen::Matrix3d J_leg;
    J_leg.col(0) = J_full.block<3,1>(0, model.joints[model.getJointId(jointNameHAA)].idx_v());
    J_leg.col(1) = J_full.block<3,1>(0, model.joints[model.getJointId(jointNameHFE)].idx_v());
    J_leg.col(2) = J_full.block<3,1>(0, model.joints[model.getJointId(jointNameKFE)].idx_v());

    // torque vector for this leg (raw)
    Eigen::Vector3d tau_leg_raw;
    tau_leg_raw(0) = tauJoints_(leg * 3);
    tau_leg_raw(1) = tauJoints_(leg * 3 + 1);
    tau_leg_raw(2) = tauJoints_(leg * 3 + 2);

    // Hampel-filtered tau (outlier removal)
    Eigen::Vector3d tau_leg;
    tau_leg(0) = hampelFilter_(leg * 3 + 0, tau_leg_raw(0));
    tau_leg(1) = hampelFilter_(leg * 3 + 1, tau_leg_raw(1));
    tau_leg(2) = hampelFilter_(leg * 3 + 2, tau_leg_raw(2));

    // estimate contact force
    Eigen::Vector3d contactForce = J_leg.transpose().colPivHouseholderQr().solve(tau_leg);
    footStates_[footName].contactForce_W = contactForce;

    // store contactForceNorm_z
    footStates_[footName].contactForceNorm_z = std::abs(contactForce.z());
  }

  // -----------------------------
  // 5) Compute F_ratio and F_ratio_dot
  // -----------------------------
  std::array<double, 4> Fleg{{0.0, 0.0, 0.0, 0.0}};
  double Fsum = 0.0;
  for (int leg = 0; leg < 4; ++leg) {
    const std::string& footName = footNames_[leg];
    const auto it = footStates_.find(footName);
    if (it == footStates_.end()) continue;

    double f = it->second.contactForceNorm_z;
    Fleg[leg] = f;
    Fsum += f;
  }

  const double eps = 1e-9;
  // compute dt for LPF alpha
  double dt = 0.0;
  if (prevTimeSec_ >= 0.0) dt = timeSec_ - prevTimeSec_;

  // alpha from tau
  double alpha = 1.0;
  if (dt > 1e-6) {
    alpha = dt / (fRatioLpTau_ + dt);
    if (alpha < 0.0) alpha = 0.0;
    if (alpha > 1.0) alpha = 1.0;
  }

  for (int leg = 0; leg < 4; ++leg) {
    const std::string& footName = footNames_[leg];
    auto& fs = footStates_[footName];

    fs.F_ratio = Fleg[leg] / (Fsum + eps);

    // init LPF on first valid update (avoid transient spike)
    if (!fRatioLPInit_) {
      fRatioLP_[leg] = fs.F_ratio;
      fs.F_ratio_delta = 0.0;
    } else {
      fRatioLP_[leg] = (1.0 - alpha) * fRatioLP_[leg] + alpha * fs.F_ratio;
      fs.F_ratio_delta = std::abs(fs.F_ratio - fRatioLP_[leg]);
      // ROS_INFO_STREAM("Foot " << footName << " F_ratio: " << fs.F_ratio << ", F_ratio_delta: " << fs.F_ratio_delta);
    }
  }

  // mark LPF ready after one full cycle of legs
  if (!fRatioLPInit_) {
    fRatioLPInit_ = true;
  }

  // -----------------------------
  // 6) Log features if needed
  // -----------------------------
  logFeatures_();

  // -----------------------------
  // 7) Update prev time
  // -----------------------------
  prevTimeSec_ = timeSec_;

}

}  // namespace legged
