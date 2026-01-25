# include <legged_hil_controllers/LeggedFootContactEst.h>


namespace legged {
	
LeggedFootContactEst::LeggedFootContactEst(std::shared_ptr<LeggedFootObs> footObsPtr, const LogisticParams& logisticParams, const FilterParams& filterParams)
  : footObsPtr_(footObsPtr), logisticParams_(logisticParams), filterParams_(filterParams) {
  // Initialize leg estimation states
  for (const auto& footName : footObsPtr_->footNames_) {
	legEstStates_[footName] = LegEstState{};
  }
}

void LeggedFootContactEst::reset(double initLogOdds) {
  initLogOdds = clamp_(initLogOdds, filterParams_.L_min, filterParams_.L_max);
  
  const double p1 = sigmoid_(initLogOdds);
  const double p0 = 1.0 - p1;
  const double logP1 = std::log(p1);
  const double logP0 = std::log(p0);

  for (auto& [footName, state] : legEstStates_) {
    state.logOdds = initLogOdds;
    state.probStance = p1;
    state.probSwing = p0;
    state.logProbStance = logP1;
    state.logProbSwing = logP0;
    state.currentLogistic = 0.0;
    state.N_enter_counter = 100;
    state.N_exit_counter = 100;
    
    updateHysteresis_(state);
  }
}

void LeggedFootContactEst::predict() {
  for (auto& [footName, state] : legEstStates_){
    double lopP0Prev = state.logProbSwing;
    double lopP1Prev = state.logProbStance;

    double lopP0 = std::log(std::exp(lopP0Prev) * (1.0 - filterParams_.alpha) + std::exp(lopP1Prev) * filterParams_.beta);
    double lopP1 = std::log(std::exp(lopP0Prev) * filterParams_.alpha + std::exp(lopP1Prev) * (1.0 - filterParams_.beta));

    // Normalize
    double z = std::log(std::exp(lopP0) + std::exp(lopP1));
    lopP0 -= z;
    lopP1 -= z;
    
    state.logProbSwing = lopP0;
    state.logProbStance = lopP1;
    state.probSwing = std::exp(lopP0);
    state.probStance = std::exp(lopP1);
    state.logOdds = lopP1 - lopP0;
  }
}

void LeggedFootContactEst::update() {
  for (auto& [footName, state] : legEstStates_) {
    // 1) Get features from foot observation
    const auto& footStates = footObsPtr_->getFootStates();
    const auto it = footStates.find(footName);
    if (it == footStates.end()) continue;

    const auto& fs = it->second;
    // FeatureVec x_raw{fs.F_ratio, fs.F_ratio_delta, fs.velocity_W.z(), fs.position_W.z()};
    FeatureVec x_raw{fs.F_ratio, fs.F_ratio_delta, fs.velocity_W.z()};
    

    // 2) Normalize features
    FeatureVec x_norm = normalizeFeatures_(x_raw);

    // 3) Compute logistic regression output
    double logit = computeLogistic_(x_norm);
    state.currentLogistic = logit;

    // 4) Update log-prob
    state.logProbSwing -= 0.5 * logit;
    state.logProbStance += 0.5 * logit;

    // 5) Normalize to log-prob
    double z = std::log(std::exp(state.logProbSwing) + std::exp(state.logProbStance));
    state.logProbSwing -= z;
    state.logProbStance -= z;

    // 6) Update log-odds and prob
    state.logOdds = state.logProbStance - state.logProbSwing;
    state.logOdds = clamp_(state.logOdds, filterParams_.L_min, filterParams_.L_max);
    state.probStance = sigmoid_(state.logOdds);
    state.probSwing = 1.0 - state.probStance;
    state.logProbStance = std::log(state.probStance);
    state.logProbSwing = std::log(state.probSwing);
    
    // 7) Update hysteresis decision
    updateHysteresis_(state);
  }
}

double LeggedFootContactEst::computeLogistic_(const FeatureVec& x_norm) const {
  double logit = logisticParams_.b;
  logit -= logisticParams_.priorLogOdds;
  for (std::size_t i = 0; i < kDim; ++i) {
    logit += logisticParams_.w[i] * x_norm[i];
  }
  return logit;
}

FeatureVec LeggedFootContactEst::normalizeFeatures_(const FeatureVec& x_raw) const {
  FeatureVec x_norm;
  for (std::size_t i = 0; i < kDim; ++i) {
    x_norm[i] = (x_raw[i] - logisticParams_.mu[i]) / logisticParams_.sigma[i];
  }
  return x_norm;
}

double LeggedFootContactEst::sigmoid_(double x) const {
  return 1.0 / (1.0 + std::exp(-x));
}

double LeggedFootContactEst::clamp_(double logOdds, double min, double max) const {
  if (logOdds < min) return min;
  if (logOdds > max) return max;
  return logOdds;
}

void LeggedFootContactEst::updateHysteresis_(LegEstState& state) {
  if (!state.isStance) {
    // Currently swing
    if (state.logOdds >= filterParams_.L_enter) {
      state.N_enter_counter++;
      if (state.N_enter_counter >= filterParams_.N_enter) {
        state.isStance = true;
        state.N_exit_counter = 0;
        state.N_enter_counter = 0;
      }
    }
    else {
      state.N_exit_counter = 0;
      state.N_enter_counter = 0;
    }
  }
  else {
    // Currently stance
    if (state.logOdds <= filterParams_.L_exit) {
      state.N_exit_counter++;
      if (state.N_exit_counter >= filterParams_.N_exit) {
        state.isStance = false;
        state.N_exit_counter = 0;
        state.N_enter_counter = 0;
      }
    }
    else {
      state.N_exit_counter = 0;
      state.N_enter_counter = 0;
    }
  }

}

}  // namespace legged