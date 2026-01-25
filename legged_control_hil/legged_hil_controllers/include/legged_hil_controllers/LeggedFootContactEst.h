# include <legged_hil_controllers/LeggedFootObs.h>

# include <unordered_map>

# include <algorithm>


namespace legged {

	static constexpr std::size_t kNumFeet = 4;
	static constexpr std::size_t kDim = 3;
	using FeatureVec = std::array<double, kDim>;

class LeggedFootContactEst {
	public:
		// Logistic regression parameters (trained)
		struct LogisticParams {
			FeatureVec mu{};
			FeatureVec sigma{};
			FeatureVec w{};
			double b = 0.0;
			double priorLogOdds = 0.0;
		};

		// Filter params (log-odds domain)
		struct FilterParams {
			double alpha = 0.003; // touchdown filter time constant
			double beta = 0.003;  // liftoff filter time constant

			double L_min = -20.0;   // clamp range for log-odds (anti-windup)
			double L_max =  20.0;

			// hysteresis in log-odds domain
			double L_enter = 0.85;  // stance enter threshold (logit(0.7) ~ 0.847)
			double L_exit  = -0.85; // stance exit threshold  (logit(0.3) ~ -0.847)

			int N_enter = 1;  // number of consecutive samples to enter stance
			int N_exit  = 1;  // number of consecutive samples to exit stance
		};

		// Result
		struct LegEstState {
			double logOdds = 0.0;      			// filtered log-odds
			double probStance = 0.5;				// filtered probability
			double probSwing = 0.5;				// filtered probability
			double logProbStance = std::log(0.5);		// log prob
			double logProbSwing = std::log(0.5);			// log prob
			double currentLogistic = 0.0; 	// current logistic regression output (log-odds)
			int N_enter_counter = 0;              		// hysteresis counter
			int N_exit_counter = 0;               		// hysteresis counter
			bool isStance = false;     			// hysteresis decision		
		};

	public:
		LeggedFootContactEst() = default;
		LeggedFootContactEst(std::shared_ptr<LeggedFootObs> footObsPtr, const LogisticParams& logisticParams, const FilterParams& filterParams);

		void reset(double initLogOdds = 0.0);

		void predict();
		void update();

		const std::unordered_map<std::string, LegEstState>& getLegEstStates() const { return legEstStates_; }

	private:
		LogisticParams logisticParams_;
		FilterParams filterParams_;
		std::unordered_map<std::string, LegEstState> legEstStates_; 
		std::shared_ptr<LeggedFootObs> footObsPtr_ = nullptr;

	private:
		double computeLogistic_(const FeatureVec& x_norm) const;
		FeatureVec normalizeFeatures_(const FeatureVec& x_raw) const;

		double sigmoid_(double x) const;
		double clamp_(double logOdds, double min, double max) const;

		void updateHysteresis_(LegEstState& state);
};

}  // namespace legged