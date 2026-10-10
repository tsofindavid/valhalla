#include "sif/truckcost.h"
#include "baldr/accessrestriction.h"
#include "baldr/directededge.h"
#include "baldr/graphconstants.h"
#include "baldr/nodeinfo.h"
#include "baldr/rapidjson_utils.h"
#include "proto_conversions.h"
#include "sif/osrm_car_duration.h"

#include <array>
#include <limits>
#include <vector>

#ifdef INLINE_TEST
#include "test.h"
#include "worker.h"

#include <random>
#endif

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace sif {

// Default options/values
namespace {

// Base transition costs
// Note: all roads of class "Service, other" are already penalized with low_class_penalty, so for
// generic service roads these penalties will add up
constexpr float kDefaultServicePenalty = 0.0f; // Seconds

// Other options
constexpr float kDefaultLowClassPenalty = 15.0f; // Seconds
constexpr float kDefaultLowClassFactor = 1.5f;
constexpr float kDefaultUseTolls = 0.5f; // Factor between 0 and 1
constexpr float kDefaultUseTracks = 0.f; // Avoid tracks by default. Factor between 0 and 1
constexpr float kDefaultUseLivingStreets =
    0.f;                                    // Avoid living streets by default. Factor between 0 and 1
constexpr float kDefaultUseHighways = 0.5f; // Factor between 0 and 1

// Default turn costs
constexpr float kTCStraight = 0.5f;
constexpr float kTCSlight = 0.75f;
constexpr float kTCFavorable = 1.0f;
constexpr float kTCFavorableSharp = 1.5f;
constexpr float kTCCrossing = 2.0f;
constexpr float kTCUnfavorable = 2.5f;
constexpr float kTCUnfavorableSharp = 3.5f;
constexpr float kTCReverse = 9.5f;
constexpr float kTCRamp = 1.5f;
constexpr float kTCRoundabout = 0.5f;

// Default truck attributes
constexpr float kDefaultTruckWeight = 21.77f;  // Metric Tons (48,000 lbs)
constexpr float kDefaultTruckAxleLoad = 9.07f; // Metric Tons (20,000 lbs)
constexpr float kDefaultTruckHeight = 4.11f;   // Meters (13 feet 6 inches)
constexpr float kDefaultTruckWidth = 2.6f;     // Meters (102.36 inches)
constexpr float kDefaultTruckLength = 21.64f;  // Meters (71 feet)
constexpr uint32_t kDefaultAxleCount = 5;      // 5 axles for above truck config

// Fuel model: resistance, power at constant speed and convex fuel rate from Rakha et al.
// Speed in km/h, mass in kg, force in N, power in kW, fuel in liters.
constexpr float kFuelAirDensity = 1.2256f; // kg/m^3
constexpr float kFuelDragCoefficient = 0.78f;
constexpr float kFuelFrontalArea = 10.f; // m^2
constexpr float kFuelRollingCoefficient = 1.25f;
constexpr float kFuelRollingC1 = 0.0328f;
constexpr float kFuelRollingC2 = 4.575f;
constexpr float kFuelDrivelineEfficiency = 0.94f;
// TODO: the paper's formula reads 2 * alpha0 for P >= 0, confirm against the source
constexpr float kFuelIdleMultiplier = 2.f;
constexpr float kGravity = 9.8066f;
constexpr float kDefaultFuelCostFactor = 90.f; // seconds per liter: 1.5 per liter / 60 per hour
constexpr float kDefaultFuelAlpha0 = 1.56e-3f; // L/s
constexpr float kDefaultFuelAlpha1 = 8.1e-5f;  // L/(s*kW)
constexpr float kDefaultFuelAlpha2 = 1e-8f;    // L/(s*kW^2)

// Turn costs based on side of street driving
constexpr float kRightSideTurnCosts[] = {kTCStraight,       kTCSlight,  kTCFavorable,
                                         kTCFavorableSharp, kTCReverse, kTCUnfavorableSharp,
                                         kTCUnfavorable,    kTCSlight};
constexpr float kLeftSideTurnCosts[] = {kTCStraight,         kTCSlight,  kTCUnfavorable,
                                        kTCUnfavorableSharp, kTCReverse, kTCFavorableSharp,
                                        kTCFavorable,        kTCSlight};

// How much to favor truck routes.
constexpr float kTruckRouteFactor = 0.85f;
constexpr float kDefaultUseTruckRoute = 0.0f;
constexpr float kMinNonTruckRouteFactor = 1.0f;

constexpr float kHighwayFactor[] = {
    1.0f, // Motorway
    0.5f, // Trunk
    0.0f, // Primary
    0.0f, // Secondary
    0.0f, // Tertiary
    0.0f, // Unclassified
    0.0f, // Residential
    0.0f  // Service, other
};

constexpr float kSurfaceFactor[] = {
    0.0f, // kPavedSmooth
    0.0f, // kPaved
    0.0f, // kPaveRough
    0.1f, // kCompacted
    0.2f, // kDirt
    0.5f, // kGravel
    1.0f  // kPath
};

// Valid ranges and defaults
constexpr ranged_default_t<float> kLowClassPenaltyRange{0.f, kDefaultLowClassPenalty, kMaxPenalty};
constexpr ranged_default_t<float> kLowClassFactorRange{kMinFactor, kDefaultLowClassFactor,
                                                       kMaxFactor};
constexpr ranged_default_t<float> kTruckAxleLoadRange{0.f, kDefaultTruckAxleLoad, 40.0f};
constexpr ranged_default_t<float> kUseTollsRange{0.f, kDefaultUseTolls, 1.0f};
constexpr ranged_default_t<uint32_t> kAxleCountRange{2, kDefaultAxleCount, 20};
constexpr ranged_default_t<float> kUseHighwaysRange{0.f, kDefaultUseHighways, 1.0f};
constexpr ranged_default_t<float> kTopSpeedRange{10.f, kMaxAssumedTruckSpeed, kMaxSpeedKph};
constexpr ranged_default_t<float> kHGVNoAccessRange{0.f, kMaxPenalty, kMaxPenalty};
constexpr ranged_default_t<float> kUseTruckRouteRange{0.f, kDefaultUseTruckRoute, 1.0f};
constexpr ranged_default_t<float> kFuelWeightRange{0.f, 0.f, 1.0f};
constexpr ranged_default_t<float> kFuelCostFactorRange{0.f, kDefaultFuelCostFactor, 10000.f};
constexpr ranged_default_t<float> kFuelAlpha0Range{0.f, kDefaultFuelAlpha0, 1.f};
constexpr ranged_default_t<float> kFuelAlpha1Range{0.f, kDefaultFuelAlpha1, 1.f};
constexpr ranged_default_t<float> kFuelAlpha2Range{0.f, kDefaultFuelAlpha2, 1.f};

BaseCostingOptionsConfig GetBaseCostOptsConfig() {
  BaseCostingOptionsConfig cfg{};
  // override defaults
  cfg.service_penalty_.def = kDefaultServicePenalty;
  cfg.use_tracks_.def = kDefaultUseTracks;
  cfg.use_living_streets_.def = kDefaultUseLivingStreets;
  cfg.height_.def = kDefaultTruckHeight;
  cfg.width_.def = kDefaultTruckWidth;
  cfg.length_.def = kDefaultTruckLength;
  cfg.weight_.def = kDefaultTruckWeight;
  return cfg;
}

const BaseCostingOptionsConfig kBaseCostOptsConfig = GetBaseCostOptsConfig();

// Fuel in liters per meter at a constant speed (km/h) on a grade (percent) for a mass in tonnes
float FuelPerMeter(const float speed,
                   const float grade,
                   const float weight,
                   const float alpha0,
                   const float alpha1,
                   const float alpha2) {
  // idle for kSpeedFactor[0] seconds per meter
  if (speed <= 0.f) {
    return alpha0 * kSpeedFactor[0];
  }
  const float mass = weight * 1000.f;
  const float resistance =
      kFuelAirDensity / 25.92f * kFuelDragCoefficient * kFuelFrontalArea * speed * speed +
      kGravity * mass * kFuelRollingCoefficient / 1000.f * (kFuelRollingC1 * speed + kFuelRollingC2) +
      kGravity * mass * grade * 0.01f;
  const float power = resistance * speed / (3600.f * kFuelDrivelineEfficiency);
  const float rate =
      power >= 0.f ? kFuelIdleMultiplier * alpha0 + alpha1 * power + alpha2 * power * power : alpha0;
  return rate * 3.6f / speed;
}

} // namespace

/**
 * Derived class providing dynamic edge costing for truck routes.
 */
class TruckCost : public DynamicCost {
public:
  /**
   * Construct truck costing. Pass in cost type and costing_options using protocol buffer(pbf).
   * @param  costing specified costing type.
   * @param  costing_options pbf with request costing_options.
   */
  TruckCost(const Costing& costing_options);

  virtual ~TruckCost();

  /**
   * Does the costing allow hierarchy transitions. Truck costing will allow
   * transitions by default.
   * @return  Returns true if the costing model allows hierarchy transitions).
   */
  virtual bool AllowTransitions() const;

  /**
   * Does the costing method allow multiple passes (with relaxed hierarchy
   * limits).
   * @return  Returns true if the costing model allows multiple passes.
   */
  virtual bool AllowMultiPass() const override;

  /**
   * Checks if access is allowed for the provided directed edge.
   * This is generally based on mode of travel and the access modes
   * allowed on the edge. However, it can be extended to exclude access
   * based on other parameters such as conditional restrictions and
   * conditional access that can depend on time and travel mode.
   * @param  edge                        Pointer to a directed edge.
   * @param  is_dest                     Is a directed edge the destination?
   * @param  pred                        Predecessor edge information.
   * @param  tile                        Current tile.
   * @param  edgeid                      GraphId of the directed edge.
   * @param  current_time                Current time (seconds since epoch). A value of 0
   *                                     indicates the route is not time dependent.
   * @param  tz_index                    timezone index for the node
   * @param  destonly_access_restr_mask  Mask containing access restriction types that had a
   * local traffic exemption at the start of the expansion. This mask will be mutated by eliminating
   * flags for locally exempt access restriction types that no longer exist on the passed edge
   *
   * @return Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::DirectedEdge* edge,
                       const bool is_dest,
                       const EdgeLabel& pred,
                       const graph_tile_ptr& tile,
                       const baldr::GraphId& edgeid,
                       const uint64_t current_time,
                       const uint32_t tz_index,
                       uint8_t& restriction_idx,
                       uint8_t& destonly_access_restr_mask) const override;

  /**
   * Checks if access is allowed for an edge on the reverse path
   * (from destination towards origin). Both opposing edges (current and
   * predecessor) are provided. The access check is generally based on mode
   * of travel and the access modes allowed on the edge. However, it can be
   * extended to exclude access based on other parameters such as conditional
   * restrictions and conditional access that can depend on time and travel
   * mode.
   * @param  edge                        Pointer to a directed edge.
   * @param  pred                        Predecessor edge information.
   * @param  opp_edge                    Pointer to the opposing directed edge.
   * @param  tile                        Current tile.
   * @param  edgeid                      GraphId of the opposing edge.
   * @param  current_time                Current time (seconds since epoch). A value of 0
   *                                     indicates the route is not time dependent.
   * @param  tz_index                    timezone index for the node
   * @param  destonly_access_restr_mask  Mask containing access restriction types that had a
   * local traffic exemption at the start of the expansion. This mask will be mutated by eliminating
   * flags for locally exempt access restriction types that no longer exist on the passed edge
   *
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool AllowedReverse(const baldr::DirectedEdge* edge,
                              const EdgeLabel& pred,
                              const baldr::DirectedEdge* opp_edge,
                              const graph_tile_ptr& tile,
                              const baldr::GraphId& opp_edgeid,
                              const uint64_t current_time,
                              const uint32_t tz_index,
                              uint8_t& restriction_idx,
                              uint8_t& destonly_access_restr_mask) const override;

  /**
   * Callback for Allowed doing mode  specific restriction checks
   */
  virtual bool ModeSpecificAllowed(const baldr::AccessRestriction& restriction) const override;

  /**
   * Only transit costings are valid for this method call, hence we throw
   * @param edge
   * @param departure
   * @param curr_time
   * @return
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge*,
                        const baldr::TransitDeparture*,
                        const uint32_t) const override {
    throw std::runtime_error("TruckCost::EdgeCost does not support transit edges");
  }

  /**
   * Get the cost to traverse the specified directed edge. Cost includes
   * the time (seconds) to traverse the edge.
   * @param  edge      Pointer to a directed edge.
   * @param  tile      Current tile.
   * @param  time_info Time info about edge passing.
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge* edge,
                        const baldr::GraphId& edgeid,
                        const graph_tile_ptr& tile,
                        const baldr::TimeInfo& time_info,
                        uint8_t& flow_sources) const override;

  /**
   * Returns the cost to make the transition from the predecessor edge.
   * Defaults to 0. Costing models that wish to include edge transition
   * costs (i.e., intersection/turn costs) must override this method.
   * @param  edge          Directed edge (the to edge)
   * @param  node          Node (intersection) where transition occurs.
   * @param  pred          Predecessor edge information.
   * @param  tile          Pointer to the graph tile containing the to edge.
   * @param  reader_getter Functor that facilitates access to a limited version of the graph reader
   * @return Returns the cost and time (seconds)
   */
  virtual Cost
  TransitionCost(const baldr::DirectedEdge* edge,
                 const baldr::NodeInfo* node,
                 const EdgeLabel& pred,
                 const graph_tile_ptr& tile,
                 const std::function<LimitedGraphReader()>& reader_getter) const override;

  /**
   * Returns the cost to make the transition from the predecessor edge
   * when using a reverse search (from destination towards the origin).
   * @param  idx                Directed edge local index
   * @param  node               Node (intersection) where transition occurs.
   * @param  pred               the opposing current edge in the reverse tree.
   * @param  edge               the opposing predecessor in the reverse tree
   * @param  tile               Graphtile that contains the node and the opp_edge
   * @param  edge_id            Graph ID of opp_pred_edge to get its tile if needed
   * @param  reader_getter      Functor that facilitates access to a limited version of the graph
   * reader
   * @param  has_measured_speed Do we have any of the measured speed types set?
   * @param  internal_turn      Did we make an turn on a short internal edge.
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost TransitionCostReverse(const uint32_t idx,
                                     const baldr::NodeInfo* node,
                                     const baldr::DirectedEdge* pred,
                                     const baldr::DirectedEdge* edge,
                                     const graph_tile_ptr& tile,
                                     const GraphId& pred_id,
                                     const std::function<LimitedGraphReader()>& reader_getter,
                                     const bool has_measured_speed,
                                     const InternalTurn internal_turn) const override;

  /**
   * Get the cost factor for A* heuristics. This factor is multiplied
   * with the distance to the destination to produce an estimate of the
   * minimum cost to the destination. The A* heuristic must underestimate the
   * cost to the destination. So a time based estimate based on speed should
   * assume the maximum speed is used to the destination such that the time
   * estimate is less than the least possible time along roads.
   */
  virtual float AStarCostFactor() const override;

  /**
   * Get the current travel type.
   * @return  Returns the current travel type.
   */
  virtual uint8_t travel_type() const override;

  /**
   * Function to be used in location searching which will
   * exclude and allow ranking results from the search by looking at each
   * edges attribution and suitability for use as a location by the travel
   * mode used by the costing method. It's also used to filter
   * edges not usable / inaccessible by truck.
   */
  bool Allowed(const baldr::DirectedEdge* edge,
               const graph_tile_ptr& tile,
               uint16_t disallow_mask = kDisallowNone) const override {
    bool allow_closures = (!filter_closures_ && !(disallow_mask & kDisallowClosure)) ||
                          !(flow_mask_ & kCurrentFlowMask);
    return DynamicCost::Allowed(edge, tile, disallow_mask) && !edge->bss_connection() &&
           (allow_closures || !tile->IsClosed(edge));
  }

public:
  VehicleType type_;        // Vehicle type: truck
  float toll_factor_;       // Factor applied when road has a toll
  float low_class_penalty_; // Penalty (seconds) to go to residential or service road
  float low_class_factor_;  // Factor applied when edge has low road class

  // Vehicle attributes (used for special restrictions and costing)
  bool hazmat_;                  // Carrying hazardous materials
  float weight_;                 // Vehicle weight in metric tons
  float axle_load_;              // Axle load weight in metric tons
  float height_;                 // Vehicle height in meters
  float width_;                  // Vehicle width in meters
  float length_;                 // Vehicle length in meters
  float highway_factor_;         // Factor applied when road is a motorway or trunk
  float non_truck_route_factor_; // Factor applied when road is not part of a designated truck route
  uint8_t axle_count_;           // Vehicle axle count

  // determine if we should allow hgv=no edges and penalize them instead
  float no_hgv_access_penalty_;

  // Weight of fuel versus time in the edge cost, 0 disables the fuel model
  float fuel_weight_;
  // Weighted fuel cost per meter indexed by weighted grade and speed (kph)
  std::vector<std::array<float, kMaxSpeedKph + 1>> fuel_cost_;
  // Lowest weighted fuel cost per meter at or below top speed, for the A* heuristic
  float min_fuel_cost_;
};

// Constructor
TruckCost::TruckCost(const Costing& costing)
    : DynamicCost(costing, TravelMode::kDrive, kTruckAccess, true) {
  const auto& costing_options = costing.options();

  type_ = VehicleType::kTruck;

  // Get the base costs
  get_base_costs(costing);

  low_class_penalty_ = costing_options.low_class_penalty();
  low_class_factor_ = costing_options.low_class_factor() - 1.f; // gets added to the base factor
  non_truck_route_factor_ =
      costing_options.use_truck_route() < 0.5f
          ? kMinNonTruckRouteFactor + 2.f * costing_options.use_truck_route()
          : ((kMinNonTruckRouteFactor - 5.f) + 12.f * costing_options.use_truck_route());

  // Get the vehicle attributes
  hazmat_ = costing_options.hazmat();
  weight_ = costing_options.weight();
  axle_load_ = costing_options.axle_load();
  height_ = costing_options.height();
  width_ = costing_options.width();
  length_ = costing_options.length();
  axle_count_ = costing_options.axle_count();

  // Create speed cost table
  // Preference to use highways. Is a value from 0 to 1
  // Factor for highway use - use a non-linear factor with values at 0.5 being neutral (factor
  // of 0). Values between 0.5 and 1 slowly decrease to a maximum of -0.125 (to slightly prefer
  // highways) while values between 0.5 to 0 slowly increase to a maximum of kMaxHighwayBiasFactor
  // to avoid/penalize highways.
  float use_highways = costing_options.use_highways();
  if (use_highways >= 0.5f) {
    float f = (0.5f - use_highways);
    highway_factor_ = f * f * f;
  } else {
    float f = 1.0f - (use_highways * 2.0f);
    highway_factor_ = kMaxHighwayBiasFactor * (f * f);
  }

  // Preference to use toll roads (separate from toll booth penalty). Sets a toll
  // factor. A toll factor of 0 would indicate no adjustment to weighting for toll roads.
  // use_tolls = 1 would reduce weighting slightly (a negative delta) while
  // use_tolls = 0 would penalize (positive delta to weighting factor).
  float use_tolls = costing_options.use_tolls();
  toll_factor_ = use_tolls < 0.5f ? (2.0f - 4 * use_tolls) : // ranges from 2 to 0
                     (0.5f - use_tolls) * 0.03f;             // ranges from 0 to -0.015

  // determine what to do with hgv=no edges
  bool no_hgv_access_penalty_active = !(costing_options.hgv_no_access_penalty() == kMaxPenalty);
  no_hgv_access_penalty_ = no_hgv_access_penalty_active * costing_options.hgv_no_access_penalty();
  // set the access mask to both car & truck if that penalty is active
  access_mask_ = no_hgv_access_penalty_active ? (kAutoAccess | kTruckAccess) : kTruckAccess;

  fuel_weight_ = costing_options.fuel_weight();
  min_fuel_cost_ = 0.f;
  if (fuel_weight_ > 0.f) {
    const float scale = fuel_weight_ * costing_options.fuel_cost_factor();
    min_fuel_cost_ = std::numeric_limits<float>::max();
    fuel_cost_.resize(kMaxGradeFactor + 1);
    for (uint32_t grade = 0; grade <= kMaxGradeFactor; ++grade) {
      // inverse of the weighted grade encoding in elevationbuilder.cc
      const float grade_pct = (static_cast<float>(grade) - 6.f) / 0.6f;
      for (uint32_t speed = 0; speed <= kMaxSpeedKph; ++speed) {
        fuel_cost_[grade][speed] =
            scale * FuelPerMeter(speed, grade_pct, weight_, costing_options.fuel_alpha0(),
                                 costing_options.fuel_alpha1(), costing_options.fuel_alpha2());
        if (speed > 0 && speed <= top_speed_) {
          min_fuel_cost_ = std::min(min_fuel_cost_, fuel_cost_[grade][speed]);
        }
      }
    }
  }
}

// Destructor
TruckCost::~TruckCost() {
}

// Auto costing will allow hierarchy transitions by default.
bool TruckCost::AllowTransitions() const {
  return true;
}

// Does the costing method allow multiple passes (with relaxed hierarchy
// limits).
bool TruckCost::AllowMultiPass() const {
  return true;
}

bool TruckCost::ModeSpecificAllowed(const baldr::AccessRestriction& restriction) const {
  switch (restriction.type()) {
    case AccessType::kHazmat:
      if (hazmat_ && !restriction.value()) {
        return false;
      }
      break;
    case AccessType::kMaxAxleLoad:
      if (axle_load_ > static_cast<float>(restriction.value() * 0.01)) {
        return false;
      }
      break;
    case AccessType::kMaxAxles:
      if (axle_count_ > static_cast<uint8_t>(restriction.value())) {
        return false;
      }
      break;
    case AccessType::kMaxHeight:
      if (height_ > static_cast<float>(restriction.value() * 0.01)) {
        return false;
      }
      break;
    case AccessType::kMaxLength:
      if (length_ > static_cast<float>(restriction.value() * 0.01)) {
        return false;
      }
      break;
    case AccessType::kMaxWeight:
      if (weight_ > static_cast<float>(restriction.value() * 0.01)) {
        return false;
      }
      break;
    case AccessType::kMaxWidth:
      if (width_ > static_cast<float>(restriction.value() * 0.01)) {
        return false;
      }
      break;
    default:
      return true;
  };
  return true;
}

// Check if access is allowed on the specified edge.
inline bool TruckCost::Allowed(const baldr::DirectedEdge* edge,
                               const bool is_dest,
                               const EdgeLabel& pred,
                               const graph_tile_ptr& tile,
                               const baldr::GraphId& edgeid,
                               const uint64_t current_time,
                               const uint32_t tz_index,
                               uint8_t& restriction_idx,
                               uint8_t& destonly_access_restr_mask) const {
  // Check access, U-turn, and simple turn restriction.
  if (!IsAccessible(edge) || (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx()) ||
      ((pred.restrictions() & (1 << edge->localedgeidx())) && (!ignore_turn_restrictions_)) ||
      edge->surface() == Surface::kImpassable || IsUserAvoidEdge(edgeid) ||
      (!allow_destination_only_ && !pred.destonly() && edge->destonly_hgv()) ||
      (pred.closure_pruning() && IsClosed(edge, tile)) ||
      (exclude_unpaved_ && !pred.unpaved() && edge->unpaved()) || CheckExclusions<true>(edge, pred)) {
    return false;
  }

  return DynamicCost::EvaluateRestrictions(access_mask_, edge, is_dest, tile, edgeid, current_time,
                                           tz_index, restriction_idx, destonly_access_restr_mask);
}

// Checks if access is allowed for an edge on the reverse path (from
// destination towards origin). Both opposing edges are provided.
bool TruckCost::AllowedReverse(const baldr::DirectedEdge* edge,
                               const EdgeLabel& pred,
                               const baldr::DirectedEdge* opp_edge,
                               const graph_tile_ptr& tile,
                               const baldr::GraphId& opp_edgeid,
                               const uint64_t current_time,
                               const uint32_t tz_index,
                               uint8_t& restriction_idx,
                               uint8_t& destonly_access_restr_mask) const {
  // Check access, U-turn, and simple turn restriction.
  if (!IsAccessible(opp_edge) || (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx()) ||
      ((opp_edge->restrictions() & (1 << pred.opp_local_idx())) && !ignore_turn_restrictions_) ||
      opp_edge->surface() == Surface::kImpassable || IsUserAvoidEdge(opp_edgeid) ||
      (!allow_destination_only_ && !pred.destonly() && opp_edge->destonly_hgv()) ||
      (pred.closure_pruning() && IsClosed(opp_edge, tile)) ||
      (exclude_unpaved_ && !pred.unpaved() && opp_edge->unpaved()) ||
      CheckExclusions<false>(opp_edge, pred)) {
    return false;
  }

  return DynamicCost::EvaluateRestrictions(access_mask_, opp_edge, false, tile, opp_edgeid,
                                           current_time, tz_index, restriction_idx,
                                           destonly_access_restr_mask);
}

// Get the cost to traverse the edge in seconds
Cost TruckCost::EdgeCost(const baldr::DirectedEdge* edge,
                         const baldr::GraphId& edgeid,
                         const graph_tile_ptr& tile,
                         const baldr::TimeInfo& time_info,
                         uint8_t& flow_sources) const {
  auto edge_speed = fixed_speed_ == baldr::kDisableFixedSpeed
                        ? tile->GetSpeed(edge, flow_mask_, time_info.second_of_week, true,
                                         &flow_sources, time_info.seconds_from_now)
                        : fixed_speed_;

  auto final_speed =
      std::min(edge_speed,
               edge->truck_speed() ? std::min(edge->truck_speed(), top_speed_) : top_speed_);

  float sec = edge->length() * kSpeedFactor[final_speed];

  if (shortest_) {
    return Cost(edge->length(), sec);
  }

  float cost = sec * inv_distance_factor_ + edge->length() * distance_factor_;
  float factor = 1.f;
  switch (edge->use()) {
    case Use::kFerry:
      factor = ferry_factor_;
      break;
    case Use::kRailFerry:
      factor = rail_ferry_factor_;
      break;
    default:
      factor = kDensityFactor[edge->density()];
      // the engine is off on ferries, so they keep a time based cost
      if (fuel_weight_ > 0.f) {
        cost = cost * (1.f - fuel_weight_) +
               edge->length() * fuel_cost_[edge->weighted_grade()][final_speed];
      }
      break;
  }

  factor += highway_factor_ * kHighwayFactor[static_cast<uint32_t>(edge->classification())] +
            kSurfaceFactor[static_cast<uint32_t>(edge->surface())] +
            SpeedPenalty(edge, tile, time_info, flow_sources, edge_speed) +
            edge->toll() * toll_factor_ +
            (low_class_factor_ * (edge->classification() >= baldr::RoadClass::kResidential));

  if (edge->truck_route() > 0) {
    factor *= kTruckRouteFactor;
  } else {
    factor *= non_truck_route_factor_;
  }

  if (edge->use() == Use::kTrack) {
    factor *= track_factor_;
  } else if (edge->use() == Use::kLivingStreet) {
    factor *= living_street_factor_;
  } else if (edge->use() == Use::kServiceRoad) {
    factor *= service_factor_;
  }

  factor *= EdgeFactor(edgeid);

  if (IsClosed(edge, tile)) {
    // Add a penalty for traversing a closed edge
    factor *= closure_factor_;
  }

  return Cost(cost * factor, sec);
}

// Returns the time (in seconds) to make the transition from the predecessor
Cost TruckCost::TransitionCost(const baldr::DirectedEdge* edge,
                               const baldr::NodeInfo* node,
                               const EdgeLabel& pred,
                               const graph_tile_ptr& /*tile*/,
                               const std::function<LimitedGraphReader()>& /*reader_getter*/) const {
  // Get the transition cost for country crossing, ferry, gate, toll booth,
  // destination only, alley, maneuver penalty
  uint32_t idx = pred.opp_local_idx();
  Cost c = base_transition_cost(node, edge, &pred, idx);
  c.secs += OSRMCarTurnDuration(edge, node, idx);

  // Penalty to transition onto low class roads.
  if (!shortest_ && pred.classification() <= baldr::RoadClass::kUnclassified &&
      edge->classification() >= baldr::RoadClass::kResidential) {
    c.cost += low_class_penalty_;
  }

  // Penalty if the request wants to avoid hgv=no edges instead of disallowing
  c.cost +=
      no_hgv_access_penalty_ * (pred.has_hgv_access() && !(edge->forwardaccess() & kTruckAccess));

  const auto stopimpact = edge->stopimpact(idx);
  const auto turntype = edge->turntype(idx);
  // Transition time = turncost * stopimpact * densityfactor
  if (stopimpact > 0 && !shortest_) {
    float turn_cost;
    if (edge->edge_to_right(idx) && edge->edge_to_left(idx)) {
      turn_cost = kTCCrossing;
    } else {
      turn_cost = (node->drive_on_right()) ? kRightSideTurnCosts[static_cast<uint32_t>(turntype)]
                                           : kLeftSideTurnCosts[static_cast<uint32_t>(turntype)];
    }

    if ((edge->use() != Use::kRamp && pred.use() == Use::kRamp) ||
        (edge->use() == Use::kRamp && pred.use() != Use::kRamp)) {
      turn_cost += kTCRamp;
      if (edge->roundabout())
        turn_cost += kTCRoundabout;
    }

    float seconds = turn_cost;

    bool has_left =
        (turntype == baldr::Turn::Type::kLeft || turntype == baldr::Turn::Type::kSharpLeft);
    bool has_right =
        (turntype == baldr::Turn::Type::kRight || turntype == baldr::Turn::Type::kSharpRight);
    bool has_reverse = turntype == baldr::Turn::Type::kReverse;
    bool is_turn = has_left || has_right || has_reverse;
    // Separate time and penalty when traffic is present. With traffic, edge speeds account for
    // much of the intersection transition time (TODO - evaluate different elapsed time settings).
    // Still want to add a penalty so routes avoid high cost intersections.
    if (is_turn) {
      seconds *= stopimpact;
    }

    AddUturnPenalty(idx, node, edge, has_reverse, has_left, has_right, true, pred.internal_turn(),
                    seconds);

    // Apply density factor and stop impact penalty if there isn't traffic on this edge or you're not
    // using traffic
    if (!pred.has_measured_speed()) {
      if (!is_turn)
        seconds *= stopimpact;
      seconds *= kTransDensityFactor[node->density()];
    }
    c.cost += seconds;
  }

  // Account for the user preferring distance
  c.cost *= inv_distance_factor_;
  return c;
}

// Returns the cost to make the transition from the predecessor edge
// when using a reverse search (from destination towards the origin).
// pred is the opposing current edge in the reverse tree
// edge is the opposing predecessor in the reverse tree
Cost TruckCost::TransitionCostReverse(const uint32_t idx,
                                      const baldr::NodeInfo* node,
                                      const baldr::DirectedEdge* pred,
                                      const baldr::DirectedEdge* edge,
                                      const graph_tile_ptr& /*tile*/,
                                      const GraphId& /*pred_id*/,
                                      const std::function<LimitedGraphReader()>& /*reader_getter*/,
                                      const bool has_measured_speed,
                                      const InternalTurn internal_turn) const {

  // TODO: do we want to update the cost if we have flow or speed from traffic.

  // Get the transition cost for country crossing, ferry, gate, toll booth,
  // destination only, alley, maneuver penalty
  Cost c = base_transition_cost(node, edge, pred, idx);
  c.secs += OSRMCarTurnDuration(edge, node, pred->opp_local_idx());

  // Penalty to transition onto low class roads.
  if (!shortest_ && pred->classification() <= baldr::RoadClass::kUnclassified &&
      edge->classification() >= baldr::RoadClass::kResidential) {
    c.cost += low_class_penalty_;
  }

  // Penalty if the request wants to avoid hgv=no edges instead of disallowing
  c.cost += no_hgv_access_penalty_ *
            ((pred->forwardaccess() & kTruckAccess) && !(edge->forwardaccess() & kTruckAccess));

  const auto stopimpact = edge->stopimpact(idx);
  const auto turntype = edge->turntype(idx);
  // Transition time = turncost * stopimpact * densityfactor
  if (stopimpact > 0 && !shortest_) {
    float turn_cost;
    if (edge->edge_to_right(idx) && edge->edge_to_left(idx)) {
      turn_cost = kTCCrossing;
    } else {
      turn_cost = (node->drive_on_right()) ? kRightSideTurnCosts[static_cast<uint32_t>(turntype)]
                                           : kLeftSideTurnCosts[static_cast<uint32_t>(turntype)];
    }

    if ((edge->use() != Use::kRamp && pred->use() == Use::kRamp) ||
        (edge->use() == Use::kRamp && pred->use() != Use::kRamp)) {
      turn_cost += kTCRamp;
      if (edge->roundabout())
        turn_cost += kTCRoundabout;
    }

    float seconds = turn_cost;
    bool has_left =
        (turntype == baldr::Turn::Type::kLeft || turntype == baldr::Turn::Type::kSharpLeft);
    bool has_right =
        (turntype == baldr::Turn::Type::kRight || turntype == baldr::Turn::Type::kSharpRight);
    bool has_reverse = turntype == baldr::Turn::Type::kReverse;

    bool is_turn = has_left || has_right || has_reverse;
    // Separate time and penalty when traffic is present. With traffic, edge speeds account for
    // much of the intersection transition time (TODO - evaluate different elapsed time settings).
    // Still want to add a penalty so routes avoid high cost intersections.
    if (is_turn) {
      seconds *= stopimpact;
    }

    AddUturnPenalty(idx, node, edge, has_reverse, has_left, has_right, true, internal_turn, seconds);

    // Apply density factor and stop impact penalty if there isn't traffic on this edge or you're not
    // using traffic
    if (!has_measured_speed) {
      if (!is_turn)
        seconds *= stopimpact;
      seconds *= kTransDensityFactor[node->density()];
    }
    c.cost += seconds;
  }

  // Account for the user preferring distance
  c.cost *= inv_distance_factor_;

  return c;
}

// Get the cost factor for A* heuristics. This factor is multiplied
// with the distance to the destination to produce an estimate of the
// minimum cost to the destination. The A* heuristic must underestimate the
// cost to the destination. So a time based estimate based on speed should
// assume the maximum speed is used to the destination such that the time
// estimate is less than the least possible time along roads.
float TruckCost::AStarCostFactor() const {
  float factor = kSpeedFactor[top_speed_];
  if (fuel_weight_ > 0.f) {
    // ferries keep a time based cost
    factor = std::min(factor, factor * (1.f - fuel_weight_) + min_fuel_cost_);
  }
  return factor * min_linear_cost_factor_;
}

// Returns the current travel type.
uint8_t TruckCost::travel_type() const {
  return static_cast<uint8_t>(type_);
}

void ParseTruckCostOptions(const rapidjson::Document& doc,
                           const std::string& costing_options_key,
                           Costing* c,
                           google::protobuf::RepeatedPtrField<CodedDescription>& warnings) {
  c->set_type(Costing::truck);
  c->set_name(Costing_Enum_Name(c->type()));
  auto* co = c->mutable_options();

  rapidjson::Value dummy;
  const auto& json = rapidjson::get_child(doc, costing_options_key.c_str(), dummy);

  ParseBaseCostOptions(json, c, kBaseCostOptsConfig, warnings);
  JSON_PBF_RANGED_DEFAULT(co, kLowClassPenaltyRange, json, "/low_class_penalty", low_class_penalty,
                          warnings);
  JSON_PBF_RANGED_DEFAULT(co, kLowClassFactorRange, json, "/low_class_factor", low_class_factor,
                          warnings);
  JSON_PBF_DEFAULT_V2(co, false, json, "/hazmat", hazmat);
  JSON_PBF_RANGED_DEFAULT(co, kTruckAxleLoadRange, json, "/axle_load", axle_load, warnings);
  JSON_PBF_RANGED_DEFAULT(co, kUseTollsRange, json, "/use_tolls", use_tolls, warnings);
  JSON_PBF_RANGED_DEFAULT(co, kUseHighwaysRange, json, "/use_highways", use_highways, warnings);
  JSON_PBF_RANGED_DEFAULT_V2(co, kAxleCountRange, json, "/axle_count", axle_count, warnings);
  JSON_PBF_RANGED_DEFAULT(co, kTopSpeedRange, json, "/top_speed", top_speed, warnings);
  JSON_PBF_RANGED_DEFAULT(co, kHGVNoAccessRange, json, "/hgv_no_access_penalty",
                          hgv_no_access_penalty, warnings);
  JSON_PBF_RANGED_DEFAULT_V2(co, kUseTruckRouteRange, json, "/use_truck_route", use_truck_route,
                             warnings);
  JSON_PBF_RANGED_DEFAULT(co, kFuelWeightRange, json, "/fuel_weight", fuel_weight, warnings);
  JSON_PBF_RANGED_DEFAULT(co, kFuelCostFactorRange, json, "/fuel_cost_factor", fuel_cost_factor,
                          warnings);
  JSON_PBF_RANGED_DEFAULT(co, kFuelAlpha0Range, json, "/fuel_alpha0", fuel_alpha0, warnings);
  JSON_PBF_RANGED_DEFAULT(co, kFuelAlpha1Range, json, "/fuel_alpha1", fuel_alpha1, warnings);
  JSON_PBF_RANGED_DEFAULT(co, kFuelAlpha2Range, json, "/fuel_alpha2", fuel_alpha2, warnings);
  // shortcuts have a flat weighted grade, so the fuel model needs the base graph
  if (co->fuel_weight() > 0.f && !rapidjson::get_optional<bool>(json, "/disable_hierarchy_pruning")) {
    co->set_disable_hierarchy_pruning(true);
  }
}

cost_ptr_t CreateTruckCost(const Costing& costing_options) {
  return std::make_shared<TruckCost>(costing_options);
}

} // namespace sif
} // namespace valhalla

/**********************************************************************************************/

#ifdef INLINE_TEST

using namespace valhalla;
using namespace sif;

namespace {

class TestTruckCost : public TruckCost {
public:
  TestTruckCost(const Costing& costing_options) : TruckCost(costing_options){};

  using TruckCost::alley_penalty_;
  using TruckCost::country_crossing_cost_;
  using TruckCost::destination_only_penalty_;
  using TruckCost::ferry_transition_cost_;
  using TruckCost::fuel_cost_;
  using TruckCost::gate_cost_;
  using TruckCost::maneuver_penalty_;
  using TruckCost::min_fuel_cost_;
  using TruckCost::service_factor_;
  using TruckCost::service_penalty_;
  using TruckCost::toll_booth_cost_;
};

TestTruckCost* make_truckcost_from_json(const std::string& property, float testVal) {
  std::stringstream ss;
  ss << R"({"costing": "truck", "costing_options":{"truck":{")" << property << R"(":)" << testVal
     << "}}}";
  Api request;
  ParseApi(ss.str(), valhalla::Options::route, request);
  return new TestTruckCost(request.options().costings().find(Costing::truck)->second);
}

std::uniform_real_distribution<float>*
make_distributor_from_range(const ranged_default_t<float>& range) {
  float rangeLength = range.max - range.min;
  return new std::uniform_real_distribution<float>(range.min - rangeLength, range.max + rangeLength);
}

TEST(TruckCost, testTruckCostParams) {
  constexpr unsigned testIterations = 250;
  constexpr unsigned seed = 0;
  std::mt19937 generator(seed);
  std::shared_ptr<std::uniform_real_distribution<float>> distributor;
  std::shared_ptr<TestTruckCost> ctorTester;

  const auto& defaults = kBaseCostOptsConfig;

  // maneuver_penalty_
  distributor.reset(make_distributor_from_range(defaults.maneuver_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("maneuver_penalty", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->maneuver_penalty_,
                test::IsBetween(defaults.maneuver_penalty_.min, defaults.maneuver_penalty_.max));
  }

  // alley_penalty_
  distributor.reset(make_distributor_from_range(defaults.alley_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("alley_penalty", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->alley_penalty_,
                test::IsBetween(defaults.alley_penalty_.min, defaults.alley_penalty_.max));
  }

  // destination_only_penalty_
  distributor.reset(make_distributor_from_range(defaults.dest_only_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("destination_only_penalty", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->destination_only_penalty_,
                test::IsBetween(defaults.dest_only_penalty_.min, defaults.dest_only_penalty_.max));
  }

  // gate_cost_ (Cost.secs)
  distributor.reset(make_distributor_from_range(defaults.gate_cost_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("gate_cost", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->gate_cost_.secs,
                test::IsBetween(defaults.gate_cost_.min, defaults.gate_cost_.max));
  }

  // gate_penalty_ (Cost.cost)
  distributor.reset(make_distributor_from_range(defaults.gate_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("gate_penalty", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->gate_cost_.cost,
                test::IsBetween(defaults.gate_penalty_.min, defaults.gate_penalty_.max));
  }

  // tollbooth_cost_ (Cost.secs)
  distributor.reset(make_distributor_from_range(defaults.toll_booth_cost_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("toll_booth_cost", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->toll_booth_cost_.secs,
                test::IsBetween(defaults.toll_booth_cost_.min, defaults.toll_booth_cost_.max));
  }

  // tollbooth_penalty_ (Cost.cost)
  distributor.reset(make_distributor_from_range(defaults.toll_booth_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("toll_booth_penalty", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->toll_booth_cost_.cost,
                test::IsBetween(defaults.toll_booth_penalty_.min,
                                defaults.toll_booth_penalty_.max + defaults.toll_booth_cost_.def));
  }

  // country_crossing_cost_ (Cost.secs)
  distributor.reset(make_distributor_from_range(defaults.country_crossing_cost_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("country_crossing_cost", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->country_crossing_cost_.secs,
                test::IsBetween(defaults.country_crossing_cost_.min,
                                defaults.country_crossing_cost_.max));
  }

  // country_crossing_penalty_ (Cost.cost)
  distributor.reset(make_distributor_from_range(defaults.country_crossing_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("country_crossing_penalty", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->country_crossing_cost_.cost,
                test::IsBetween(defaults.country_crossing_penalty_.min,
                                defaults.country_crossing_penalty_.max +
                                    defaults.country_crossing_cost_.def));
  }

  // ferry_transition_cost_ (Cost.secs)
  distributor.reset(make_distributor_from_range(defaults.ferry_cost_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("ferry_cost", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->ferry_transition_cost_.secs,
                test::IsBetween(defaults.ferry_cost_.min, defaults.ferry_cost_.max));
  }

  // low_class_penalty_
  distributor.reset(make_distributor_from_range(kLowClassPenaltyRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("low_class_penalty", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->low_class_penalty_,
                test::IsBetween(kLowClassPenaltyRange.min, kLowClassPenaltyRange.max));
  }

  // service_penalty_
  distributor.reset(make_distributor_from_range(defaults.service_penalty_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("service_penalty", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->service_penalty_,
                test::IsBetween(defaults.service_penalty_.min, defaults.service_penalty_.max));
  }

  // service_factor_
  distributor.reset(make_distributor_from_range(defaults.service_factor_));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("service_factor", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->service_factor_,
                test::IsBetween(defaults.service_factor_.min, defaults.service_factor_.max));
  }

  // axle_load_
  distributor.reset(make_distributor_from_range(kTruckAxleLoadRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_truckcost_from_json("axle_load", (*distributor)(generator)));
    EXPECT_THAT(ctorTester->axle_load_,
                test::IsBetween(kTruckAxleLoadRange.min, kTruckAxleLoadRange.max));
  }
}

TEST(TruckCost, FuelPerMeterMatchesReference) {
  // control values from the Python reference model: {weight t, grade code, speed kph, L/m}
  const std::vector<std::tuple<float, uint32_t, float, float>> cases = {
      {10.f, 0, 30.f, 1.872e-4f}, {10.f, 6, 30.f, 4.618191e-4f}, {10.f, 15, 80.f, 1.768453e-3f},
      {40.f, 0, 80.f, 7.02e-5f},  {40.f, 6, 80.f, 6.566943e-4f}, {40.f, 15, 30.f, 6.069433e-3f},
  };
  for (const auto& [weight, grade, speed, expected] : cases) {
    const float grade_pct = (static_cast<float>(grade) - 6.f) / 0.6f;
    EXPECT_NEAR(FuelPerMeter(speed, grade_pct, weight, kDefaultFuelAlpha0, kDefaultFuelAlpha1,
                             kDefaultFuelAlpha2),
                expected, expected * 1e-4f)
        << weight << " t, grade " << grade << ", " << speed << " kph";
  }
}

TEST(TruckCost, FuelTable) {
  Api request;
  ParseApi(R"({"costing":"truck","costing_options":{"truck":{"fuel_weight":1}}})",
           valhalla::Options::route, request);
  const auto& costing = request.options().costings().find(Costing::truck)->second;
  EXPECT_TRUE(costing.options().disable_hierarchy_pruning());
  TestTruckCost light(costing);

  ParseApi(R"({"costing":"truck","costing_options":{"truck":{"fuel_weight":1,"weight":40}}})",
           valhalla::Options::route, request);
  TestTruckCost heavy(request.options().costings().find(Costing::truck)->second);

  ASSERT_EQ(light.fuel_cost_.size(), kMaxGradeFactor + 1);
  for (uint32_t speed = 1; speed <= kMaxAssumedTruckSpeed; ++speed) {
    for (uint32_t grade = 1; grade <= kMaxGradeFactor; ++grade) {
      EXPECT_GE(light.fuel_cost_[grade][speed], light.fuel_cost_[grade - 1][speed]);
      // downhill a heavier truck coasts (P < 0) where a lighter one still needs power
      if (grade >= 6) {
        EXPECT_GE(heavy.fuel_cost_[grade][speed], light.fuel_cost_[grade][speed]);
      }
      // the heuristic must never exceed the cost of any edge
      EXPECT_LE(light.AStarCostFactor(), light.fuel_cost_[grade][speed]);
    }
  }
  EXPECT_GT(light.min_fuel_cost_, 0.f);

  // the fuel model is off by default and does not touch hierarchy pruning
  ParseApi(R"({"costing":"truck"})", valhalla::Options::route, request);
  const auto& off = request.options().costings().find(Costing::truck)->second;
  EXPECT_FALSE(off.options().disable_hierarchy_pruning());
  EXPECT_TRUE(TestTruckCost(off).fuel_cost_.empty());
}
} // namespace

#endif
