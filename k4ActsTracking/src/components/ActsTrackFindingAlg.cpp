#include <GaudiAlg/GaudiTool.h>
#include <GaudiKernel/ToolHandle.h>

#include <edm4hep/MutableTrack.h>
#include <edm4hep/TrackerHitPlaneCollection.h>
#include <k4FWCore/DataHandle.h>

#include <Acts/EventData/MultiTrajectory.hpp>
#include <Acts/Geometry/TrackingGeometry.hpp>
#include <Acts/MagneticField/MagneticFieldContext.hpp>
#include <Acts/MagneticField/MagneticFieldProvider.hpp>
#include <Acts/Propagator/EigenStepper.hpp>
#include <Acts/Propagator/Navigator.hpp>
#include <Acts/Propagator/Propagator.hpp>
#include <Acts/Surfaces/PerigeeSurface.hpp>
#include <Acts/TrackFinding/CombinatorialKalmanFilter.hpp>
#include <Acts/TrackFinding/MeasurementSelector.hpp>
#include <Acts/TrackFitting/GainMatrixSmoother.hpp>
#include <Acts/TrackFitting/GainMatrixUpdater.hpp>

#include "IActsTrackFindingAlg.h"
#include "MeasurementCalibrator.h"
#include "SeedSpacePoint.h"

using namespace Acts::UnitLiterals;

class ActsTrackFindingAlg final : public GaudiTool, virtual public IActsTrackFindingAlg {
public:
  explicit ActsTrackFindingAlg(const std::string& type, const std::string& name, const IInterface* parent)
      : GaudiTool(type, name, parent),
        output_seeds_{"seedFindings", Gaudi::DataHandle::Writer, this},
        output_tracks_{"trackFindings", Gaudi::DataHandle::Writer, this},
        propagate_backward_{this, "propagateBackward", false},
        prop_max_steps_{this, "propagatorMaxSteps", 10'000},
        ckf_chi2_cutoff_{this, "chi2Cutoff", 15., "Maximum local chi2 contribution."},
        ckf_num_measurements_cutoff_{this, "numMeasurementsCutoff", 10,
                                     "Maximum number of associated measurements on a single surface."} {}

  inline StatusCode initialize() override {
    if (const auto ret = GaudiTool::initialize(); ret.isFailure())
      return ret;
    Navigator::Config navigator_config(tracking_geom_);
    navigator_config.resolvePassive   = false;
    navigator_config.resolveMaterial  = true;
    navigator_config.resolveSensitive = true;
    Navigator navigator(navigator_config);

    // construct all components for the fitter
    Stepper    stepper(magnetic_field_);
    Propagator propagator(std::move(stepper), std::move(navigator));
    track_finder_ = std::make_unique<CKF>(std::move(propagator));

    Acts::MeasurementSelector::Config measurement_selector_config = {
        {Acts::GeometryIdentifier(),
         {{}, {ckf_chi2_cutoff_}, {static_cast<std::size_t>(ckf_num_measurements_cutoff_)}}}};
    measurement_selector_ = std::make_unique<Acts::MeasurementSelector>(measurement_selector_config);

    return StatusCode::SUCCESS;
  }

  inline StatusCode run(const edm4hep::TrackerHitPlaneCollection& tracker_hits) const override {
    std::vector<std::pair<Acts::GeometryIdentifier, edm4hep::TrackerHitPlane>> sorted_hits;
    for (const auto& hit : tracker_hits)
      sorted_hits.push_back(std::make_pair(geometry_id_mapping_(hit), hit));

    // sort by GeoID
    std::sort(sorted_hits.begin(), sorted_hits.end(),
              [](const auto& lhs, const auto& rhs) -> bool { return lhs.first < rhs.first; });

    k4ActsTracking::MeasurementContainer    measurements;
    k4ActsTracking::SourceLinkContainer     source_links;
    k4ActsTracking::SeedSpacePointContainer space_points;

    source_links.reserve(sorted_hits.size());
    for (const auto& [geo_id, hit] : sorted_hits) {
      const auto* surface = tracking_geom_->findSurface(geo_id);
      if (!surface)
        throw std::runtime_error("Surface not found");

      const auto&   edm_global_pos = hit.getPosition();
      Acts::Vector3 global_pos{edm_global_pos.x, edm_global_pos.y, edm_global_pos.z};

      Acts::Result<Acts::Vector2> lp_result = surface->globalToLocal(*geometry_context_, global_pos, {0, 0, 0}, 0.5_um);
      if (!lp_result.ok())
        throw std::runtime_error("Global to local transformation did not succeed.");

      Acts::Vector2 loc = lp_result.value();

      Acts::SquareMatrix2 local_cov = Acts::SquareMatrix2::Zero();
      local_cov(0, 0)               = std::pow(hit.getDu() * Acts::UnitConstants::mm, 2);
      local_cov(1, 1)               = std::pow(hit.getDv() * Acts::UnitConstants::mm, 2);

      k4ActsTracking::SourceLink source_link(surface->geometryId(), measurements.size(), &hit);
      Acts::SourceLink           src_wrap{source_link};
      Acts::Measurement meas = Acts::makeMeasurement(src_wrap, loc, local_cov, Acts::eBoundLoc0, Acts::eBoundLoc1);

      measurements.push_back(meas);
      source_links.emplace_hint(source_links.end(), source_link);
    }

    Acts::GainMatrixUpdater               kf_updater;
    Acts::GainMatrixSmoother              kf_smoother;
    k4ActsTracking::MeasurementCalibrator measurement_calibrator{measurements};

    Acts::CombinatorialKalmanFilterExtensions<Acts::VectorMultiTrajectory> extensions;
    extensions.calibrator.connect<&k4ActsTracking::MeasurementCalibrator::calibrate>(&measurement_calibrator);
    extensions.updater.connect<&Acts::GainMatrixUpdater::operator()<Acts::VectorMultiTrajectory>>(&kf_updater);
    extensions.smoother.connect<&Acts::GainMatrixSmoother::operator()<Acts::VectorMultiTrajectory>>(&kf_smoother);
    extensions.measurementSelector.connect<&Acts::MeasurementSelector::select<Acts::VectorMultiTrajectory>>(
        measurement_selector_.get());

    Acts::PropagatorPlainOptions propagator_options;
    propagator_options.maxSteps = prop_max_steps_;
    if (propagate_backward_)
      propagator_options.direction = Acts::Direction::Backward;

    auto perigee_surface = Acts::Surface::makeShared<Acts::PerigeeSurface>(Acts::Vector3{0., 0., 0.});

    k4ActsTracking::SourceLinkAccessor source_link_accessor;
    source_link_accessor.container = &source_links;

    Acts::SourceLinkAccessorDelegate<k4ActsTracking::SourceLinkAccessor::Iterator> source_link_accessor_delegate;
    source_link_accessor_delegate.connect<&k4ActsTracking::SourceLinkAccessor::range>(&source_link_accessor);
    /*CKFOptions ckf_options(geometry_context_, magnetic_field_context_, calibration_context_,
                           source_link_accessor_delegate, extensions, propagator_options, perigee_surface.get());*/

    return StatusCode::SUCCESS;
  }

  void setGeometryIdMappingTool(std::function<uint64_t(const edm4hep::TrackerHitPlane&)> mapping) override {
    geometry_id_mapping_ = mapping;
  }
  void setCalibrationContext(Acts::CalibrationContext* context) override { calibration_context_.reset(context); }
  void setGeometryContext(Acts::GeometryContext* context) override { geometry_context_.reset(context); }
  void setTrackingGeometry(const Acts::TrackingGeometry* geom) override { tracking_geom_.reset(geom); }
  void setMagneticField(const Acts::MagneticFieldProvider* magfield) override { magnetic_field_.reset(magfield); }
  void setMagneticFieldContext(Acts::MagneticFieldContext* magfield_context) override {
    magnetic_field_context_.reset(magfield_context);
  }

private:
  using Updater    = Acts::GainMatrixUpdater;
  using Smoother   = Acts::GainMatrixSmoother;
  using Stepper    = Acts::EigenStepper<>;
  using Navigator  = Acts::Navigator;
  using Propagator = Acts::Propagator<Stepper, Navigator>;
  using CKF        = Acts::CombinatorialKalmanFilter<Propagator, Acts::VectorMultiTrajectory>;
  using CKFOptions =
      Acts::CombinatorialKalmanFilterOptions<k4ActsTracking::SourceLinkAccessor::Iterator, Acts::VectorMultiTrajectory>;

  mutable DataHandle<edm4hep::TrackCollection> output_seeds_;
  mutable DataHandle<edm4hep::TrackCollection> output_tracks_;

  Gaudi::Property<bool>   propagate_backward_;
  Gaudi::Property<int>    prop_max_steps_;
  Gaudi::Property<double> ckf_chi2_cutoff_;
  Gaudi::Property<int>    ckf_num_measurements_cutoff_;

  std::function<uint64_t(const edm4hep::TrackerHitPlane&)> geometry_id_mapping_{nullptr};
  std::shared_ptr<Acts::CalibrationContext>                calibration_context_{nullptr};
  std::shared_ptr<Acts::GeometryContext>                   geometry_context_{nullptr};
  std::shared_ptr<const Acts::TrackingGeometry>            tracking_geom_{nullptr};
  std::shared_ptr<const Acts::MagneticFieldProvider>       magnetic_field_{nullptr};
  std::shared_ptr<Acts::MagneticFieldContext>              magnetic_field_context_{nullptr};

  std::unique_ptr<CKF>                       track_finder_;
  std::unique_ptr<Acts::MeasurementSelector> measurement_selector_;
};

DECLARE_COMPONENT(ActsTrackFindingAlg)
