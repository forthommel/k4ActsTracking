#include <Acts/MagneticField/MagneticFieldContext.hpp>
#include <Acts/MagneticField/MagneticFieldProvider.hpp>

#include <DD4hep/DetFactoryHelper.h>
#include <DD4hep/Factories.h>

#include <edm4hep/TrackerHitPlaneCollection.h>

#include "ActsAlg.h"
#include "IActsGeoSvc.h"

DECLARE_COMPONENT(ActsAlg)

ActsAlg::ActsAlg(const std::string& aName, ISvcLocator* aSvcLoc)
    : GaudiAlgorithm(aName, aSvcLoc), input_hits_{"inputs", Gaudi::DataHandle::Reader, this} {
  declareProperty("trackFindingAlgorithm", finding_algo_, "track finding algorithm");
  declareProperty("trackFittingAlgorithm", fitting_algo_, "track fitting algorithm");
}

ActsAlg::~ActsAlg() {}

StatusCode ActsAlg::initialize() {
  info() << "Initializing the ACTS algorithm" << endmsg;
  int                 argc = 0;
  char*               argv = {};
  dd4hep::PluginDebug dbg;
  const std::string   name = "k4acts_addActsExtensions";
  if (const auto result = dd4hep::PluginService::Create<long>(name, &dd4hep::Detector::getInstance(), argc, &argv);
      result == 0) {
    error() << "Failed to locate plugin " << name << "\n" << dbg.missingFactory(name) << endmsg;
    return StatusCode::FAILURE;
  }

  info() << "Successfully loaded the plugin!" << endmsg;

  auto geom_svc = svc<IActsGeoSvc>("ActsGeoSvc", true);
  if (!geom_svc)
    return Error("Failed to retrieve the Acts geometry service.");
  tracking_geom_ = &geom_svc->trackingGeometry();

  // parameterise the track finding algorithm
  if (const auto ret = finding_algo_->initialize(); ret.isFailure())
    return ret;
  finding_algo_->setGeometryIdMappingTool([](const edm4hep::TrackerHitPlane&) -> uint64_t { return 0; });
  finding_algo_->setCalibrationContext(new Acts::CalibrationContext{});
  finding_algo_->setGeometryContext(new Acts::GeometryContext{});
  finding_algo_->setTrackingGeometry(tracking_geom_);
  finding_algo_->setMagneticField(&geom_svc->magneticField());
  finding_algo_->setMagneticFieldContext(new Acts::MagneticFieldContext{});

  // parameterise the track fitting algorithm
  if (const auto ret = fitting_algo_->initialize(); ret.isFailure())
    return ret;
  fitting_algo_->setTrackingGeometry(tracking_geom_);

  return StatusCode::SUCCESS;
}

StatusCode ActsAlg::execute() {
  if (const auto ret = finding_algo_->run(*input_hits_.get()); ret.isFailure())
    return ret;
  if (const auto ret = fitting_algo_->run(); ret.isFailure())
    return ret;
  return StatusCode::SUCCESS;
}

StatusCode ActsAlg::finalize() {
  if (const auto ret = finding_algo_->finalize(); ret.isFailure())
    return ret;
  if (const auto ret = fitting_algo_->finalize(); ret.isFailure())
    return ret;
  return StatusCode::SUCCESS;
}
