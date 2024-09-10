#include <DD4hep/DetFactoryHelper.h>
#include <DD4hep/Factories.h>

#include "ActsAlg.h"
#include "IActsGeoSvc.h"

DECLARE_COMPONENT(ActsAlg)

ActsAlg::ActsAlg(const std::string& aName, ISvcLocator* aSvcLoc) : GaudiAlgorithm(aName, aSvcLoc) {
  declareProperty("algorithm", algo_, "track reconstruction algorithm");
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
  if (const auto ret = algo_->initialize(); ret.isFailure())
    return ret;
  algo_->setTrackingGeometry(tracking_geom_);

  return StatusCode::SUCCESS;
}

StatusCode ActsAlg::execute() {
  if (const auto ret = algo_->run(); ret.isFailure())
    return ret;
  return StatusCode::SUCCESS;
}

StatusCode ActsAlg::finalize() {
  if (const auto ret = algo_->finalize(); ret.isFailure())
    return ret;
  return StatusCode::SUCCESS;
}
