#include <GaudiAlg/GaudiTool.h>
#include <GaudiKernel/ToolHandle.h>

#include <Acts/Geometry/TrackingGeometry.hpp>

#include "IActsTrackRecoAlg.h"

class ActsTrackRecoAlg final : public GaudiTool, virtual public IActsTrackRecoAlg {
public:
  explicit ActsTrackRecoAlg(const std::string& type, const std::string& name, const IInterface* parent)
      : GaudiTool(type, name, parent) {}

  inline StatusCode initialize() override {
    if (const auto ret = GaudiTool::initialize(); ret.isFailure())
      return ret;
    return StatusCode::SUCCESS;
  }
  inline StatusCode run() const override { return StatusCode::SUCCESS; }
  inline StatusCode finalize() override {
    if (const auto ret = GaudiTool::finalize(); ret.isFailure())
      return ret;
    return StatusCode::SUCCESS;
  }

  void setTrackingGeometry(const Acts::TrackingGeometry* geom) { tracking_geom_ = geom; }

private:
  const Acts::TrackingGeometry* tracking_geom_{nullptr};
};

DECLARE_COMPONENT(ActsTrackRecoAlg)
