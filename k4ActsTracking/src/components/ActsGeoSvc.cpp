/*
 * Copyright (c) 2014-2024 Key4hep-Project.
 *
 * This file is part of Key4hep.
 * See https://key4hep.github.io/key4hep-doc/ for further info.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <DD4hep/Printout.h>
#include <GaudiKernel/Service.h>
#include <k4Interface/IGeoSvc.h>

#include <Acts/Geometry/TrackingGeometry.hpp>
#include <Acts/MagneticField/MagneticFieldContext.hpp>
#include <Acts/Plugins/DD4hep/ConvertDD4hepDetector.hpp>
#include <Acts/Plugins/DD4hep/DD4hepFieldAdapter.hpp>
#include <Acts/Surfaces/PlaneSurface.hpp>
#include <Acts/Utilities/Logger.hpp>
#include <Acts/Visualization/GeometryView3D.hpp>
#include <Acts/Visualization/ObjVisualization3D.hpp>

#include "ActsGeoSvc.h"

using namespace Gaudi;

DECLARE_COMPONENT(ActsGeoSvc)

ActsGeoSvc::ActsGeoSvc(const std::string& name, ISvcLocator* svc)
    : base_class(name, svc),
      geo_service_name_{this, "GeoSvcName", "GeoSvc", "The name of the GeoSvc instance"},
      debug_geometry_{this, "debugGeometry", false, "Option for geometry debugging"},
      output_file_name_{this, "outputFileName", "", "Output file name"},
      log_(msgSvc(), name) {}

StatusCode ActsGeoSvc::initialize() {
  dd4hep_detector_ = svcLocator()->service<IGeoSvc>(geo_service_name_)->getDetector();
  // necessary?
  // dd4hep_detector_->addExtension<IActsGeoSvc>(this);

  Acts::BinningType bTypePhi              = Acts::equidistant;
  Acts::BinningType bTypeR                = Acts::equidistant;
  Acts::BinningType bTypeZ                = Acts::equidistant;
  double            layerEnvelopeR        = Acts::UnitConstants::mm;
  double            layerEnvelopeZ        = Acts::UnitConstants::mm;
  double            defaultLayerThickness = Acts::UnitConstants::fm;
  using Acts::sortDetElementsByID;
  auto logger        = Acts::getDefaultLogger("k4ActsTracking", acts_logging_level_);
  tracking_geometry_ = Acts::convertDD4hepDetector(dd4hep_detector_->world(), *logger, bTypePhi, bTypeR, bTypeZ,
                                                   layerEnvelopeR, layerEnvelopeZ, defaultLayerThickness,
                                                   sortDetElementsByID, geometry_context_, material_decorator_);
  magnetic_field_    = std::make_unique<Acts::DD4hepFieldAdapter>(dd4hep_detector_->field());

  /// Setting geometry debug option
  if (debug_geometry_) {
    log_ << MSG::INFO << "Geometry debugging is ON." << endmsg;

    if (createGeoObj().isFailure()) {
      log_ << MSG::ERROR << "Could not create geometry OBJ" << endmsg;
      return StatusCode::FAILURE;
    } else
      log_ << MSG::INFO << "Geometry OBJ SUCCESSFULLY created" << endmsg;
  } else {
    log_ << MSG::VERBOSE << "Geometry debugging is OFF." << endmsg;
    return StatusCode::SUCCESS;
  }
  std::cout << "works!" << std::endl;

  return StatusCode::SUCCESS;
}

/// Create a geometry OBJ file
StatusCode ActsGeoSvc::createGeoObj() {
  // Convert DD4Hep geometry to acts
  Acts::ObjVisualization3D object_vis;

  if (!tracking_geometry_)
    return StatusCode::FAILURE;
  tracking_geometry_->visitSurfaces([&](const Acts::Surface* surface) {
    if (!surface) {
      warning() << "no surface???" << endmsg;
      return;
    }
    Acts::GeometryView3D::drawSurface(object_vis, *surface, geometry_context_);
  });
  object_vis.write(output_file_name_);
  log_ << MSG::INFO << output_file_name_ << " SUCCESSFULLY written." << endmsg;

  return StatusCode::SUCCESS;
}
