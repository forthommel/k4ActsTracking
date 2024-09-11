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

#ifndef ACTSGEOSVC_H
#define ACTSGEOSVC_H

#include <DD4hep/Detector.h>
#include <DDRec/Surface.h>
#include <DDRec/SurfaceManager.h>
#include <GaudiKernel/MsgStream.h>
#include <GaudiKernel/Service.h>

#include <Acts/Geometry/GeometryContext.hpp>
#include <Acts/Geometry/TrackingGeometry.hpp>
#include <Acts/Material/IMaterialDecorator.hpp>
#include <Acts/Surfaces/Surface.hpp>
#include <Acts/Utilities/Logger.hpp>

#include "IActsGeoSvc.h"

class ActsGeoSvc : public extends<Service, IActsGeoSvc> {
public:
  explicit ActsGeoSvc(const std::string& name, ISvcLocator* svc);
  virtual ~ActsGeoSvc() = default;

  StatusCode initialize() override;

  inline const Acts::TrackingGeometry&      trackingGeometry() const override { return *tracking_geometry_; }
  inline const Acts::MagneticFieldProvider& magneticField() const override { return *magnetic_field_; }

  StatusCode createGeoObj();

  using VolumeSurfaceMap = std::unordered_map<uint64_t, const Acts::Surface*>;

private:
  Gaudi::Property<std::string> geo_service_name_;
  Gaudi::Property<bool>        debug_geometry_;    ///< Option for the Debug Geometry
  Gaudi::Property<std::string> output_file_name_;  ///< Output file name

  dd4hep::Detector*                        dd4hep_detector_{nullptr};
  std::map<int64_t, dd4hep::rec::Surface*> dd4hep_surfaces_map_;  ///< DD4hep surface map

  std::unique_ptr<const Acts::TrackingGeometry>      tracking_geometry_{nullptr};  ///< ACTS Tracking Geometry
  std::unique_ptr<const Acts::MagneticFieldProvider> magnetic_field_{nullptr};     ///< ACTS Magnetic field provider

  Acts::GeometryContext geometry_context_;  ///< ACTS Tracking Geometry Context
  VolumeSurfaceMap      surfaces_map_;  ///< ACTS surface lookup container for hit surfaces that generate smeared hits
  std::shared_ptr<const Acts::IMaterialDecorator> material_decorator_{nullptr};  ///< ACTS Material Decorator

  MsgStream            log_;                                      ///< Gaudi logging output
  Acts::Logging::Level acts_logging_level_{Acts::Logging::INFO};  ///< ACTS Logging Level
};

#endif
