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

#ifndef IActsTrackFindingAlg_h
#define IActsTrackFindingAlg_h

#include <GaudiKernel/IAlgTool.h>

#include <Acts/Geometry/GeometryContext.hpp>
#include <Acts/Utilities/CalibrationContext.hpp>

namespace edm4hep {
  class TrackerHitPlane;
  class TrackerHitPlaneCollection;
}  // namespace edm4hep

namespace Acts {
  class MagneticFieldProvider;
  class TrackingGeometry;
}  // namespace Acts

class GAUDI_API IActsTrackFindingAlg : virtual public IAlgTool {
public:
  DeclareInterfaceID(IActsTrackFindingAlg, 1, 0);

  virtual StatusCode run(const edm4hep::TrackerHitPlaneCollection& tracker_hits) const = 0;

  virtual void setGeometryIdMappingTool(std::function<uint64_t(const edm4hep::TrackerHitPlane&)>) = 0;
  virtual void setCalibrationContext(Acts::CalibrationContext*)                                   = 0;
  virtual void setGeometryContext(Acts::GeometryContext*)                                         = 0;
  virtual void setTrackingGeometry(const Acts::TrackingGeometry*)                                 = 0;
  virtual void setMagneticField(const Acts::MagneticFieldProvider*)                               = 0;
  virtual void setMagneticFieldContext(Acts::MagneticFieldContext*)                               = 0;
};

#endif
