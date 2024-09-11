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

#ifndef k4ActsTracking_ActsAlg_h
#define k4ActsTracking_ActsAlg_h

#include <Gaudi/Property.h>
#include <GaudiAlg/GaudiAlgorithm.h>
#include <GaudiKernel/ToolHandle.h>

#include <k4FWCore/DataHandle.h>

#include <Acts/Geometry/TrackingGeometry.hpp>

#include "IActsTrackFindingAlg.h"
#include "IActsTrackFittingAlg.h"

class ActsAlg : public GaudiAlgorithm {
public:
  explicit ActsAlg(const std::string&, ISvcLocator*);

  virtual ~ActsAlg();
  /// Initialize.
  /// @return status code
  virtual StatusCode initialize() final;
  /// Execute.
  /// @return status code
  virtual StatusCode execute() final;
  /// Finalize.
  /// @return status code
  virtual StatusCode finalize() final;

private:
  mutable DataHandle<edm4hep::TrackerHitPlaneCollection> input_hits_;

  const Acts::TrackingGeometry*    tracking_geom_{nullptr};
  ToolHandle<IActsTrackFindingAlg> finding_algo_;
  ToolHandle<IActsTrackFittingAlg> fitting_algo_;
};

#endif
