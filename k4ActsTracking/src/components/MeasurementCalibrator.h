#ifndef k4ActsTracking_MeasurementCalibrator_h
#define k4ActsTracking_MeasurementCalibrator_h

#include <Acts/EventData/Measurement.hpp>
#include <Acts/EventData/SourceLink.hpp>
#include <Acts/EventData/VectorMultiTrajectory.hpp>
#include <Acts/Geometry/GeometryContext.hpp>
#include <Acts/Utilities/CalibrationContext.hpp>

#include "SourceLink.h"

namespace k4ActsTracking {
  using Measurement          = Acts::BoundVariantMeasurement;  ///< Hit stored as an measurement
  using MeasurementContainer = std::vector<Measurement>;       ///< Collection of measurements

  class MeasurementCalibrator {
  public:
    MeasurementCalibrator() = default;  ///< Invalid calibrator. Required to allow copying.
    /// Construct using a user-provided container to chose measurements from.
    explicit MeasurementCalibrator(const MeasurementContainer& measurements) : measurements_(measurements) {}

    //! Find the measurement corresponding to the source link.
    /**
     * @tparam T Track parameters type
     * @param source_link Input source link
     * @param parameters Input track parameters (unused)
     */
    template <typename T>
    const Measurement& operator()(const SourceLink& source_link, const T& /* parameters */) const {
      assert(source_link.index() < measurements_.size() and "Source link index is outside the container bounds");
      return measurements_.at(source_link.index());
    }

    void calibrate(const Acts::GeometryContext&, const Acts::CalibrationContext&, const Acts::SourceLink& source_link,
                   Acts::VectorMultiTrajectory::TrackStateProxy track_state) const {
      track_state.setUncalibratedSourceLink(source_link);
      const auto& idx_source_link = source_link.get<SourceLink>();

      assert(idx_source_link.index() < measurements_.size() and "Source link index is outside the container bounds");

      const auto& meas = std::get<1>(measurements_.at(idx_source_link.index()));  ///< @TODO workaround
      track_state.allocateCalibrated(meas.size());
      track_state.setCalibrated(meas);
    }

  private:
    const MeasurementContainer& measurements_;
  };

}  // namespace k4ActsTracking

#endif
