#ifndef k4ActsTracking_SeedSpacePoint_h
#define k4ActsTracking_SeedSpacePoint_h

#include <Acts/Definitions/Algebra.hpp>
#include <Acts/Definitions/Common.hpp>

#include <cmath>
#include <vector>

namespace k4ActsTracking {
  /// Space point representation of a measurement suitable for track seeding.
  /// \note Based on ACTS' `ActsExamples::SimSpacePoint`.
  class SeedSpacePoint {
  public:
    /** Construct the space point from global position and selected variances.
     *
     * @tparam T Input position type
     * @param pos Global position
     * @param var_rho Measurement variance of the global transverse distance
     * @param var_z Measurement variance of the global longitudinal position
     * @param source_link Link to the original measurement
     */
    template <typename T>
    explicit SeedSpacePoint(const Eigen::MatrixBase<T>& pos, float var_rho, float var_z, const SourceLink& source_link)
        : x_(pos[Acts::ePos0]),
          y_(pos[Acts::ePos1]),
          z_(pos[Acts::ePos2]),
          rho_(std::hypot(x_, y_)),
          rho_variance_(var_rho),
          z_variance_(var_z),
          source_link_(source_link) {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(T, 3);
    }

    constexpr float      x() const { return x_; }
    constexpr float      y() const { return y_; }
    constexpr float      z() const { return z_; }
    constexpr float      r() const { return rho_; }
    constexpr float      varianceR() const { return rho_variance_; }
    constexpr float      varianceZ() const { return z_variance_; }
    constexpr SourceLink sourceLink() const { return source_link_; }

    const std::optional<float> t() const { return source_link_.edm4hepTHitP()->getTime(); }
    /// @TODO missing: const std::optional<float> varianceT() const

  private:
    float      x_;  ///< Global x position
    float      y_;  ///< Global y position
    float      z_;  ///< Global z position
    float      rho_;
    float      rho_variance_;  ///< Variance in rho of the global coordinates
    float      z_variance_;    ///< Variance in z of the global coordinates
    SourceLink source_link_;   ///< Index of the corresponding measurement
  };

  /**
   * @TODO would it be sufficient to check just the index under the assumption
   * that the same measurement index always produces the same space point?
   * no need to check r since it is fully defined by x/y
   */
  constexpr bool operator==(const SeedSpacePoint& lhs, const SeedSpacePoint& rhs) {
    return lhs.sourceLink() == rhs.sourceLink() && lhs.x() == rhs.x() && lhs.y() == rhs.y() && lhs.z() == rhs.z() &&
           lhs.varianceR() == rhs.varianceR() && lhs.varianceZ() == rhs.varianceZ();
  }

  /// Container of space points.
  using SeedSpacePointContainer = std::vector<SeedSpacePoint>;
}  // namespace k4ActsTracking

#endif
