#ifndef k4ActsTracking_SourceLink_h
#define k4ActsTracking_SourceLink_h

#include <edm4hep/TrackerHitPlane.h>

#include <Acts/Surfaces/Surface.hpp>

#include "GeometryContainers.h"

namespace k4ActsTracking {
  //! \brief Link between an ACTS surface and hit index
  class SourceLink final {
  public:
    /// Construct from geometry identifier and hit
    explicit SourceLink(Acts::GeometryIdentifier gid, std::size_t index, const edm4hep::TrackerHitPlane* hit)
        : geometry_id_(gid), index_(index), edm4hep_hit_(hit) {}

    // Construct an invalid source link. Must be default constructible to
    /// satisfy SourceLinkConcept.
    SourceLink()                             = default;
    SourceLink(const SourceLink&)            = default;
    SourceLink(SourceLink&&)                 = default;
    SourceLink& operator=(const SourceLink&) = default;
    SourceLink& operator=(SourceLink&&)      = default;

    /// Access the geometry identifier.
    constexpr Acts::GeometryIdentifier geometryId() const { return geometry_id_; }
    /// Access the index.
    constexpr std::size_t index() const { return index_; }
    /// Access the edm4hep TrackerHitPlane
    /// @TODO: We want this to be a TrackerHit to support multiple types of tracking detector. However, TrackerHitPlane is currently not derived from TrackerHit as expected.
    constexpr const edm4hep::TrackerHitPlane* edm4hepTHitP() const { return edm4hep_hit_; }

  private:
    Acts::GeometryIdentifier        geometry_id_;
    std::size_t                     index_       = -1;
    const edm4hep::TrackerHitPlane* edm4hep_hit_ = nullptr;

    friend constexpr bool operator==(const SourceLink& lhs, const SourceLink& rhs) {
      return lhs.geometry_id_ == rhs.geometry_id_ && lhs.index_ == rhs.index_ && lhs.edm4hep_hit_ == rhs.edm4hep_hit_;
    }
    friend constexpr bool operator!=(const SourceLink& lhs, const SourceLink& rhs) { return not(lhs == rhs); }
  };

  /// Container of index source links
  using SourceLinkContainer = GeometryIdMultiset<SourceLink>;
  /// Accessor for the above source link container
  ///
  /// It wraps up a few lookup methods to be used in the Combinatorial Kalman
  /// Filter
  struct SourceLinkAccessor : GeometryIdMultisetAccessor<SourceLink> {
    using BaseIterator = GeometryIdMultisetAccessor<SourceLink>::Iterator;
    using Iterator     = Acts::SourceLinkAdapterIterator<BaseIterator>;

    // get the range of elements with requested geoId
    std::pair<Iterator, Iterator> range(const Acts::Surface& surface) const {
      assert(container != nullptr);
      auto [begin, end] = container->equal_range(surface.geometryId());
      return {Iterator{begin}, Iterator{end}};
    }
  };
}  // namespace k4ActsTracking

#endif
