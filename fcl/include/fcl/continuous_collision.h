#ifndef FCL_CONTINUOUS_COLLISION_H
#define FCL_CONTINUOUS_COLLISION_H

#include "fcl/math/vec_3f.h"
#include "fcl/collision_object.h"
#include "fcl/collision_data.h"
#include "fcl/ccd/motion_AM.h"  // Modbyme
#include "fcl/ccd/conservative_advancement.h"
#include "fcl/traversal/traversal_recurse.h"

namespace fcl
{

/// @brief continuous collision checking between two objects
FCL_REAL continuousCollide(const CollisionGeometry* o1, const Transform3f& tf1_beg, const Transform3f& tf1_end,
                           const CollisionGeometry* o2, const Transform3f& tf2_beg, const Transform3f& tf2_end,
                           const ContinuousCollisionRequest& request,
                           ContinuousCollisionResult& result);

FCL_REAL continuousCollide(const CollisionObject* o1, const Transform3f& tf1_end,
                           const CollisionObject* o2, const Transform3f& tf2_end,
                           const ContinuousCollisionRequest& request,
                           ContinuousCollisionResult& result);

FCL_REAL continuousCollide(const CollisionGeometry* o1, const std::vector<double> state1_beg, const std::vector<double> state1_end,
                           const CollisionGeometry* o2, const std::vector<double> state2_beg, const std::vector<double> state2_end,
                           const ContinuousCollisionRequest& request,
                           ContinuousCollisionResult& result);  // Modbyme

FCL_REAL continuousCollideNaive(const CollisionGeometry* o1, const MotionAM* motion1,
                                const CollisionGeometry* o2, const MotionAM* motion2,
                                const ContinuousCollisionRequest& request,
                                ContinuousCollisionResult& result);   // Modbyme

FCL_REAL continuousCollideConservativeAdvancement(const CollisionGeometry* o1, const MotionAM* motion1,
                                                  const CollisionGeometry* o2, const MotionAM* motion2,
                                                  const ContinuousCollisionRequest& request,
                                                  ContinuousCollisionResult& result);   // Modbyme

template<typename NarrowPhaseSolver>
FCL_REAL continuousCollideConservativeAdvancement(const CollisionGeometry* o1, const MotionAM* motion1,
                                                  const CollisionGeometry* o2, const MotionAM* motion2,
                                                  const NarrowPhaseSolver* nsolver_,
                                                  const ContinuousCollisionRequest& request,
                                                  ContinuousCollisionResult& result);   // Modbyme

template<typename BV, typename NarrowPhaseSolver>
FCL_REAL BVHConservativeAdvancement(const CollisionGeometry* o1, const MotionAM* motion1,
                                    const CollisionGeometry* o2, const MotionAM* motion2,
                                    const NarrowPhaseSolver* nsolver,
                                    const ContinuousCollisionRequest& request, ContinuousCollisionResult& result);  // Modbyme

bool conservativeAdvancement(const BVHModel<OBBRSS>& o1, const MotionAM* motion1,
                             const BVHModel<OBBRSS>& o2, const MotionAM* motion2,
                             const CollisionRequest& request, CollisionResult& result, FCL_REAL& toc);  // Modbyme

template<typename BV, typename ConservativeAdvancementOrientedNode>
bool conservativeAdvancementMeshOriented(const BVHModel<BV>& o1, const MotionAM* motion1,
                                         const BVHModel<BV>& o2, const MotionAM* motion2,
                                         const CollisionRequest& request, CollisionResult& result, FCL_REAL& toc);  // Modbyme

FCL_REAL collide(const ContinuousCollisionObject* o1, const ContinuousCollisionObject* o2,
             const ContinuousCollisionRequest& request,
             ContinuousCollisionResult& result);
          
}

#endif
