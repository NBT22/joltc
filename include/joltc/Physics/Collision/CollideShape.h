//
// Created by NBT22 on 1/25/26.
//

#ifndef JOLTC_COLLIDESHAPE_H
#define JOLTC_COLLIDESHAPE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <joltc/enums.h>
#include <joltc/Math/Vector3.h>

typedef struct JPH_CollideShapeResult JPH_CollideShapeResult;

typedef struct JPH_CollideSettingsBase
{
        /// How active edges (edges that a moving object should bump into) are handled
        JPH_ActiveEdgeMode activeEdgeMode /* = JPH_ActiveEdgeMode_CollideOnlyWithActive*/;

        /// If colliding faces should be collected or only the collision point
        JPH_CollectFacesMode collectFacesMode /* = JPH_CollectFacesMode_NoFaces*/;

        /// If objects are closer than this distance, they are considered to be colliding (used for GJK) (unit: meter)
        float collisionTolerance /* = JPH_DEFAULT_COLLISION_TOLERANCE*/;

        /// A factor that determines the accuracy of the penetration depth calculation. If the change of the squared distance is less than tolerance * current_penetration_depth^2 the algorithm will terminate. (unit: dimensionless)
        float penetrationTolerance /* = JPH_DEFAULT_PENETRATION_TOLERANCE*/;

        /// When mActiveEdgeMode is CollideOnlyWithActive a movement direction can be provided. When hitting an inactive edge, the system will select the triangle normal as penetration depth only if it impedes the movement less than with the calculated penetration depth.
        Vector3 activeEdgeMovementDirection /* = Vec3::sZero()*/;
} JPH_CollideSettingsBase;

/* CollideShapeSettings */
typedef struct JPH_CollideShapeSettings
{
        JPH_CollideSettingsBase base; /* Inherits JPH_CollideSettingsBase */
        /// When > 0 contacts in the vicinity of the query shape can be found. All nearest contacts that are not further away than this distance will be found (unit: meter)
        float maxSeparationDistance /* = 0.0f*/;

        /// How backfacing triangles should be treated
        JPH_BackFaceMode backFaceMode /* = JPH_BackFaceMode_IgnoreBackFaces*/;
} JPH_CollideShapeSettings;

typedef void JPH_CollideShapeResultCallback(void *context, const JPH_CollideShapeResult *result);


JPH_CAPI void JPH_CollideSettingsBase_Init(JPH_CollideSettingsBase *settings);

JPH_CAPI void JPH_CollideShapeSettings_Init(JPH_CollideShapeSettings *settings);


#ifdef __cplusplus
}
#endif

#endif //JOLTC_COLLIDESHAPE_H
