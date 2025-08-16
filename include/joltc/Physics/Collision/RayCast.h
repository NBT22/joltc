//
// Created by NBT22 on 8/15/25.
//

#ifndef JOLTC_RAYCAST_H
#define JOLTC_RAYCAST_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <joltc/enums.h>

typedef struct JPH_RayCastSettings
{
        /// How backfacing triangles should be treated (should we report back facing hits for triangle based shapes, e.g. MeshShape/HeightFieldShape?)
        JPH_BackFaceMode backFaceModeTriangles /* = JPH_BackFaceMode_IgnoreBackFaces*/;

        /// How backfacing convex objects should be treated (should we report back facing hits for convex shapes?)
        JPH_BackFaceMode backFaceModeConvex /* = JPH_BackFaceMode_IgnoreBackFaces*/;

        /// If convex shapes should be treated as solid. When true, a ray starting inside a convex shape will generate a hit at fraction 0.
        bool treatConvexAsSolid /* = true*/;
} JPH_RayCastSettings;

#ifdef __cplusplus
}
#endif

#endif //JOLTC_RAYCAST_H
