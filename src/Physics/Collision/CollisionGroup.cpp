//
// Created by NBT22 on 8/15/25.
//

#include <Physics/Body/BodyCreationSettings.hpp>
#include <Physics/Collision/CollisionGroup.hpp>

void JPH_BodyCreationSettings_GetCollisionGroup(const JPH_BodyCreationSettings *settings, JPH_CollisionGroup *result)
{
    FromJolt(AsBodyCreationSettings(settings)->mCollisionGroup, result);
}

void JPH_BodyCreationSettings_SetCollisionGroup(JPH_BodyCreationSettings *settings, const JPH_CollisionGroup *value)
{
    AsBodyCreationSettings(settings)->mCollisionGroup = ToJolt(value);
}
