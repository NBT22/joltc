//
// Created by NBT22 on 8/15/25.
//

#include <joltc/Physics/SoftBody/SoftBodyCreationSettings.h>
#include <Jolt/Jolt.h>
#include <Jolt/Physics/SoftBody/SoftBodyCreationSettings.h>
#include <Physics/SoftBody/SoftBodyCreationSettings.hpp>

JPH_SoftBodyCreationSettings *JPH_SoftBodyCreationSettings_Create()
{
    JPH::SoftBodyCreationSettings *const bodyCreationSettings = new JPH::SoftBodyCreationSettings();
    return ToSoftBodyCreationSettings(bodyCreationSettings);
}

void JPH_SoftBodyCreationSettings_Destroy(JPH_SoftBodyCreationSettings *settings)
{
    if (settings != nullptr)
    {
        delete AsSoftBodyCreationSettings(settings);
    }
}
