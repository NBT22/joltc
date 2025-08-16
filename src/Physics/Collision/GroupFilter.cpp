//
// Created by NBT22 on 8/15/25.
//

#include <joltc/Physics/Collision/GroupFilter.h>
#include <joltc/Physics/Collision/CollisionGroup.h>
#include <Physics/Collision/GroupFilter.hpp>
#include <Physics/Collision/CollisionGroup.hpp>

void JPH_GroupFilter_Destroy(JPH_GroupFilter *groupFilter)
{
    if (groupFilter != nullptr)
    {
        AsGroupFilter(groupFilter)->Release();
    }
}

bool JPH_GroupFilter_CanCollide(JPH_GroupFilter *groupFilter,
                                const JPH_CollisionGroup *group1,
                                const JPH_CollisionGroup *group2)
{
    return AsGroupFilter(groupFilter)->CanCollide(ToJolt(group1), ToJolt(group2));
}
