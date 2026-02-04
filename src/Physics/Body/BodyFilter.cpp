//
// Created by NBT22 on 2/3/26.
//

#include <joltc/Physics/Body/Body.h>
#include <joltc/Physics/Body/BodyFilter.h>
#include <Jolt/Jolt.h>
#include <Jolt/Core/IssueReporting.h>
#include <Jolt/Physics/Body/BodyFilter.h>
#include <Jolt/Physics/Body/BodyID.h>

class ManagedBodyFilter final: public JPH::BodyFilter
{
    public:
        explicit ManagedBodyFilter(const JPH_BodyFilter_Impl *impl): impl(impl) {}

        [[nodiscard]] bool ShouldCollide(const JPH::BodyID &bodyID) const override
        {
            if (impl != nullptr && impl->ShouldCollide != nullptr)
            {
                return impl->ShouldCollide(bodyID.GetIndexAndSequenceNumber());
            }

            return true;
        }

        [[nodiscard]] bool ShouldCollideLocked(const JPH::Body &body) const override
        {
            if (impl != nullptr && impl->ShouldCollideLocked != nullptr)
            {
                return impl->ShouldCollideLocked(reinterpret_cast<const JPH_Body *>(&body));
            }

            return true;
        }

        const JPH_BodyFilter_Impl *impl{};
};

JPH_BodyFilter *JPH_BodyFilter_Create(const JPH_BodyFilter_Impl *impl)
{
    ManagedBodyFilter *const filter = new ManagedBodyFilter(impl);
    return reinterpret_cast<JPH_BodyFilter *>(filter);
}

void JPH_BodyFilter_Destroy(JPH_BodyFilter *filter)
{
    delete reinterpret_cast<ManagedBodyFilter *>(filter);
}

void JPH_BodyFilter_SetImpl(JPH_BodyFilter *filter, const JPH_BodyFilter_Impl *impl)
{
    JPH_ASSERT(filter);
    reinterpret_cast<ManagedBodyFilter *>(filter)->impl = impl;
}
