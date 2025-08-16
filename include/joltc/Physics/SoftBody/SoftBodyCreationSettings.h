//
// Created by NBT22 on 8/15/25.
//

#ifndef JOLTC_SOFTBODYCREATIONSETTINGS_H
#define JOLTC_SOFTBODYCREATIONSETTINGS_H

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct JPH_SoftBodyCreationSettings JPH_SoftBodyCreationSettings;

JPH_CAPI JPH_SoftBodyCreationSettings *JPH_SoftBodyCreationSettings_Create(void);
JPH_CAPI void JPH_SoftBodyCreationSettings_Destroy(JPH_SoftBodyCreationSettings *settings);

#ifdef __cplusplus
}
#endif

#endif //JOLTC_SOFTBODYCREATIONSETTINGS_H
