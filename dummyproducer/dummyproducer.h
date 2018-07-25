/* This file was generated automatically: DO NOT MODIFY IT ! */

/* Declaration of the functions that have to be provided by the user */

#ifndef __USER_CODE_H_dummyproducer__
#define __USER_CODE_H_dummyproducer__

#include "C_ASN1_Types.h"

#ifdef __cplusplus
extern "C" {
#endif

void dummyproducer_startup();

void dummyproducer_PI_clock();

extern void dummyproducer_RI_robotPose(const asn1SccBase_samples_RigidBodyState *);

extern void dummyproducer_RI_relativeMarkerPose(const asn1SccBase_samples_RigidBodyState *);

#ifdef __cplusplus
}
#endif


#endif
