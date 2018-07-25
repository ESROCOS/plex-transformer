/* This file was generated automatically: DO NOT MODIFY IT ! */

/* Declaration of the functions that have to be provided by the user */

#ifndef __USER_CODE_H_transformer__
#define __USER_CODE_H_transformer__

#include "C_ASN1_Types.h"

#ifdef __cplusplus
extern "C" {
#endif

void transformer_startup();

void transformer_PI_robotPose(const asn1SccBase_samples_RigidBodyState *);

void transformer_PI_relativeMarkerPose(const asn1SccBase_samples_RigidBodyState *);

extern void transformer_RI_absoluteMarkerPose(const asn1SccBase_samples_RigidBodyState *);

#ifdef __cplusplus
}
#endif


#endif
