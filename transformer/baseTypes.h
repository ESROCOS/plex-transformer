/*
 * A header file to import all the asn1 types either from the
 * types library or from dataview-uniq.h (if building from TASTE).
 */

#ifndef BASETYPES_HPP
#define BASETYPES_HPP

#include "C_ASN1_Types.h"
// Include types generated from ASN.1. These are either:
// - in dataview-uniq.h, if building from TASTE
//   (IMPORTANT: note that the order of includes matters)
// - in the asn1, if building from outside TASTE
// We distingush between the two cases by checking the include variable
// (the variable name has changed in different TASTE versions)
#ifndef _INC_DATAVIEW_UNIQ_H
#ifndef GENERATED_ASN1_DATAVIEW_UNIQ_H
#ifndef GENERATED_ASN1SCC_DATAVIEW_UNIQ_H
#ifndef GENERATED_ASN1SCC_dataview_uniq_H

//#include "asn1/asn1crt.h"
//#include "asn1/taste-extended.h"
//#include "asn1/taste-types.h"
//#include "asn1/base.h"
//#include "asn1/userdefs-base.h"

#endif //GENERATED_ASN1SCC_dataview_uniq_H
#endif //GENERATED_ASN1SCC_DATAVIEW_UNIQ_H
#endif //GENERATED_ASN1_DATAVIEW_UNIQ_H
#endif //_INC_DATAVIEW_UNIQ_H

#endif //BASETYPES_HPP
