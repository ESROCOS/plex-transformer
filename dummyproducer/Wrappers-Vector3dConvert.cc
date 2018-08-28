
#include "Wrappers-Vector3dConvert.hpp"

//Conversion functions from asn1 to c++ type



void asn1SccWrappers_Vector3d_fromAsn1(wrappers::Matrix<double,3,1>& result, const asn1SccWrappers_Vector3d& asnVal)
{
    for(int i = 0; i < 3;i++)
    {
        result.data[i] = asnVal.data.arr[i];
    }

}

//Conversion functions from c++ type to asn1

void asn1SccWrappers_Vector3d_toAsn1(asn1SccWrappers_Vector3d& result, const wrappers::Matrix<double,3,1>& baseObj)
{
    for(int i = 0; i < 3;i++)
    {
        result.data.arr[i] = baseObj.data[i];
    }

}


