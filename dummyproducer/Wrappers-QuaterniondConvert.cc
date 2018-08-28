
#include "Wrappers-QuaterniondConvert.hpp"


//Conversion functions from asn1 to c++ type



void asn1SccWrappers_Quaterniond_fromAsn1(wrappers::Quaternion<double>& result, const asn1SccWrappers_Quaterniond& asnVal)
{
    for(int i = 0; i < 3;i++)
    {
        result.im[i] = asnVal.im.arr[i];
    }

    result.re = asnVal.re;

}

//Conversion functions from c++ type to asn1

void asn1SccWrappers_Quaterniond_toAsn1(asn1SccWrappers_Quaterniond& result, const wrappers::Quaternion<double>& baseObj)
{
    for(int i = 0; i < 3;i++)
    {
        result.im.arr[i] = baseObj.im[i];
    }

    result.re = baseObj.re;

}


