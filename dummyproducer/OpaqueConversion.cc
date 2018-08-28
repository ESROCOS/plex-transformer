
#include "OpaqueConversion.hpp"
#include "Wrappers-Vector3dConvert.hpp"
#include "Wrappers-Matrix3dConvert.hpp"
#include "Wrappers-QuaterniondConvert.hpp"

//Conversion functions from the instanced type to the marshaled type


void Base_Vector3d_fromIntermediate(base::Vector3d& result, const wrappers::Vector3d& cppMarshall)
{
    /*Add user code for this conversion*/
    for(int i= 0; i < 3; i++)
    {
        result[i] = cppMarshall.data[i];
    }
}

void Base_Vector3d_toIntermediate(wrappers::Vector3d& result, const base::Vector3d& cppval)
{
    /*Add user code for this conversion*/
    for(int i= 0; i < 3; i++)
    {
         result.data[i] = cppval[i];
    }
}


void Base_Quaterniond_fromIntermediate(base::Quaterniond& result, const wrappers::Quaterniond& cppMarshall)
{
    /*Add user code for this conversion*/
    result.w() = cppMarshall.re;
    result.x()=cppMarshall.im[0];
    result.y()=cppMarshall.im[1];
    result.z()=cppMarshall.im[2];
}

void Base_Quaterniond_toIntermediate(wrappers::Quaterniond& result, const base::Quaterniond& cppval)
{
    /*Add user code for this conversion*/
    result.re = cppval.w();
    result.im[0] = cppval.x();
    result.im[1] = cppval.y();
    result.im[2] = cppval.z();
}


void Base_Matrix3d_fromIntermediate(base::Matrix3d& result, const wrappers::Matrix3d& cppMarshall)
{
    /*Add user code for this conversion*/
    for(int i = 0; i < 3; i++)
    {
        for(int j=0; j < 3; j++)
        {
            result(i,j)=cppMarshall.data[j*4+i];
        }
    }
}

void Base_Matrix3d_toIntermediate(wrappers::Matrix3d& result, const base::Matrix3d& cppval)
{
    /*Add user code for this conversion*/
    for(int i = 0; i < 3; i++)
    {
        for(int j=0; j < 3; j++)
        {
            result.data[j*3+i] = cppval(i,j);
        }
    }
}

void asn1Scc_Vector3d_fromAsn1(base::Vector3d& result, const asn1SccWrappers_Vector3d& asnVal)
{
    wrappers::Vector3d intermediate;
    asn1SccWrappers_Vector3d_fromAsn1(intermediate, asnVal);
    Base_Vector3d_fromIntermediate(result, intermediate);
}

void asn1Scc_Vector3d_toAsn1(asn1SccWrappers_Vector3d& result, const base::Vector3d& baseObj)
{
    wrappers::Vector3d intermediate;
    Base_Vector3d_toIntermediate(intermediate, baseObj);
    asn1SccWrappers_Vector3d_toAsn1(result, intermediate);
}

void asn1Scc_Matrix3d_fromAsn1(base::Matrix3d& result, const asn1SccWrappers_Matrix3d& asnVal)
{
    wrappers::Matrix3d intermediate;
    asn1SccWrappers_Matrix3d_fromAsn1(intermediate, asnVal);
    Base_Matrix3d_fromIntermediate(result, intermediate);
}

void asn1Scc_Matrix3d_toAsn1(asn1SccWrappers_Matrix3d& result, const base::Matrix3d& baseObj)
{
    wrappers::Matrix3d intermediate;
    Base_Matrix3d_toIntermediate(intermediate, baseObj);
    asn1SccWrappers_Matrix3d_toAsn1(result, intermediate);
}

void asn1Scc_Quaterniond_fromAsn1(base::Quaterniond& result, const asn1SccWrappers_Quaterniond& asnVal)
{
    wrappers::Quaterniond intermediate;
    asn1SccWrappers_Quaterniond_fromAsn1(intermediate, asnVal);
    Base_Quaterniond_fromIntermediate(result, intermediate);
}

void asn1Scc_Quaterniond_toAsn1(asn1SccWrappers_Quaterniond& result, const base::Quaterniond& baseObj)
{
    wrappers::Quaterniond intermediate;
    Base_Quaterniond_toIntermediate(intermediate, baseObj);
    asn1SccWrappers_Quaterniond_toAsn1(result, intermediate);
}

