#pragma once

#include <stdint.h>

#include "intmath.h"

// ===================================================================================================

template <class Type>
 class Vector3D
{ public:
   union
   { Type A[3];
     struct
     { Type X, Y, Z; } ;
   } ;

  public:

   Type & operator [](int Idx) { return A[Idx]; }

   void Set(Type V=0)
   { X=V; Y=V; Z=V; }

   void Min(const Vector3D<Type> &V)
   { if(V.X<X) X=V.X;
     if(V.Y<Y) Y=V.Y;
     if(V.Z<Z) Z=V.Z; }

   void Max(const Vector3D<Type> &V)
   { if(V.X>X) X=V.X;
     if(V.Y>Y) Y=V.Y;
     if(V.Z>Z) Z=V.Z; }

   template <class ArgType>
    void Sub(const Vector3D<ArgType> &V)
   { X-=V.X;
     Y-=V.Y;
     Z-=V.Z; }

   template <class ArgType>
    void Add(const Vector3D<ArgType> &V)
   { X+=V.X;
     Y+=V.Y;
     Z+=V.Z; }

} ;

// ===================================================================================================

template <class Type=int32_t>
 class SphereFitInt
{ public:
   Vector3D<Type> O;       // sphere center
            Type  R;       // sphere radius

   Vector3D<int32_t> Sum_dO;
            int32_t  Sum_dR;
            int64_t  Sum_dRR;
            int32_t  Sum_Cnt;

  public:

   void Clear(void)
   { Sum_dR=0; Sum_dRR=0; Sum_dO.Set(0); Sum_Cnt=0; }

   int getO(Vector3D<Type> &dO)
   { if(Sum_Cnt==0) return 0;
     dO.X = Sum_dO.X/Sum_Cnt/R;
     dO.Y = Sum_dO.Y/Sum_Cnt/R;
     dO.Z = Sum_dO.Z/Sum_Cnt/R;
     return Sum_Cnt; }

   int getR(Type &dR, Type &dRR)
   { if(Sum_Cnt==0) return 0;
     dR = Sum_dR/Sum_Cnt;
     dRR = IntSqrt((Sum_dRR-(int64_t)Sum_Cnt*dR*dR)/Sum_Cnt);
     return Sum_Cnt; }

   void Process(Vector3D<Type> I)
   { I.Sub(O);
     uint32_t S = VectPwr(I);
      int32_t D = IntSqrt(S);
      int32_t dR = D-R;
     Sum_dR  += dR;
     Sum_dRR += (int32_t)dR*dR;
     Sum_dO.X += (int32_t)I.X*dR;
     Sum_dO.Y += (int32_t)I.Y*dR;
     Sum_dO.Z += (int32_t)I.Z*dR;
     Sum_Cnt++; }

   static uint32_t VectPwr(const Vector3D<Type> &V)
   { return (int32_t)V.X*V.X + (int32_t)V.Y*V.Y + (int32_t)V.Z*V.Z; }

} ;

// ===================================================================================================
