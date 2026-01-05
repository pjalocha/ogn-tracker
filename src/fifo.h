#ifndef __FIFO_H__
#define __FIFO_H__

#include <stdint.h>
#include <stdlib.h>

template <class Type, const unsigned Size> // size must be (!) a power of 2 like 4, 8, 16, 32, etc.
 class FIFO
{ public:
   static const unsigned Len = Size;
   static const unsigned PtrMask = Size-1;

   volatile unsigned ReadPtr;
   volatile unsigned WritePtr;
   Type Data[Len];

  public:
   FIFO() { Clear(); }

   bool isCorrupt(void) const
   { return ReadPtr>=Size || WritePtr>=Size; }

   void Clear(void)                          // clear all stored data
   { ReadPtr=0; WritePtr=0; }

   unsigned Write(Type Byte)                // write a single element
   { unsigned Ptr=WritePtr;
     Data[Ptr]=Byte;
     Ptr++; Ptr&=PtrMask;
     if(Ptr==ReadPtr) return 0;
     WritePtr=Ptr; return 1; }

   bool isFull(void) const               // if FIFO full ?
   { unsigned Ptr=WritePtr;
     Ptr++; Ptr&=PtrMask;
     return Ptr==ReadPtr; }

    unsigned Free(void) const               // number of free elements: how much can you write into te FIFO
    { return ((ReadPtr-WritePtr-1)&PtrMask); }

    unsigned Full(void) const               // number of stored elements: how much you can read from the FIFO
    { return ((WritePtr-ReadPtr)&PtrMask); }

   Type *getWrite(void)                  // get pointer to the next element which can be written
   { return Data+WritePtr; }             // even when the FIFO is full this element exists

   unsigned Write(void)                    // advance the write pointer: to be used with getWrite()
   { unsigned Ptr=WritePtr;
     Ptr++; Ptr&=PtrMask;
     if(Ptr==ReadPtr) return 0;            // but, when the write pointer hits the read pointer then give up, do not advance
     WritePtr=Ptr; return 1; }

   unsigned Read(Type &Byte)               // read a single element
   { unsigned Ptr=ReadPtr;
     if(Ptr==WritePtr) return 0;           // FIFO empty
     Byte=Data[Ptr];
     Ptr++; Ptr&=PtrMask;                  // increment pointer
     ReadPtr=Ptr; return 1; }              // store it

   void Read(void)                         // increment the read-pointer (thus forget the oldest item)
   { unsigned Ptr=ReadPtr;
     if(Ptr==WritePtr) return;
     Ptr++; Ptr&=PtrMask;
     ReadPtr=Ptr; }

   Type *getRead(void)                     // get pointer to the most recent item.
   { if(ReadPtr==WritePtr) return 0;
     return Data+ReadPtr; }

   Type *getRead(unsigned Idx)
   { if(Idx>=Full()) return 0;
     unsigned Ptr=(ReadPtr+Idx)&PtrMask;
     return Data+Ptr; }

   unsigned getReadBlock(Type *&Byte)      // get a pointer to the first element and the number of consecutive elements available for read
   { if(ReadPtr==WritePtr) { Byte=0; return 0; }
     Byte = Data+ReadPtr;
     if(ReadPtr<WritePtr) return WritePtr-ReadPtr;
     return Size-ReadPtr; }

   void flushReadBlock(unsigned Len)       // flush the elements which were already read: to be used after getReadBlock()
   { ReadPtr+=Len; ReadPtr&=PtrMask; }

   bool isEmpty(void) const                // is the FIFO all empty ?
   { return ReadPtr==WritePtr; }

   unsigned Write(const Type *Data, unsigned Len) // write a block of elements into the FIFO (could possibly be smarter than colling single Write many times)
   { unsigned Idx;
     for(Idx=0; Idx<Len; Idx++)
     { if(Write(Data[Idx])==0) break; }
     return Idx; }

/*
   Type Read(void)
   { uint16_t Ptr=ReadPtr;
     if(Ptr==WritePtr) return 0x00;
     uint8_t Byte=Data[Ptr];
     Ptr++; Ptr&=PtrMask;
     ReadPtr=Ptr; return Byte; }

   uint16_t ReadReady(void) const
   { return ReadPtr!=WritePtr; }

   uint8_t WriteReady(void) const
   { uint8_t Ptr=WritePtr;
     Ptr++; Ptr&=PtrMask;
     return Ptr!=ReadPtr; }
*/
} ;

template <class Type, const uint8_t Size=8> // size must be (!) a power of 2 like 4, 8, 16, 32, etc.
 class Delay
{ public:
   static const uint8_t Len = Size;
   static const uint8_t PtrMask = Size-1;

   Type Data[Len];
   uint8_t Ptr;

  public:
   void Clear(Type Zero=0)
   { for(uint8_t Idx=0; Idx<Len; Idx++)
       Data[Idx]=Zero;
     Ptr=0; }

   Type Input(Type Inp)
   { Ptr--; Ptr&=PtrMask; 
     Type Out=Data[Ptr];
     Data[Ptr]=Inp;
     return Out; }

   Type & operator[](uint8_t Idx)
   { Idx = Ptr-Idx; Idx&=PtrMask;
     return Data[Idx]; }

} ;

#endif // __FIFO_H__
