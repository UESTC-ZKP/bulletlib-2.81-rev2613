/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 2.0.8
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

namespace BulletCSharp {

using System;
using System.Runtime.InteropServices;

public class btAlignedObjectArrayInt : IDisposable {
  private HandleRef swigCPtr;
  protected bool swigCMemOwn;

  internal btAlignedObjectArrayInt(IntPtr cPtr, bool cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = new HandleRef(this, cPtr);
  }

  internal static HandleRef getCPtr(btAlignedObjectArrayInt obj) {
    return (obj == null) ? new HandleRef(null, IntPtr.Zero) : obj.swigCPtr;
  }

  ~btAlignedObjectArrayInt() {
    Dispose();
  }

  public virtual void Dispose() {
    lock(this) {
      if (swigCPtr.Handle != IntPtr.Zero) {
        if (swigCMemOwn) {
          swigCMemOwn = false;
          BulletCollisionPINVOKE.delete_btAlignedObjectArrayInt(swigCPtr);
        }
        swigCPtr = new HandleRef(null, IntPtr.Zero);
      }
      GC.SuppressFinalize(this);
    }
  }

  public btAlignedObjectArrayInt() : this(BulletCollisionPINVOKE.new_btAlignedObjectArrayInt__SWIG_0(), true) {
  }

  public btAlignedObjectArrayInt(btAlignedObjectArrayInt otherArray) : this(BulletCollisionPINVOKE.new_btAlignedObjectArrayInt__SWIG_1(btAlignedObjectArrayInt.getCPtr(otherArray)), true) {
    if (BulletCollisionPINVOKE.SWIGPendingException.Pending) throw BulletCollisionPINVOKE.SWIGPendingException.Retrieve();
  }

  public int size() {
    int ret = BulletCollisionPINVOKE.btAlignedObjectArrayInt_size(swigCPtr);
    return ret;
  }

  public int at(int n) {
    int ret = BulletCollisionPINVOKE.btAlignedObjectArrayInt_at__SWIG_0(swigCPtr, n);
    return ret;
  }

  public void clear() {
    BulletCollisionPINVOKE.btAlignedObjectArrayInt_clear(swigCPtr);
  }

  public void pop_back() {
    BulletCollisionPINVOKE.btAlignedObjectArrayInt_pop_back(swigCPtr);
  }

  public void resizeNoInitialize(int newsize) {
    BulletCollisionPINVOKE.btAlignedObjectArrayInt_resizeNoInitialize(swigCPtr, newsize);
  }

  public void resize(int newsize, int fillData) {
    BulletCollisionPINVOKE.btAlignedObjectArrayInt_resize__SWIG_0(swigCPtr, newsize, fillData);
  }

  public void resize(int newsize) {
    BulletCollisionPINVOKE.btAlignedObjectArrayInt_resize__SWIG_1(swigCPtr, newsize);
  }

  public SWIGTYPE_p_int expandNonInitializing() {
    SWIGTYPE_p_int ret = new SWIGTYPE_p_int(BulletCollisionPINVOKE.btAlignedObjectArrayInt_expandNonInitializing(swigCPtr), false);
    return ret;
  }

  public SWIGTYPE_p_int expand(int fillValue) {
    SWIGTYPE_p_int ret = new SWIGTYPE_p_int(BulletCollisionPINVOKE.btAlignedObjectArrayInt_expand__SWIG_0(swigCPtr, fillValue), false);
    return ret;
  }

  public SWIGTYPE_p_int expand() {
    SWIGTYPE_p_int ret = new SWIGTYPE_p_int(BulletCollisionPINVOKE.btAlignedObjectArrayInt_expand__SWIG_1(swigCPtr), false);
    return ret;
  }

  public void push_back(int _Val) {
    BulletCollisionPINVOKE.btAlignedObjectArrayInt_push_back(swigCPtr, _Val);
  }

  public int capacity() {
    int ret = BulletCollisionPINVOKE.btAlignedObjectArrayInt_capacity(swigCPtr);
    return ret;
  }

  public void reserve(int _Count) {
    BulletCollisionPINVOKE.btAlignedObjectArrayInt_reserve(swigCPtr, _Count);
  }

  public void swap(int index0, int index1) {
    BulletCollisionPINVOKE.btAlignedObjectArrayInt_swap(swigCPtr, index0, index1);
  }

  public int findBinarySearch(int key) {
    int ret = BulletCollisionPINVOKE.btAlignedObjectArrayInt_findBinarySearch(swigCPtr, key);
    return ret;
  }

  public int findLinearSearch(int key) {
    int ret = BulletCollisionPINVOKE.btAlignedObjectArrayInt_findLinearSearch(swigCPtr, key);
    return ret;
  }

  public void remove(int key) {
    BulletCollisionPINVOKE.btAlignedObjectArrayInt_remove(swigCPtr, key);
  }

  public void initializeFromBuffer(SWIGTYPE_p_void buffer, int size, int capacity) {
    BulletCollisionPINVOKE.btAlignedObjectArrayInt_initializeFromBuffer(swigCPtr, SWIGTYPE_p_void.getCPtr(buffer), size, capacity);
  }

  public void copyFromArray(btAlignedObjectArrayInt otherArray) {
    BulletCollisionPINVOKE.btAlignedObjectArrayInt_copyFromArray(swigCPtr, btAlignedObjectArrayInt.getCPtr(otherArray));
    if (BulletCollisionPINVOKE.SWIGPendingException.Pending) throw BulletCollisionPINVOKE.SWIGPendingException.Retrieve();
  }

}

}
