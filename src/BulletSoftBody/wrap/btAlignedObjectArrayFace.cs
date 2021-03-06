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

public class btAlignedObjectArrayFace : IDisposable {
  private HandleRef swigCPtr;
  protected bool swigCMemOwn;

  internal btAlignedObjectArrayFace(IntPtr cPtr, bool cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = new HandleRef(this, cPtr);
  }

  internal static HandleRef getCPtr(btAlignedObjectArrayFace obj) {
    return (obj == null) ? new HandleRef(null, IntPtr.Zero) : obj.swigCPtr;
  }

  ~btAlignedObjectArrayFace() {
    Dispose();
  }

  public virtual void Dispose() {
    lock(this) {
      if (swigCPtr.Handle != IntPtr.Zero) {
        if (swigCMemOwn) {
          swigCMemOwn = false;
          BulletSoftBodyPINVOKE.delete_btAlignedObjectArrayFace(swigCPtr);
        }
        swigCPtr = new HandleRef(null, IntPtr.Zero);
      }
      GC.SuppressFinalize(this);
    }
  }

  public btAlignedObjectArrayFace() : this(BulletSoftBodyPINVOKE.new_btAlignedObjectArrayFace__SWIG_0(), true) {
  }

  public btAlignedObjectArrayFace(btAlignedObjectArrayFace otherArray) : this(BulletSoftBodyPINVOKE.new_btAlignedObjectArrayFace__SWIG_1(btAlignedObjectArrayFace.getCPtr(otherArray)), true) {
    if (BulletSoftBodyPINVOKE.SWIGPendingException.Pending) throw BulletSoftBodyPINVOKE.SWIGPendingException.Retrieve();
  }

  public int size() {
    int ret = BulletSoftBodyPINVOKE.btAlignedObjectArrayFace_size(swigCPtr);
    return ret;
  }

  public Face at(int n) {
    Face ret = new Face(BulletSoftBodyPINVOKE.btAlignedObjectArrayFace_at__SWIG_0(swigCPtr, n), false);
    return ret;
  }

  public void clear() {
    BulletSoftBodyPINVOKE.btAlignedObjectArrayFace_clear(swigCPtr);
  }

  public void pop_back() {
    BulletSoftBodyPINVOKE.btAlignedObjectArrayFace_pop_back(swigCPtr);
  }

  public void resizeNoInitialize(int newsize) {
    BulletSoftBodyPINVOKE.btAlignedObjectArrayFace_resizeNoInitialize(swigCPtr, newsize);
  }

  public void resize(int newsize, Face fillData) {
    BulletSoftBodyPINVOKE.btAlignedObjectArrayFace_resize__SWIG_0(swigCPtr, newsize, Face.getCPtr(fillData));
    if (BulletSoftBodyPINVOKE.SWIGPendingException.Pending) throw BulletSoftBodyPINVOKE.SWIGPendingException.Retrieve();
  }

  public void resize(int newsize) {
    BulletSoftBodyPINVOKE.btAlignedObjectArrayFace_resize__SWIG_1(swigCPtr, newsize);
  }

  public Face expandNonInitializing() {
    Face ret = new Face(BulletSoftBodyPINVOKE.btAlignedObjectArrayFace_expandNonInitializing(swigCPtr), false);
    return ret;
  }

  public Face expand(Face fillValue) {
    Face ret = new Face(BulletSoftBodyPINVOKE.btAlignedObjectArrayFace_expand__SWIG_0(swigCPtr, Face.getCPtr(fillValue)), false);
    if (BulletSoftBodyPINVOKE.SWIGPendingException.Pending) throw BulletSoftBodyPINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public Face expand() {
    Face ret = new Face(BulletSoftBodyPINVOKE.btAlignedObjectArrayFace_expand__SWIG_1(swigCPtr), false);
    return ret;
  }

  public void push_back(Face _Val) {
    BulletSoftBodyPINVOKE.btAlignedObjectArrayFace_push_back(swigCPtr, Face.getCPtr(_Val));
    if (BulletSoftBodyPINVOKE.SWIGPendingException.Pending) throw BulletSoftBodyPINVOKE.SWIGPendingException.Retrieve();
  }

  public int capacity() {
    int ret = BulletSoftBodyPINVOKE.btAlignedObjectArrayFace_capacity(swigCPtr);
    return ret;
  }

  public void reserve(int _Count) {
    BulletSoftBodyPINVOKE.btAlignedObjectArrayFace_reserve(swigCPtr, _Count);
  }

  public void swap(int index0, int index1) {
    BulletSoftBodyPINVOKE.btAlignedObjectArrayFace_swap(swigCPtr, index0, index1);
  }

  public int findBinarySearch(Face key) {
    int ret = BulletSoftBodyPINVOKE.btAlignedObjectArrayFace_findBinarySearch(swigCPtr, Face.getCPtr(key));
    if (BulletSoftBodyPINVOKE.SWIGPendingException.Pending) throw BulletSoftBodyPINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public int findLinearSearch(Face key) {
    int ret = BulletSoftBodyPINVOKE.btAlignedObjectArrayFace_findLinearSearch(swigCPtr, Face.getCPtr(key));
    if (BulletSoftBodyPINVOKE.SWIGPendingException.Pending) throw BulletSoftBodyPINVOKE.SWIGPendingException.Retrieve();
    return ret;
  }

  public void remove(Face key) {
    BulletSoftBodyPINVOKE.btAlignedObjectArrayFace_remove(swigCPtr, Face.getCPtr(key));
    if (BulletSoftBodyPINVOKE.SWIGPendingException.Pending) throw BulletSoftBodyPINVOKE.SWIGPendingException.Retrieve();
  }

  public void initializeFromBuffer(SWIGTYPE_p_void buffer, int size, int capacity) {
    BulletSoftBodyPINVOKE.btAlignedObjectArrayFace_initializeFromBuffer(swigCPtr, SWIGTYPE_p_void.getCPtr(buffer), size, capacity);
  }

  public void copyFromArray(btAlignedObjectArrayFace otherArray) {
    BulletSoftBodyPINVOKE.btAlignedObjectArrayFace_copyFromArray(swigCPtr, btAlignedObjectArrayFace.getCPtr(otherArray));
    if (BulletSoftBodyPINVOKE.SWIGPendingException.Pending) throw BulletSoftBodyPINVOKE.SWIGPendingException.Retrieve();
  }

}

}
