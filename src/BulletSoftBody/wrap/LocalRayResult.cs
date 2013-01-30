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

public class LocalRayResult : IDisposable {
  private HandleRef swigCPtr;
  protected bool swigCMemOwn;

  internal LocalRayResult(IntPtr cPtr, bool cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = new HandleRef(this, cPtr);
  }

  internal static HandleRef getCPtr(LocalRayResult obj) {
    return (obj == null) ? new HandleRef(null, IntPtr.Zero) : obj.swigCPtr;
  }

  ~LocalRayResult() {
    Dispose();
  }

  public virtual void Dispose() {
    lock(this) {
      if (swigCPtr.Handle != IntPtr.Zero) {
        if (swigCMemOwn) {
          swigCMemOwn = false;
          BulletSoftBodyPINVOKE.delete_LocalRayResult(swigCPtr);
        }
        swigCPtr = new HandleRef(null, IntPtr.Zero);
      }
      GC.SuppressFinalize(this);
    }
  }

  public LocalRayResult(SWIGTYPE_p_btCollisionObject collisionObject, LocalShapeInfo localShapeInfo, SWIGTYPE_p_btVector3 hitNormalLocal, float hitFraction) : this(BulletSoftBodyPINVOKE.new_LocalRayResult(SWIGTYPE_p_btCollisionObject.getCPtr(collisionObject), LocalShapeInfo.getCPtr(localShapeInfo), SWIGTYPE_p_btVector3.getCPtr(hitNormalLocal), hitFraction), true) {
    if (BulletSoftBodyPINVOKE.SWIGPendingException.Pending) throw BulletSoftBodyPINVOKE.SWIGPendingException.Retrieve();
  }

  public SWIGTYPE_p_btCollisionObject m_collisionObject {
    set {
      BulletSoftBodyPINVOKE.LocalRayResult_m_collisionObject_set(swigCPtr, SWIGTYPE_p_btCollisionObject.getCPtr(value));
    } 
    get {
      IntPtr cPtr = BulletSoftBodyPINVOKE.LocalRayResult_m_collisionObject_get(swigCPtr);
      SWIGTYPE_p_btCollisionObject ret = (cPtr == IntPtr.Zero) ? null : new SWIGTYPE_p_btCollisionObject(cPtr, false);
      return ret;
    } 
  }

  public LocalShapeInfo m_localShapeInfo {
    set {
      BulletSoftBodyPINVOKE.LocalRayResult_m_localShapeInfo_set(swigCPtr, LocalShapeInfo.getCPtr(value));
    } 
    get {
      IntPtr cPtr = BulletSoftBodyPINVOKE.LocalRayResult_m_localShapeInfo_get(swigCPtr);
      LocalShapeInfo ret = (cPtr == IntPtr.Zero) ? null : new LocalShapeInfo(cPtr, false);
      return ret;
    } 
  }

  public SWIGTYPE_p_btVector3 m_hitNormalLocal {
    set {
      BulletSoftBodyPINVOKE.LocalRayResult_m_hitNormalLocal_set(swigCPtr, SWIGTYPE_p_btVector3.getCPtr(value));
      if (BulletSoftBodyPINVOKE.SWIGPendingException.Pending) throw BulletSoftBodyPINVOKE.SWIGPendingException.Retrieve();
    } 
    get {
      SWIGTYPE_p_btVector3 ret = new SWIGTYPE_p_btVector3(BulletSoftBodyPINVOKE.LocalRayResult_m_hitNormalLocal_get(swigCPtr), true);
      if (BulletSoftBodyPINVOKE.SWIGPendingException.Pending) throw BulletSoftBodyPINVOKE.SWIGPendingException.Retrieve();
      return ret;
    } 
  }

  public float m_hitFraction {
    set {
      BulletSoftBodyPINVOKE.LocalRayResult_m_hitFraction_set(swigCPtr, value);
    } 
    get {
      float ret = BulletSoftBodyPINVOKE.LocalRayResult_m_hitFraction_get(swigCPtr);
      return ret;
    } 
  }

}

}