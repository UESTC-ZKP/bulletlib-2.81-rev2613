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

public class SoftBodyLinkData : IDisposable {
  private HandleRef swigCPtr;
  protected bool swigCMemOwn;

  internal SoftBodyLinkData(IntPtr cPtr, bool cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = new HandleRef(this, cPtr);
  }

  internal static HandleRef getCPtr(SoftBodyLinkData obj) {
    return (obj == null) ? new HandleRef(null, IntPtr.Zero) : obj.swigCPtr;
  }

  ~SoftBodyLinkData() {
    Dispose();
  }

  public virtual void Dispose() {
    lock(this) {
      if (swigCPtr.Handle != IntPtr.Zero) {
        if (swigCMemOwn) {
          swigCMemOwn = false;
          BulletSoftBodyPINVOKE.delete_SoftBodyLinkData(swigCPtr);
        }
        swigCPtr = new HandleRef(null, IntPtr.Zero);
      }
      GC.SuppressFinalize(this);
    }
  }

  public SoftBodyMaterialData m_material {
    set {
      BulletSoftBodyPINVOKE.SoftBodyLinkData_m_material_set(swigCPtr, SoftBodyMaterialData.getCPtr(value));
    } 
    get {
      IntPtr cPtr = BulletSoftBodyPINVOKE.SoftBodyLinkData_m_material_get(swigCPtr);
      SoftBodyMaterialData ret = (cPtr == IntPtr.Zero) ? null : new SoftBodyMaterialData(cPtr, false);
      return ret;
    } 
  }

  public SWIGTYPE_p_int m_nodeIndices {
    set {
      BulletSoftBodyPINVOKE.SoftBodyLinkData_m_nodeIndices_set(swigCPtr, SWIGTYPE_p_int.getCPtr(value));
    } 
    get {
      IntPtr cPtr = BulletSoftBodyPINVOKE.SoftBodyLinkData_m_nodeIndices_get(swigCPtr);
      SWIGTYPE_p_int ret = (cPtr == IntPtr.Zero) ? null : new SWIGTYPE_p_int(cPtr, false);
      return ret;
    } 
  }

  public float m_restLength {
    set {
      BulletSoftBodyPINVOKE.SoftBodyLinkData_m_restLength_set(swigCPtr, value);
    } 
    get {
      float ret = BulletSoftBodyPINVOKE.SoftBodyLinkData_m_restLength_get(swigCPtr);
      return ret;
    } 
  }

  public int m_bbending {
    set {
      BulletSoftBodyPINVOKE.SoftBodyLinkData_m_bbending_set(swigCPtr, value);
    } 
    get {
      int ret = BulletSoftBodyPINVOKE.SoftBodyLinkData_m_bbending_get(swigCPtr);
      return ret;
    } 
  }

  public SoftBodyLinkData() : this(BulletSoftBodyPINVOKE.new_SoftBodyLinkData(), true) {
  }

}

}
