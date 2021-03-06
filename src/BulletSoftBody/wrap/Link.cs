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

public class Link : Feature {
  private HandleRef swigCPtr;

  internal Link(IntPtr cPtr, bool cMemoryOwn) : base(BulletSoftBodyPINVOKE.Link_SWIGUpcast(cPtr), cMemoryOwn) {
    swigCPtr = new HandleRef(this, cPtr);
  }

  internal static HandleRef getCPtr(Link obj) {
    return (obj == null) ? new HandleRef(null, IntPtr.Zero) : obj.swigCPtr;
  }

  ~Link() {
    Dispose();
  }

  public override void Dispose() {
    lock(this) {
      if (swigCPtr.Handle != IntPtr.Zero) {
        if (swigCMemOwn) {
          swigCMemOwn = false;
          BulletSoftBodyPINVOKE.delete_Link(swigCPtr);
        }
        swigCPtr = new HandleRef(null, IntPtr.Zero);
      }
      GC.SuppressFinalize(this);
      base.Dispose();
    }
  }

  public SWIGTYPE_p_p_Node m_n {
    set {
      BulletSoftBodyPINVOKE.Link_m_n_set(swigCPtr, SWIGTYPE_p_p_Node.getCPtr(value));
    } 
    get {
      IntPtr cPtr = BulletSoftBodyPINVOKE.Link_m_n_get(swigCPtr);
      SWIGTYPE_p_p_Node ret = (cPtr == IntPtr.Zero) ? null : new SWIGTYPE_p_p_Node(cPtr, false);
      return ret;
    } 
  }

  public float m_rl {
    set {
      BulletSoftBodyPINVOKE.Link_m_rl_set(swigCPtr, value);
    } 
    get {
      float ret = BulletSoftBodyPINVOKE.Link_m_rl_get(swigCPtr);
      return ret;
    } 
  }

  public int m_bbending {
    set {
      BulletSoftBodyPINVOKE.Link_m_bbending_set(swigCPtr, value);
    } 
    get {
      int ret = BulletSoftBodyPINVOKE.Link_m_bbending_get(swigCPtr);
      return ret;
    } 
  }

  public float m_c0 {
    set {
      BulletSoftBodyPINVOKE.Link_m_c0_set(swigCPtr, value);
    } 
    get {
      float ret = BulletSoftBodyPINVOKE.Link_m_c0_get(swigCPtr);
      return ret;
    } 
  }

  public float m_c1 {
    set {
      BulletSoftBodyPINVOKE.Link_m_c1_set(swigCPtr, value);
    } 
    get {
      float ret = BulletSoftBodyPINVOKE.Link_m_c1_get(swigCPtr);
      return ret;
    } 
  }

  public float m_c2 {
    set {
      BulletSoftBodyPINVOKE.Link_m_c2_set(swigCPtr, value);
    } 
    get {
      float ret = BulletSoftBodyPINVOKE.Link_m_c2_get(swigCPtr);
      return ret;
    } 
  }

  public SWIGTYPE_p_btVector3 m_c3 {
    set {
      BulletSoftBodyPINVOKE.Link_m_c3_set(swigCPtr, SWIGTYPE_p_btVector3.getCPtr(value));
      if (BulletSoftBodyPINVOKE.SWIGPendingException.Pending) throw BulletSoftBodyPINVOKE.SWIGPendingException.Retrieve();
    } 
    get {
      SWIGTYPE_p_btVector3 ret = new SWIGTYPE_p_btVector3(BulletSoftBodyPINVOKE.Link_m_c3_get(swigCPtr), true);
      if (BulletSoftBodyPINVOKE.SWIGPendingException.Pending) throw BulletSoftBodyPINVOKE.SWIGPendingException.Retrieve();
      return ret;
    } 
  }

  public Link() : this(BulletSoftBodyPINVOKE.new_Link(), true) {
  }

}

}
