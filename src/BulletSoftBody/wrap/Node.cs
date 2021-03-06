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

public class Node : Feature {
  private HandleRef swigCPtr;

  internal Node(IntPtr cPtr, bool cMemoryOwn) : base(BulletSoftBodyPINVOKE.Node_SWIGUpcast(cPtr), cMemoryOwn) {
    swigCPtr = new HandleRef(this, cPtr);
  }

  internal static HandleRef getCPtr(Node obj) {
    return (obj == null) ? new HandleRef(null, IntPtr.Zero) : obj.swigCPtr;
  }

  ~Node() {
    Dispose();
  }

  public override void Dispose() {
    lock(this) {
      if (swigCPtr.Handle != IntPtr.Zero) {
        if (swigCMemOwn) {
          swigCMemOwn = false;
          BulletSoftBodyPINVOKE.delete_Node(swigCPtr);
        }
        swigCPtr = new HandleRef(null, IntPtr.Zero);
      }
      GC.SuppressFinalize(this);
      base.Dispose();
    }
  }

  public SWIGTYPE_p_btVector3 m_x {
    set {
      BulletSoftBodyPINVOKE.Node_m_x_set(swigCPtr, SWIGTYPE_p_btVector3.getCPtr(value));
      if (BulletSoftBodyPINVOKE.SWIGPendingException.Pending) throw BulletSoftBodyPINVOKE.SWIGPendingException.Retrieve();
    } 
    get {
      SWIGTYPE_p_btVector3 ret = new SWIGTYPE_p_btVector3(BulletSoftBodyPINVOKE.Node_m_x_get(swigCPtr), true);
      if (BulletSoftBodyPINVOKE.SWIGPendingException.Pending) throw BulletSoftBodyPINVOKE.SWIGPendingException.Retrieve();
      return ret;
    } 
  }

  public SWIGTYPE_p_btVector3 m_q {
    set {
      BulletSoftBodyPINVOKE.Node_m_q_set(swigCPtr, SWIGTYPE_p_btVector3.getCPtr(value));
      if (BulletSoftBodyPINVOKE.SWIGPendingException.Pending) throw BulletSoftBodyPINVOKE.SWIGPendingException.Retrieve();
    } 
    get {
      SWIGTYPE_p_btVector3 ret = new SWIGTYPE_p_btVector3(BulletSoftBodyPINVOKE.Node_m_q_get(swigCPtr), true);
      if (BulletSoftBodyPINVOKE.SWIGPendingException.Pending) throw BulletSoftBodyPINVOKE.SWIGPendingException.Retrieve();
      return ret;
    } 
  }

  public SWIGTYPE_p_btVector3 m_v {
    set {
      BulletSoftBodyPINVOKE.Node_m_v_set(swigCPtr, SWIGTYPE_p_btVector3.getCPtr(value));
      if (BulletSoftBodyPINVOKE.SWIGPendingException.Pending) throw BulletSoftBodyPINVOKE.SWIGPendingException.Retrieve();
    } 
    get {
      SWIGTYPE_p_btVector3 ret = new SWIGTYPE_p_btVector3(BulletSoftBodyPINVOKE.Node_m_v_get(swigCPtr), true);
      if (BulletSoftBodyPINVOKE.SWIGPendingException.Pending) throw BulletSoftBodyPINVOKE.SWIGPendingException.Retrieve();
      return ret;
    } 
  }

  public SWIGTYPE_p_btVector3 m_f {
    set {
      BulletSoftBodyPINVOKE.Node_m_f_set(swigCPtr, SWIGTYPE_p_btVector3.getCPtr(value));
      if (BulletSoftBodyPINVOKE.SWIGPendingException.Pending) throw BulletSoftBodyPINVOKE.SWIGPendingException.Retrieve();
    } 
    get {
      SWIGTYPE_p_btVector3 ret = new SWIGTYPE_p_btVector3(BulletSoftBodyPINVOKE.Node_m_f_get(swigCPtr), true);
      if (BulletSoftBodyPINVOKE.SWIGPendingException.Pending) throw BulletSoftBodyPINVOKE.SWIGPendingException.Retrieve();
      return ret;
    } 
  }

  public SWIGTYPE_p_btVector3 m_n {
    set {
      BulletSoftBodyPINVOKE.Node_m_n_set(swigCPtr, SWIGTYPE_p_btVector3.getCPtr(value));
      if (BulletSoftBodyPINVOKE.SWIGPendingException.Pending) throw BulletSoftBodyPINVOKE.SWIGPendingException.Retrieve();
    } 
    get {
      SWIGTYPE_p_btVector3 ret = new SWIGTYPE_p_btVector3(BulletSoftBodyPINVOKE.Node_m_n_get(swigCPtr), true);
      if (BulletSoftBodyPINVOKE.SWIGPendingException.Pending) throw BulletSoftBodyPINVOKE.SWIGPendingException.Retrieve();
      return ret;
    } 
  }

  public float m_im {
    set {
      BulletSoftBodyPINVOKE.Node_m_im_set(swigCPtr, value);
    } 
    get {
      float ret = BulletSoftBodyPINVOKE.Node_m_im_get(swigCPtr);
      return ret;
    } 
  }

  public float m_area {
    set {
      BulletSoftBodyPINVOKE.Node_m_area_set(swigCPtr, value);
    } 
    get {
      float ret = BulletSoftBodyPINVOKE.Node_m_area_get(swigCPtr);
      return ret;
    } 
  }

  public SWIGTYPE_p_btDbvtNode m_leaf {
    set {
      BulletSoftBodyPINVOKE.Node_m_leaf_set(swigCPtr, SWIGTYPE_p_btDbvtNode.getCPtr(value));
    } 
    get {
      IntPtr cPtr = BulletSoftBodyPINVOKE.Node_m_leaf_get(swigCPtr);
      SWIGTYPE_p_btDbvtNode ret = (cPtr == IntPtr.Zero) ? null : new SWIGTYPE_p_btDbvtNode(cPtr, false);
      return ret;
    } 
  }

  public int m_battach {
    set {
      BulletSoftBodyPINVOKE.Node_m_battach_set(swigCPtr, value);
    } 
    get {
      int ret = BulletSoftBodyPINVOKE.Node_m_battach_get(swigCPtr);
      return ret;
    } 
  }

  public Node() : this(BulletSoftBodyPINVOKE.new_Node(), true) {
  }

}

}
