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

public class btRigidBodyConstructionInfo : IDisposable {
  private HandleRef swigCPtr;
  protected bool swigCMemOwn;

  internal btRigidBodyConstructionInfo(IntPtr cPtr, bool cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = new HandleRef(this, cPtr);
  }

  internal static HandleRef getCPtr(btRigidBodyConstructionInfo obj) {
    return (obj == null) ? new HandleRef(null, IntPtr.Zero) : obj.swigCPtr;
  }

  ~btRigidBodyConstructionInfo() {
    Dispose();
  }

  public virtual void Dispose() {
    lock(this) {
      if (swigCPtr.Handle != IntPtr.Zero) {
        if (swigCMemOwn) {
          swigCMemOwn = false;
          BulletDynamicsPINVOKE.delete_btRigidBodyConstructionInfo(swigCPtr);
        }
        swigCPtr = new HandleRef(null, IntPtr.Zero);
      }
      GC.SuppressFinalize(this);
    }
  }

  public float m_mass {
    set {
      BulletDynamicsPINVOKE.btRigidBodyConstructionInfo_m_mass_set(swigCPtr, value);
    } 
    get {
      float ret = BulletDynamicsPINVOKE.btRigidBodyConstructionInfo_m_mass_get(swigCPtr);
      return ret;
    } 
  }

  public SWIGTYPE_p_btMotionState m_motionState {
    set {
      BulletDynamicsPINVOKE.btRigidBodyConstructionInfo_m_motionState_set(swigCPtr, SWIGTYPE_p_btMotionState.getCPtr(value));
    } 
    get {
      IntPtr cPtr = BulletDynamicsPINVOKE.btRigidBodyConstructionInfo_m_motionState_get(swigCPtr);
      SWIGTYPE_p_btMotionState ret = (cPtr == IntPtr.Zero) ? null : new SWIGTYPE_p_btMotionState(cPtr, false);
      return ret;
    } 
  }

  public SWIGTYPE_p_btTransform m_startWorldTransform {
    set {
      BulletDynamicsPINVOKE.btRigidBodyConstructionInfo_m_startWorldTransform_set(swigCPtr, SWIGTYPE_p_btTransform.getCPtr(value));
      if (BulletDynamicsPINVOKE.SWIGPendingException.Pending) throw BulletDynamicsPINVOKE.SWIGPendingException.Retrieve();
    } 
    get {
      SWIGTYPE_p_btTransform ret = new SWIGTYPE_p_btTransform(BulletDynamicsPINVOKE.btRigidBodyConstructionInfo_m_startWorldTransform_get(swigCPtr), true);
      if (BulletDynamicsPINVOKE.SWIGPendingException.Pending) throw BulletDynamicsPINVOKE.SWIGPendingException.Retrieve();
      return ret;
    } 
  }

  public SWIGTYPE_p_btCollisionShape m_collisionShape {
    set {
      BulletDynamicsPINVOKE.btRigidBodyConstructionInfo_m_collisionShape_set(swigCPtr, SWIGTYPE_p_btCollisionShape.getCPtr(value));
    } 
    get {
      IntPtr cPtr = BulletDynamicsPINVOKE.btRigidBodyConstructionInfo_m_collisionShape_get(swigCPtr);
      SWIGTYPE_p_btCollisionShape ret = (cPtr == IntPtr.Zero) ? null : new SWIGTYPE_p_btCollisionShape(cPtr, false);
      return ret;
    } 
  }

  public SWIGTYPE_p_btVector3 m_localInertia {
    set {
      BulletDynamicsPINVOKE.btRigidBodyConstructionInfo_m_localInertia_set(swigCPtr, SWIGTYPE_p_btVector3.getCPtr(value));
      if (BulletDynamicsPINVOKE.SWIGPendingException.Pending) throw BulletDynamicsPINVOKE.SWIGPendingException.Retrieve();
    } 
    get {
      SWIGTYPE_p_btVector3 ret = new SWIGTYPE_p_btVector3(BulletDynamicsPINVOKE.btRigidBodyConstructionInfo_m_localInertia_get(swigCPtr), true);
      if (BulletDynamicsPINVOKE.SWIGPendingException.Pending) throw BulletDynamicsPINVOKE.SWIGPendingException.Retrieve();
      return ret;
    } 
  }

  public float m_linearDamping {
    set {
      BulletDynamicsPINVOKE.btRigidBodyConstructionInfo_m_linearDamping_set(swigCPtr, value);
    } 
    get {
      float ret = BulletDynamicsPINVOKE.btRigidBodyConstructionInfo_m_linearDamping_get(swigCPtr);
      return ret;
    } 
  }

  public float m_angularDamping {
    set {
      BulletDynamicsPINVOKE.btRigidBodyConstructionInfo_m_angularDamping_set(swigCPtr, value);
    } 
    get {
      float ret = BulletDynamicsPINVOKE.btRigidBodyConstructionInfo_m_angularDamping_get(swigCPtr);
      return ret;
    } 
  }

  public float m_friction {
    set {
      BulletDynamicsPINVOKE.btRigidBodyConstructionInfo_m_friction_set(swigCPtr, value);
    } 
    get {
      float ret = BulletDynamicsPINVOKE.btRigidBodyConstructionInfo_m_friction_get(swigCPtr);
      return ret;
    } 
  }

  public float m_restitution {
    set {
      BulletDynamicsPINVOKE.btRigidBodyConstructionInfo_m_restitution_set(swigCPtr, value);
    } 
    get {
      float ret = BulletDynamicsPINVOKE.btRigidBodyConstructionInfo_m_restitution_get(swigCPtr);
      return ret;
    } 
  }

  public float m_linearSleepingThreshold {
    set {
      BulletDynamicsPINVOKE.btRigidBodyConstructionInfo_m_linearSleepingThreshold_set(swigCPtr, value);
    } 
    get {
      float ret = BulletDynamicsPINVOKE.btRigidBodyConstructionInfo_m_linearSleepingThreshold_get(swigCPtr);
      return ret;
    } 
  }

  public float m_angularSleepingThreshold {
    set {
      BulletDynamicsPINVOKE.btRigidBodyConstructionInfo_m_angularSleepingThreshold_set(swigCPtr, value);
    } 
    get {
      float ret = BulletDynamicsPINVOKE.btRigidBodyConstructionInfo_m_angularSleepingThreshold_get(swigCPtr);
      return ret;
    } 
  }

  public bool m_additionalDamping {
    set {
      BulletDynamicsPINVOKE.btRigidBodyConstructionInfo_m_additionalDamping_set(swigCPtr, value);
    } 
    get {
      bool ret = BulletDynamicsPINVOKE.btRigidBodyConstructionInfo_m_additionalDamping_get(swigCPtr);
      return ret;
    } 
  }

  public float m_additionalDampingFactor {
    set {
      BulletDynamicsPINVOKE.btRigidBodyConstructionInfo_m_additionalDampingFactor_set(swigCPtr, value);
    } 
    get {
      float ret = BulletDynamicsPINVOKE.btRigidBodyConstructionInfo_m_additionalDampingFactor_get(swigCPtr);
      return ret;
    } 
  }

  public float m_additionalLinearDampingThresholdSqr {
    set {
      BulletDynamicsPINVOKE.btRigidBodyConstructionInfo_m_additionalLinearDampingThresholdSqr_set(swigCPtr, value);
    } 
    get {
      float ret = BulletDynamicsPINVOKE.btRigidBodyConstructionInfo_m_additionalLinearDampingThresholdSqr_get(swigCPtr);
      return ret;
    } 
  }

  public float m_additionalAngularDampingThresholdSqr {
    set {
      BulletDynamicsPINVOKE.btRigidBodyConstructionInfo_m_additionalAngularDampingThresholdSqr_set(swigCPtr, value);
    } 
    get {
      float ret = BulletDynamicsPINVOKE.btRigidBodyConstructionInfo_m_additionalAngularDampingThresholdSqr_get(swigCPtr);
      return ret;
    } 
  }

  public float m_additionalAngularDampingFactor {
    set {
      BulletDynamicsPINVOKE.btRigidBodyConstructionInfo_m_additionalAngularDampingFactor_set(swigCPtr, value);
    } 
    get {
      float ret = BulletDynamicsPINVOKE.btRigidBodyConstructionInfo_m_additionalAngularDampingFactor_get(swigCPtr);
      return ret;
    } 
  }

  public btRigidBodyConstructionInfo(float mass, SWIGTYPE_p_btMotionState motionState, SWIGTYPE_p_btCollisionShape collisionShape, SWIGTYPE_p_btVector3 localInertia) : this(BulletDynamicsPINVOKE.new_btRigidBodyConstructionInfo__SWIG_0(mass, SWIGTYPE_p_btMotionState.getCPtr(motionState), SWIGTYPE_p_btCollisionShape.getCPtr(collisionShape), SWIGTYPE_p_btVector3.getCPtr(localInertia)), true) {
    if (BulletDynamicsPINVOKE.SWIGPendingException.Pending) throw BulletDynamicsPINVOKE.SWIGPendingException.Retrieve();
  }

  public btRigidBodyConstructionInfo(float mass, SWIGTYPE_p_btMotionState motionState, SWIGTYPE_p_btCollisionShape collisionShape) : this(BulletDynamicsPINVOKE.new_btRigidBodyConstructionInfo__SWIG_1(mass, SWIGTYPE_p_btMotionState.getCPtr(motionState), SWIGTYPE_p_btCollisionShape.getCPtr(collisionShape)), true) {
  }

}

}
