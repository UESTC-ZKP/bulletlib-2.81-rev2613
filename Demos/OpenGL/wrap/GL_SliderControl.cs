/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 2.0.8
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */


using System;
using System.Runtime.InteropServices;

public class GL_SliderControl : GL_DialogControl {
  private HandleRef swigCPtr;

  internal GL_SliderControl(IntPtr cPtr, bool cMemoryOwn) : base(OpenGLSupportPINVOKE.GL_SliderControl_SWIGUpcast(cPtr), cMemoryOwn) {
    swigCPtr = new HandleRef(this, cPtr);
  }

  internal static HandleRef getCPtr(GL_SliderControl obj) {
    return (obj == null) ? new HandleRef(null, IntPtr.Zero) : obj.swigCPtr;
  }

  ~GL_SliderControl() {
    Dispose();
  }

  public override void Dispose() {
    lock(this) {
      if (swigCPtr.Handle != IntPtr.Zero) {
        if (swigCMemOwn) {
          swigCMemOwn = false;
          OpenGLSupportPINVOKE.delete_GL_SliderControl(swigCPtr);
        }
        swigCPtr = new HandleRef(null, IntPtr.Zero);
      }
      GC.SuppressFinalize(this);
      base.Dispose();
    }
  }

  public SWIGTYPE_p_btCollisionObject m_sliderBody {
    set {
      OpenGLSupportPINVOKE.GL_SliderControl_m_sliderBody_set(swigCPtr, SWIGTYPE_p_btCollisionObject.getCPtr(value));
    } 
    get {
      IntPtr cPtr = OpenGLSupportPINVOKE.GL_SliderControl_m_sliderBody_get(swigCPtr);
      SWIGTYPE_p_btCollisionObject ret = (cPtr == IntPtr.Zero) ? null : new SWIGTYPE_p_btCollisionObject(cPtr, false);
      return ret;
    } 
  }

  public GL_DialogWindow m_parentWindow {
    set {
      OpenGLSupportPINVOKE.GL_SliderControl_m_parentWindow_set(swigCPtr, GL_DialogWindow.getCPtr(value));
    } 
    get {
      IntPtr cPtr = OpenGLSupportPINVOKE.GL_SliderControl_m_parentWindow_get(swigCPtr);
      GL_DialogWindow ret = (cPtr == IntPtr.Zero) ? null : new GL_DialogWindow(cPtr, false);
      return ret;
    } 
  }

  public float m_lowerLimit {
    set {
      OpenGLSupportPINVOKE.GL_SliderControl_m_lowerLimit_set(swigCPtr, value);
    } 
    get {
      float ret = OpenGLSupportPINVOKE.GL_SliderControl_m_lowerLimit_get(swigCPtr);
      return ret;
    } 
  }

  public float m_upperLimit {
    set {
      OpenGLSupportPINVOKE.GL_SliderControl_m_upperLimit_set(swigCPtr, value);
    } 
    get {
      float ret = OpenGLSupportPINVOKE.GL_SliderControl_m_upperLimit_get(swigCPtr);
      return ret;
    } 
  }

  public SWIGTYPE_p_btTypedConstraint m_constraint {
    set {
      OpenGLSupportPINVOKE.GL_SliderControl_m_constraint_set(swigCPtr, SWIGTYPE_p_btTypedConstraint.getCPtr(value));
    } 
    get {
      IntPtr cPtr = OpenGLSupportPINVOKE.GL_SliderControl_m_constraint_get(swigCPtr);
      SWIGTYPE_p_btTypedConstraint ret = (cPtr == IntPtr.Zero) ? null : new SWIGTYPE_p_btTypedConstraint(cPtr, false);
      return ret;
    } 
  }

  public float m_fraction {
    set {
      OpenGLSupportPINVOKE.GL_SliderControl_m_fraction_set(swigCPtr, value);
    } 
    get {
      float ret = OpenGLSupportPINVOKE.GL_SliderControl_m_fraction_get(swigCPtr);
      return ret;
    } 
  }

  public string m_sliderText {
    set {
      OpenGLSupportPINVOKE.GL_SliderControl_m_sliderText_set(swigCPtr, value);
    } 
    get {
      string ret = OpenGLSupportPINVOKE.GL_SliderControl_m_sliderText_get(swigCPtr);
      return ret;
    } 
  }

  public GL_SliderControl(string sliderText, SWIGTYPE_p_btCollisionObject sliderBody, GL_DialogWindow parentWindow, float lowerLimit, float upperLimit, SWIGTYPE_p_btTypedConstraint constaint) : this(OpenGLSupportPINVOKE.new_GL_SliderControl(sliderText, SWIGTYPE_p_btCollisionObject.getCPtr(sliderBody), GL_DialogWindow.getCPtr(parentWindow), lowerLimit, upperLimit, SWIGTYPE_p_btTypedConstraint.getCPtr(constaint)), true) {
  }

  public override void draw(SWIGTYPE_p_int parentHorPos, SWIGTYPE_p_int parentVertPos, float deltaTime) {
    OpenGLSupportPINVOKE.GL_SliderControl_draw(swigCPtr, SWIGTYPE_p_int.getCPtr(parentHorPos), SWIGTYPE_p_int.getCPtr(parentVertPos), deltaTime);
    if (OpenGLSupportPINVOKE.SWIGPendingException.Pending) throw OpenGLSupportPINVOKE.SWIGPendingException.Retrieve();
  }

  public float btGetFraction() {
    float ret = OpenGLSupportPINVOKE.GL_SliderControl_btGetFraction(swigCPtr);
    return ret;
  }

  public float getLowerLimit() {
    float ret = OpenGLSupportPINVOKE.GL_SliderControl_getLowerLimit(swigCPtr);
    return ret;
  }

  public float getUpperLimit() {
    float ret = OpenGLSupportPINVOKE.GL_SliderControl_getUpperLimit(swigCPtr);
    return ret;
  }

  public SWIGTYPE_p_btTypedConstraint getConstraint() {
    IntPtr cPtr = OpenGLSupportPINVOKE.GL_SliderControl_getConstraint(swigCPtr);
    SWIGTYPE_p_btTypedConstraint ret = (cPtr == IntPtr.Zero) ? null : new SWIGTYPE_p_btTypedConstraint(cPtr, false);
    return ret;
  }

}