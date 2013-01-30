/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 2.0.8
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */


using System;
using System.Runtime.InteropServices;

public class vecdouble : IDisposable {
  private HandleRef swigCPtr;
  protected bool swigCMemOwn;

  internal vecdouble(IntPtr cPtr, bool cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = new HandleRef(this, cPtr);
  }

  internal static HandleRef getCPtr(vecdouble obj) {
    return (obj == null) ? new HandleRef(null, IntPtr.Zero) : obj.swigCPtr;
  }

  ~vecdouble() {
    Dispose();
  }

  public virtual void Dispose() {
    lock(this) {
      if (swigCPtr.Handle != IntPtr.Zero) {
        if (swigCMemOwn) {
          swigCMemOwn = false;
          examplePINVOKE.delete_vecdouble(swigCPtr);
        }
        swigCPtr = new HandleRef(null, IntPtr.Zero);
      }
      GC.SuppressFinalize(this);
    }
  }

  public vecdouble(int _sz) : this(examplePINVOKE.new_vecdouble(_sz), true) {
  }

  public SWIGTYPE_p_double get(int index) {
    SWIGTYPE_p_double ret = new SWIGTYPE_p_double(examplePINVOKE.vecdouble_get(swigCPtr, index), false);
    return ret;
  }

  public void set(int index, SWIGTYPE_p_double val) {
    examplePINVOKE.vecdouble_set(swigCPtr, index, SWIGTYPE_p_double.getCPtr(val));
    if (examplePINVOKE.SWIGPendingException.Pending) throw examplePINVOKE.SWIGPendingException.Retrieve();
  }

  public double getitem(int index) {
    double ret = examplePINVOKE.vecdouble_getitem(swigCPtr, index);
    return ret;
  }

  public void setitem(int index, double val) {
    examplePINVOKE.vecdouble_setitem(swigCPtr, index, val);
  }

}