/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 2.0.8
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */


using System;
using System.Runtime.InteropServices;

public class example {
  public static void myArrayCopy(SWIGTYPE_p_int sourceArray, SWIGTYPE_p_int targetArray, int nitems) {
    examplePINVOKE.myArrayCopy(SWIGTYPE_p_int.getCPtr(sourceArray), SWIGTYPE_p_int.getCPtr(targetArray), nitems);
  }

  public static void myArraySwap(SWIGTYPE_p_int array1, SWIGTYPE_p_int array2, int nitems) {
    examplePINVOKE.myArraySwap(SWIGTYPE_p_int.getCPtr(array1), SWIGTYPE_p_int.getCPtr(array2), nitems);
  }

}
