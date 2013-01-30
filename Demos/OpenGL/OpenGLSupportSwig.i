%module OpenGLSupport

%{
        #include "GLDebugFont.h"
		#include "GLDebugFont.h"
		#include "GL_DialogDynamicsWorld.h"
		#include "GL_DialogWindow.h"
		#include "GL_ShapeDrawer.h"
		#include "GL_Simplex1to4.h"
		#include "GLDebugDrawer.h"
		
		#include "RenderTexture.h"
		#include "DemoApplication.h"
		
		#include "GlutDemoApplication.h"
		#include "GlutStuff.h"

		#include "stb_image.h"
		#include "Win32DemoApplication.h"

%}

 #define _WIN32
%import "../../src/LinearMath/btScalar.h"
%include "GLDebugFont.h"
%include "GLDebugFont.h"
%include "GL_DialogDynamicsWorld.h"
%include "GL_DialogWindow.h"
%include "GL_ShapeDrawer.h"
%include "GL_Simplex1to4.h"
%include "GLDebugDrawer.h"
		
%include "RenderTexture.h"
%include "DemoApplication.h"
		
%include "GlutDemoApplication.h"
%include "GlutStuff.h"

%include "stb_image.h"
%include "Win32DemoApplication.h"