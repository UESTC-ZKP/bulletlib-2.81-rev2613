%module BulletCollision

%{
    #include "BroadphaseCollision/btAxisSweep3.h"
	#include "BroadphaseCollision/btBroadphaseInterface.h"
	#include "BroadphaseCollision/btBroadphaseProxy.h"
	#include "BroadphaseCollision/btCollisionAlgorithm.h"
	#include "BroadphaseCollision/btDbvt.h"
	#include "BroadphaseCollision/btDbvtBroadphase.h"
	#include "BroadphaseCollision/btDispatcher.h"
	#include "BroadphaseCollision/btMultiSapBroadphase.h"
	#include "BroadphaseCollision/btOverlappingPairCache.h"
	#include "BroadphaseCollision/btOverlappingPairCallback.h"
	#include "BroadphaseCollision/btQuantizedBvh.h"
	#include "BroadphaseCollision/btSimpleBroadphase.h"

	#include "CollisionDispatch/btActivatingCollisionAlgorithm.h"
	#include "CollisionDispatch/btBoxBoxCollisionAlgorithm.h"
	#include "CollisionDispatch/btBox2dBox2dCollisionAlgorithm.h"
	#include "CollisionDispatch/btBoxBoxDetector.h"
	#include "CollisionDispatch/btCollisionConfiguration.h"
	#include "CollisionDispatch/btCollisionCreateFunc.h"
	#include "CollisionDispatch/btCollisionDispatcher.h"
	#include "CollisionDispatch/btCollisionObject.h"
	#include "CollisionDispatch/btCollisionWorld.h"
	#include "CollisionDispatch/btCompoundCollisionAlgorithm.h"
	#include "CollisionDispatch/btConvexConcaveCollisionAlgorithm.h"
	#include "CollisionDispatch/btConvexConvexAlgorithm.h"
	#include "CollisionDispatch/btConvex2dConvex2dAlgorithm.h"
	#include "CollisionDispatch/btConvexPlaneCollisionAlgorithm.h"
	#include "CollisionDispatch/btDefaultCollisionConfiguration.h"
	#include "CollisionDispatch/btEmptyCollisionAlgorithm.h"
	#include "CollisionDispatch/btGhostObject.h"
	#include "CollisionDispatch/btManifoldResult.h"
	#include "CollisionDispatch/btSimulationIslandManager.h"
	#include "CollisionDispatch/btSphereBoxCollisionAlgorithm.h"
	#include "CollisionDispatch/btSphereSphereCollisionAlgorithm.h"
	#include "CollisionDispatch/btSphereTriangleCollisionAlgorithm.h"
	#include "CollisionDispatch/btUnionFind.h"
	#include "CollisionDispatch/SphereTriangleDetector.h"

	#include "CollisionShapes/btBoxShape.h"
	#include "CollisionShapes/btBox2dShape.h"
	#include "CollisionShapes/btBvhTriangleMeshShape.h"
	#include "CollisionShapes/btCapsuleShape.h"
	#include "CollisionShapes/btCollisionMargin.h"
	#include "CollisionShapes/btCollisionShape.h"
	#include "CollisionShapes/btCompoundShape.h"
	#include "CollisionShapes/btConcaveShape.h"
	#include "CollisionShapes/btConeShape.h"
	#include "CollisionShapes/btConvexHullShape.h"
	#include "CollisionShapes/btConvexInternalShape.h"
	#include "CollisionShapes/btConvexPointCloudShape.h"
	#include "CollisionShapes/btConvexPolyhedron.h"
	#include "CollisionShapes/btConvexShape.h"
	#include "CollisionShapes/btConvex2dShape.h"
	#include "CollisionShapes/btConvexTriangleMeshShape.h"
	#include "CollisionShapes/btCylinderShape.h"
	#include "CollisionShapes/btEmptyShape.h"
	#include "CollisionShapes/btHeightfieldTerrainShape.h"
	#include "CollisionShapes/btMaterial.h"
	#include "CollisionShapes/btMinkowskiSumShape.h"
	#include "CollisionShapes/btMultimaterialTriangleMeshShape.h"
	#include "CollisionShapes/btMultiSphereShape.h"
	#include "CollisionShapes/btOptimizedBvh.h"
	#include "CollisionShapes/btPolyhedralConvexShape.h"
	#include "CollisionShapes/btScaledBvhTriangleMeshShape.h"
	#include "CollisionShapes/btShapeHull.h"
	#include "CollisionShapes/btSphereShape.h"
	#include "CollisionShapes/btStaticPlaneShape.h"
	#include "CollisionShapes/btStridingMeshInterface.h"
	#include "CollisionShapes/btTetrahedronShape.h"
	#include "CollisionShapes/btTriangleBuffer.h"
	#include "CollisionShapes/btTriangleCallback.h"
	#include "CollisionShapes/btTriangleIndexVertexArray.h"
	#include "CollisionShapes/btTriangleIndexVertexMaterialArray.h"
	#include "CollisionShapes/btTriangleInfoMap.h"
	#include "CollisionShapes/btTriangleMesh.h"
	#include "CollisionShapes/btTriangleMeshShape.h"
	#include "CollisionShapes/btTriangleShape.h"
	#include "CollisionShapes/btUniformScalingShape.h"

	#include "Gimpact/btBoxCollision.h
	#include "Gimpact/btClipPolygon.h
	#include "Gimpact/btContactProcessing.h
	#include "Gimpact/btGenericPoolAllocator.h
	#include "Gimpact/btGeometryOperations.h
	#include "Gimpact/btGImpactBvh.h
	#include "Gimpact/btGImpactCollisionAlgorithm.h
	#include "Gimpact/btGImpactMassUtil.h
	#include "Gimpact/btGImpactQuantizedBvh.h
	#include "Gimpact/btGImpactShape.h
	#include "Gimpact/btQuantization.h
	#include "Gimpact/btTriangleShapeEx.h
	#include "Gimpact/gim_array.h
	#include "Gimpact/gim_basic_geometry_operations.h
	#include "Gimpact/gim_bitset.h
	#include "Gimpact/gim_box_collision.h
	#include "Gimpact/gim_box_set.h
	#include "Gimpact/gim_clip_polygon.h
	#include "Gimpact/gim_contact.h
	#include "Gimpact/gim_geom_types.h
	#include "Gimpact/gim_geometry.h
	#include "Gimpact/gim_hash_table.h
	#include "Gimpact/gim_linear_math.h
	#include "Gimpact/gim_math.h
	#include "Gimpact/gim_memory.h
	#include "Gimpact/gim_radixsort.h
	#include "Gimpact/gim_tri_collision.h

	NarrowPhaseCollision/btContinuousConvexCollision.h
	NarrowPhaseCollision/btConvexCast.h
	NarrowPhaseCollision/btConvexPenetrationDepthSolver.h
	NarrowPhaseCollision/btDiscreteCollisionDetectorInterface.h
	NarrowPhaseCollision/btGjkConvexCast.h
	NarrowPhaseCollision/btGjkEpa2.h
	NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h
	NarrowPhaseCollision/btGjkPairDetector.h
	NarrowPhaseCollision/btManifoldPoint.h
	NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.h
	NarrowPhaseCollision/btPersistentManifold.h
	NarrowPhaseCollision/btPointCollector.h
	NarrowPhaseCollision/btRaycastCallback.h
	NarrowPhaseCollision/btSimplexSolverInterface.h
	NarrowPhaseCollision/btSubSimplexConvexCast.h
	NarrowPhaseCollision/btVoronoiSimplexSolver.h
	NarrowPhaseCollision/btPolyhedralContactClipping.h
%}